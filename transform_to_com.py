"""Transform vessel motion CSV from a reference point to the ship's COM.

A vessel motion CSV contains a pose time series for a single reference point P
(e.g. a sensor mount near the stern).  This script computes what that time
series would look like at the ship's centre of mass (COM), which is at a fixed
offset from P in the ship's body frame.

Physics
-------
The ship's rigid-body motion at time t is captured by a homogeneous
transformation matrix:

    T(t) = [ R(t)  d_A(t) ]
           [  0      1    ]

where R(t) is the rotation matrix from the CSV Euler angles and d_A(t) is
the displacement of reference point A.  Applying T to the body-frame position
of any fixed point B (expressed as a homogeneous vector [r; 1]) gives its
world-frame position relative to A's equilibrium:

    p_B(t) = T(t) · [r; 1]  =  R(t)·r + d_A(t)

Subtracting the equilibrium position r yields the displacement:

    d_B(t) = p_B(t) − r  =  d_A(t) + (R(t) − I)·r

The Euler angles are a property of the whole rigid body and are unchanged.

Usage
-----
Edit the CONFIGURATION block then run:
    python -m ship_emulation.transform_to_com
"""

import csv
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# ── CONFIGURATION ─────────────────────────────────────────────────────────────

INPUT_CSV  = "ship_emulation/1_vessel_motion_com_1m.csv"
OUTPUT_CSV = "ship_emulation/1_vessel_motion_com_1m_translated_2.csv"

# Vector from the ship COM to the measurement point P (i.e. where P sits in
# the body frame relative to the COM), expressed at rest.
# Units must match the position columns in the CSV (metres for the standard files).
# Positive x = forward, y = port/starboard, z = up — use your ship geometry.
COM_OFFSET = (0.0, 8.0, 6.0)   # (dx, dy, dz) in metres — edit to match geometry

# Euler angle convention used for (roll, pitch, yaw) in the CSV.
# 'xyz' = intrinsic rotations: roll about x, then pitch about rotated y,
#          then yaw about doubly-rotated z.  Standard maritime/ITTC notation.
EULER_CONVENTION = 'xyz'

# Set True if the CSV stores angles in degrees; False for radians.
# The standard vessel motion files use radians.
ANGLES_IN_DEGREES = True

# Rotation that re-expresses vessel-frame data (x forward, y sideways, z up)
# in the robot work frame.  Applied to both position and orientation channels
# after the COM offset, before mean-centering.
# Always specified in degrees; convention follows EULER_CONVENTION above.
FRAME_ROTATION = (-90.0, 0.0, 0.0)   # (rx, ry, rz) in degrees

# Multiplicative scale applied to the linear (position) channels after all
# other processing.  Use 1000.0 to convert metres → millimetres so the output
# CSV is ready for direct use with robot interfaces that expect mm.
# Set to 1.0 to leave units unchanged.
LINEAR_SCALE = 1.0

# Multiplicative scale applied to the angular (orientation) channels after all
# other processing.  Set to 1.0 to leave units unchanged.
ANGULAR_SCALE = 1.0

# ── END CONFIGURATION ─────────────────────────────────────────────────────────


def transform_to_com(
    input_csv: str,
    output_csv: str,
    com_offset,
    euler_convention: str = 'xyz',
    angles_degrees: bool = False,
    frame_rotation=(0., 0., 0.),
    linear_scale: float = 1.0,
    angular_scale: float = 1.0,
) -> None:
    """Read input_csv, translate motion to COM position, write to output_csv."""
    # r is the body-frame vector from P to COM.
    # COM_OFFSET is the vector from COM to P, so r = -COM_OFFSET.
    r = -np.asarray(com_offset, dtype=np.float64)

    def _normalise_row(row):
        # Some CSV exports wrap each row in double-quotes, causing the reader
        # to return the entire row as a single field.  Re-split in that case.
        if len(row) == 1 and ',' in row[0]:
            return row[0].split(',')
        return row

    rows = []
    with open(input_csv, newline='') as f:
        reader = csv.reader(f)
        header = _normalise_row(next(reader))
        for row in reader:
            rows.append([float(v) for v in _normalise_row(row)])

    if not rows:
        print(f"No data rows found in {input_csv!r}.", file=sys.stderr)
        return

    data   = np.array(rows)       # (N, 7)
    d_A    = data[:, 1:4]         # (N, 3) — reference point displacements
    eulers = data[:, 4:7]         # (N, 3) — euler angles (unchanged)

    # Build per-timestep homogeneous transformation matrices T = [R | d_A; 0 1]
    R_all = Rotation.from_euler(euler_convention, eulers, degrees=angles_degrees).as_matrix()
    N = len(data)
    T = np.tile(np.eye(4), (N, 1, 1))   # (N, 4, 4)
    T[:, :3, :3] = R_all
    T[:, :3, 3]  = d_A

    # p_B = T @ [r; 1] gives world-frame position of COM relative to A's equilibrium;
    # subtract r to convert from position to displacement.
    r_hom = np.append(r, 1.0)           # (4,)
    d_com = (T @ r_hom)[:, :3] - r      # (N, 3)

    max_correction = np.abs(d_com - d_A).max(axis=0)  # in original units, before scaling

    # Apply amplitude scaling before re-expressing in the robot work frame.
    # The COM transform above uses the original unscaled angles (physically correct);
    # scaling here adjusts motion amplitudes independently of the frame change.
    if linear_scale != 1.0:
        d_com = d_com * linear_scale
    if angular_scale != 1.0:
        eulers = eulers * angular_scale

    # Re-express vessel-frame data in the robot work frame.
    # Positions rotate as v_work = R_frame @ v_vessel.
    # Orientations rotate as a similarity: R_work = R_frame @ R_vessel @ R_frame^T,
    # which re-expresses the same physical rotation in the new basis.
    # Rotation matrices are rebuilt from the (possibly scaled) eulers so the
    # similarity transform acts on the amplitude-adjusted orientations.
    R_frame = Rotation.from_euler(euler_convention, frame_rotation, degrees=True).as_matrix()
    if not np.allclose(R_frame, np.eye(3)):
        d_com    = (R_frame @ d_com.T).T                        # (N, 3)
        R_scaled = Rotation.from_euler(
            euler_convention, eulers, degrees=angles_degrees
        ).as_matrix()                                           # (N, 3, 3)
        R_work   = R_frame @ R_scaled @ R_frame.T               # (N, 3, 3)
        eulers   = Rotation.from_matrix(R_work).as_euler(
            euler_convention, degrees=angles_degrees
        )                                                        # (N, 3)

    mean_shift = d_com.mean(axis=0)

    out = np.column_stack([data[:, 0:1], d_com, eulers])

    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(out.tolist())

    angle_unit   = "deg" if angles_degrees else "rad"
    csv_unit     = "original units"
    scaled_unit  = f"scaled (×{linear_scale})" if linear_scale != 1.0 else "original units"
    print(f"Input  : {input_csv}  ({N} rows)")
    print(f"Output : {output_csv}")
    print(f"Offset (body frame): dx={r[0]:.4f}  dy={r[1]:.4f}  dz={r[2]:.4f}  [{csv_unit}]")
    print(f"Convention: Euler {euler_convention.upper()}  ({angle_unit})")
    print(f"Linear scale applied: {linear_scale}")
    print(f"Angular scale applied: {angular_scale}")
    print(
        f"Max |Δpos| vs input: "
        f"x={max_correction[0]:.6f}  y={max_correction[1]:.6f}  z={max_correction[2]:.6f}"
        f"  [{csv_unit}]"
    )
    print(
        f"Mean shift removed:  "
        f"x={mean_shift[0]:.6f}  y={mean_shift[1]:.6f}  z={mean_shift[2]:.6f}"
        f"  [{scaled_unit}]"
    )


def main() -> int:
    transform_to_com(
        INPUT_CSV,
        OUTPUT_CSV,
        COM_OFFSET,
        euler_convention=EULER_CONVENTION,
        angles_degrees=ANGLES_IN_DEGREES,
        frame_rotation=FRAME_ROTATION,
        linear_scale=LINEAR_SCALE,
        angular_scale=ANGULAR_SCALE,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
