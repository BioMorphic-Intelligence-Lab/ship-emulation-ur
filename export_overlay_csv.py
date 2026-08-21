"""Export a vessel-motion CSV with the phase2b roll/pitch overlay baked in.

Reproduces the phase2b runtime pipeline (unit scaling + sinusoidal overlay) and
writes the result to a new CSV, then stretches the timestamps to real wall-clock
time for the configured playback speed so the file represents the actual physical
platform motion.

Pipeline (matches phase2b, minus windowing/fade-in):
    CsvFileSource
        -> AugmentedSource        (LINEAR_SCALE m->mm, ANGULAR_SCALE)
            -> SinusoidalOverlaySource (roll/pitch overlay, full amplitude)
                -> timestamps *= 1/PLAYBACK_SPEED

Run:
    python -m ship_emulation.export_overlay_csv
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ship_emulation.config import AugmentationConfig, SinusoidalOverlay
from ship_emulation.data_source import AugmentedSource, CsvFileSource
from ship_emulation.motion_primitives import SinusoidalOverlaySource

# ── CONFIGURATION (mirrors the phase2b run) ───────────────────────────────────

INPUT_CSV  = "ship_emulation/1_vessel_motion_com_1m_translated.csv"
OUTPUT_CSV = "ship_emulation/1_vessel_motion_com_1m_overlay_0.3x.csv"

LINEAR_SCALE  = 1000.0  # m -> mm  (matches poses commanded to the robot)
ANGULAR_SCALE = 1.0

# Overlay (phase2b.py OVERLAY_* values)
OVERLAY_ROLL_AMP   = 10.0   # deg
OVERLAY_ROLL_FREQ  = 0.1    # Hz  (in CSV time)
OVERLAY_PITCH_AMP  = 15.0   # deg
OVERLAY_PITCH_FREQ = 0.07   # Hz  (in CSV time)

# Full file scope -> no windowing, no fade-in ramp.
FADE_ENABLED = False

# Timestamps are multiplied by 1/PLAYBACK_SPEED so the file spans real wall-clock
# time (0.3x -> ~3.33x longer). Overlay waveforms keep their CSV-time shape,
# matching what the robot physically executed.
PLAYBACK_SPEED = 0.3

# ── END CONFIGURATION ─────────────────────────────────────────────────────────


def main() -> int:
    overlay = SinusoidalOverlay(
        roll_amplitude=OVERLAY_ROLL_AMP,
        roll_frequency=OVERLAY_ROLL_FREQ,
        pitch_amplitude=OVERLAY_PITCH_AMP,
        pitch_frequency=OVERLAY_PITCH_FREQ,
    )

    raw = CsvFileSource(INPUT_CSV)
    augmented = AugmentedSource(
        raw, AugmentationConfig(linear_scale=LINEAR_SCALE, angular_scale=ANGULAR_SCALE)
    )
    source = SinusoidalOverlaySource(augmented, overlay, fade_enabled=FADE_ENABLED)

    time_stretch = 1.0 / PLAYBACK_SPEED
    n = 0
    t_first = t_last = None
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "x", "y", "z", "roll", "pitch", "yaw"])
        for p in source.poses():
            t_out = p.timestamp * time_stretch
            if t_first is None:
                t_first = t_out
            t_last = t_out
            writer.writerow([t_out, p.x, p.y, p.z, p.roll, p.pitch, p.yaw])
            n += 1

    span = (t_last - t_first) if t_first is not None else 0.0
    print(f"Input        : {INPUT_CSV}")
    print(f"Output       : {OUTPUT_CSV}  ({n} rows)")
    print(f"Units        : x/y/z in mm (scale {LINEAR_SCALE}), angles in deg")
    print(f"Overlay      : roll {OVERLAY_ROLL_AMP} deg @ {OVERLAY_ROLL_FREQ} Hz, "
          f"pitch {OVERLAY_PITCH_AMP} deg @ {OVERLAY_PITCH_FREQ} Hz (CSV-time freqs)")
    print(f"Playback     : {PLAYBACK_SPEED}x  ->  timestamps stretched x{time_stretch:.4f}")
    print(f"Wall span    : {span:.1f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
