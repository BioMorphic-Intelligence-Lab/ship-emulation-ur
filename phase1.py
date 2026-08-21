"""Phase 1 — DoF-by-DoF TacTip characterisation using trapezoidal velocity profiles.

Experiment flow
---------------
  1. ENTER → connect to robot
  2. ENTER → move to home position
  For each DoF (Rx, Ry, shear-x, shear-y, depth-z):
    3. ENTER → acknowledge axis characterisation start
    For each velocity level (low → high):
      4. ENTER → move slowly to start position
      5. ENTER → begin sweeps
             TRAP_N_CYCLES alternating one-way sweeps, DWELL_SETTLE pause after each
      6. ENTER → return to home position  (last level only)

Edit the CONFIGURATION block below, then run:
    python -m ship_emulation.phase1

Press Ctrl+C at any time to abort safely.
"""
import logging
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ship_emulation.config import EmulatorConfig
from ship_emulation.emulator import EmulationError, servo_run
from ship_emulation.motion_primitives import (
    DwellSource,
    SequentialSource,
    TrapezoidalMoveSource,
)
from ship_emulation.robot_interface import UR16Interface
from ship_emulation.ros_bridge import RosBridge
from ship_emulation.safety import SafetyChecker

# ── CONFIGURATION ─────────────────────────────────────────────────────────────

#ROBOT_IP = "172.17.0.2" # sim
ROBOT_IP = "192.168.56.101" # real
BLEND_RADIUS = 3.0  # mm

# DoF order: Rx (roll), Ry (pitch), shear-x, shear-y, depth-z.  Remove any to skip.
#AXES = ["roll", "pitch", "x", "y", "z"]
AXES = ["y"]
#AXES = ["roll"]

# Sweep start and end positions per axis [mm for linear, deg for angular].
# The robot alternates between SWEEP_START and SWEEP_END for TRAP_N_CYCLES sweeps.
# An axis whose SWEEP_START is non-zero requires a slow approach from home.
SWEEP_START = {
    "roll":  -25.0,   # deg
    "pitch": -45.0,   # deg
    "x":    -350.0,   # mm, y in base frame, this value is in work frame!
    "y":    -300.0,   # mm, -z in base frame, this value is in work frame!
    "z":    -250.0,   # mm, -x in base frame, this value is in work frame!
}
SWEEP_END = {
    "roll":   25.0,   # deg
    "pitch":  45.0,   # deg
    "x":     650.0,   # mm, y in base frame, this value is in work frame!
    "y":     300.0,   # mm, -z in base frame, this value is in work frame!
    "z":     250.0,   # mm, -x in base frame, this value is in work frame!
}

# Acceleration per axis [mm/s² or deg/s²]
TRAP_ACCELERATION = {
    "roll":  15.0,   # deg/s²
    "pitch": 15.0,
    "x":     300.0,  # mm/s²
    "y":     300.0,
    "z":     100.0,
}

# Velocity levels — one entry per level, tested low → high.
# Each level runs all DoFs before moving to the next.
# Angular: deg/s, linear: mm/s.
TRAP_VELOCITIES = [
    {"roll": 5.0,  "pitch": 15.0,  "x":  30.0, "y":  60.0, "z":   200.0},
    #{"roll": 10.0,  "pitch": 15.0,  "x":  125.0, "y":  60.0, "z":   150.0},
    #{"roll": 15.0, "pitch": 15.0, "x":  200.0, "y":  200.0, "z":   300.0},
]

TRAP_N_CYCLES = 4   # one-way sweeps per DoF per level

# Speed for the slow approach from home (0) to sweep start position (−A).
# Keep well below lowest TRAP_VELOCITIES for each axis.
APPROACH_VELOCITY = {
    "roll":  3.0,   # deg/s
    "pitch": 3.0,   # deg/s
    "x":     25.0,  # mm/s
    "y":     25.0,  # mm/s
    "z":     25.0,  # mm/s
}

DWELL_SETTLE = 5.0   # s — pause between consecutive sweeps
SAMPLE_RATE  = 10.0  # Hz

# Acceleration for speedL velocity streaming.  Controls how quickly the robot
# ramps to each commanded segment velocity.  Higher = tighter tracking.
SERVO_ACCEL = 500.0  # mm/s²

# ── END CONFIGURATION ─────────────────────────────────────────────────────────

log = logging.getLogger(__name__)

AXIS_LABELS = {
    "roll":  "Rx (roll)",
    "pitch": "Ry (pitch)",
    "x":     "shear-x",
    "y":     "shear-y",
    "z":     "depth-z",
}

# Axes whose sweep start position is non-zero and therefore require a slow approach.
_APPROACH_AXES = {a for a in AXES if SWEEP_START[a] != 0.0}

# Maps axis name to its index in the (x, y, z, roll, pitch, yaw) pose tuple.
_AXIS_POSE_IDX = {"x": 0, "y": 1, "z": 2, "roll": 3, "pitch": 4, "yaw": 5}

# Speeds used for ALL slow positioning moves — both the initial approach from home
# and direct inter-axis transitions.  Derived from APPROACH_VELOCITY so there is
# one place to tune them.
#
# movel only accepts a single speed parameter (v, in m/s = rad/s for pure rotations).
# angular_speed is only used by movej and has no effect on movel.  For pure-rotation
# approaches (roll, pitch) the robot treats v as rad/s, so the angular limit must be
# expressed in the same units: deg/s → rad/s → m/s → mm/s.
_TRANSITION_LINEAR_SPEED  = min(APPROACH_VELOCITY[a] for a in ("x", "y"))        # mm/s
_TRANSITION_ANGULAR_SPEED = min(APPROACH_VELOCITY[a] for a in ("roll", "pitch"))  # deg/s
_TRANSITION_ANGULAR_AS_LINEAR = _TRANSITION_ANGULAR_SPEED * math.pi / 180 * 1000  # mm/s equiv for movel


def _approach_speed(axis: str) -> float:
    """Return the movel speed (mm/s) for a slow approach along the given axis.

    For angular axes movel's v parameter is interpreted as rad/s, so convert.
    """
    if axis in ("roll", "pitch"):
        return _TRANSITION_ANGULAR_AS_LINEAR
    return APPROACH_VELOCITY[axis]


# ── Helpers ───────────────────────────────────────────────────────────────────

class _Aborted(Exception):
    """Raised on Ctrl+C anywhere in the experiment loop."""


def _enter(prompt: str) -> None:
    """Print prompt, wait for ENTER, raise _Aborted on Ctrl+C."""
    print(prompt)
    try:
        input()
    except KeyboardInterrupt:
        print("\nAborted.")
        raise _Aborted()


def _units(axis: str):
    """Return (velocity_units, position_units) for axis."""
    if axis in ("x", "y", "z"):
        return "mm/s", "mm"
    return "deg/s", "deg"



def _start_pose(axis: str) -> tuple:
    """Work-frame pose (x y z roll pitch yaw, mm/deg) for the sweep start of axis."""
    pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pose[_AXIS_POSE_IDX[axis]] = SWEEP_START[axis]
    return tuple(pose)


# ── Motion sequence builders ──────────────────────────────────────────────────


def _build_sweep_sequence(axis: str, v_max: float) -> SequentialSource:
    """TRAP_N_CYCLES alternating one-way sweeps with a dwell between each.

    Sweeps alternate between SWEEP_START and SWEEP_END.  Each sweep is a
    separate TrapezoidalMoveSource so the robot decelerates fully to rest at
    each endpoint before the dwell begins.
    """
    s0    = SWEEP_START[axis]
    s1    = SWEEP_END[axis]
    accel = TRAP_ACCELERATION[axis]
    _, up = _units(axis)
    label = AXIS_LABELS[axis]

    def _dwell(value: float, tag: str) -> DwellSource:
        src = DwellSource(DWELL_SETTLE, axis=axis, value=value, sample_rate=SAMPLE_RATE)
        src.name = f"Dwell {DWELL_SETTLE:.0f} s  [{tag}]"
        return src

    segments = [_dwell(s0, "pre-sweep settle")]

    current = s0
    for i in range(TRAP_N_CYCLES):
        target = s1 if i % 2 == 0 else s0
        sweep = TrapezoidalMoveSource(axis, [current, target], v_max, accel, sample_rate=SAMPLE_RATE)
        sweep.name = (
            f"Sweep {i + 1}/{TRAP_N_CYCLES}  {label}  "
            f"{current:+g}→{target:+g} {up}  v={v_max} {up}/s"
        )
        segments.append(sweep)
        segments.append(_dwell(target, f"settle after sweep {i + 1}/{TRAP_N_CYCLES}"))
        current = target

    return SequentialSource(segments)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    cfg = EmulatorConfig()
    cfg.robot.ip = ROBOT_IP
    cfg.robot.blend_radius = BLEND_RADIUS

    n_levels = len(TRAP_VELOCITIES)
    n_axes   = len(AXES)

    # ── Startup message ───────────────────────────────────────────────────────
    print(
        "\n" + "═" * 62 + "\n"
        "  Phase 1 — TacTip DoF characterisation\n"
        "  Trapezoidal velocity profiles, axis by axis\n"
        + "═" * 62 + "\n"
        f"  DoFs   : {[AXIS_LABELS[a] for a in AXES]}\n"
        f"  Levels : {n_levels}\n"
        f"  Cycles : {TRAP_N_CYCLES} one-way sweeps per DoF per level\n"
        f"  Dwell  : {DWELL_SETTLE:.0f} s between sweeps\n"
        + "─" * 62 + "\n"
        "  Ensure the workspace is clear.\n"
        "  Keep the physical E-stop within reach at all times.\n"
        + "─" * 62
    )

    bridge = RosBridge("phase1")

    try:
        _enter("\nPress ENTER to connect to the robot, or Ctrl+C to abort.")

        with UR16Interface(cfg.robot) as interface:
            safety = SafetyChecker(cfg.safety)
            bridge.start_feedback(lambda: interface.current_pose)

            # ── Go to home ────────────────────────────────────────────────────
            _enter(
                "\nConnected to robot.\n"
                "Press ENTER to move to home position, or Ctrl+C to abort."
            )
            interface.move_linear_at(
                            cfg.robot.home_pose,
                            _TRANSITION_LINEAR_SPEED,
                            _TRANSITION_ANGULAR_SPEED,
                        )
            interface.wait_until_stopped()
            print("\nAt home position.")

            for axis_idx, axis in enumerate(AXES):
                axis_num  = axis_idx + 1
                s0        = SWEEP_START[axis]
                s1        = SWEEP_END[axis]
                uv, up    = _units(axis)
                label     = AXIS_LABELS[axis]
                start     = _start_pose(axis)
                start_str = f"{s0:+g} {up}" if axis in _APPROACH_AXES else f"0 {up} (home)"

                # ── Axis header ───────────────────────────────────────────────
                vel_lines = "\n".join(
                    f"    Level {i + 1}/{n_levels}:  {v[axis]} {uv}"
                    for i, v in enumerate(TRAP_VELOCITIES)
                )
                _enter(
                    "\n" + "═" * 62 + "\n"
                    f"  Axis {axis_num}/{n_axes}: {label}\n"
                    f"  Range     : {s0:+g} → {s1:+g} {up}\n"
                    f"  Cycles    : {TRAP_N_CYCLES} one-way sweeps per level\n"
                    f"  Velocities:\n"
                    + vel_lines + "\n"
                    + "═" * 62 + "\n"
                    f"Press ENTER to start {label} characterisation, or Ctrl+C to abort."
                )

                for level_idx, velocities in enumerate(TRAP_VELOCITIES):
                    level_num = level_idx + 1
                    v_max     = velocities[axis]
                    is_last   = (level_idx == n_levels - 1)

                    bridge.set_context(f"{label}/level_{level_num}")
                    log.info(
                        "=== Axis %d/%d  %s  Level %d/%d  v_max=%.2f %s ===",
                        axis_num, n_axes, label, level_num, n_levels, v_max, uv,
                    )

                    # ── Level info ────────────────────────────────────────────
                    print(
                        "\n" + "─" * 62 + "\n"
                        f"  {label}  —  Level {level_num}/{n_levels}\n"
                        f"  Velocity  : {v_max} {uv}\n"
                        f"  Start pos : {start_str}\n"
                        + "─" * 62
                    )

                    # ── Step 1: slow move to start position ───────────────────
                    # For the first level the robot is at home.
                    # For subsequent levels it is at the previous sweep's endpoint.
                    # In both cases we move directly to this axis's start pose.
                    _enter(
                        f"\nAbout to move slowly to start position ({start_str}).\n"
                        f"Speed: {_approach_speed(axis):.2f} mm/s  /  "
                        f"{_TRANSITION_ANGULAR_SPEED} deg/s\n"
                        f"Press ENTER to begin, or Ctrl+C to abort."
                    )
                    log.info("Moving slowly to start position for %s level %d ...", label, level_num)
                    interface.move_linear_at(
                        start, _approach_speed(axis), _TRANSITION_ANGULAR_SPEED
                    )
                    interface.wait_until_stopped()

                    # ── Step 2: sweeps ────────────────────────────────────────
                    _enter(
                        f"\nAt start position ({start_str}).\n"
                        f"About to run {TRAP_N_CYCLES} sweeps of {label} "
                        f"at {v_max} {uv}, {s0:+g} ↔ {s1:+g} {up}, "
                        f"with {DWELL_SETTLE:.0f} s pauses between each.\n"
                        f"Press ENTER to begin sweeps, or Ctrl+C to abort."
                    )
                    poses = list(_build_sweep_sequence(axis, v_max).poses())
                    if not servo_run(poses, interface, safety, bridge, SERVO_ACCEL):
                        log.info("Interrupted during sweeps of %s level %d.", label, level_num)
                        raise _Aborted()

                    # ── Step 3: home only after last level of this axis ───────
                    if is_last:
                        _enter(
                            f"\n{label} level {level_num}/{n_levels} complete.\n"
                            f"Press ENTER to return to home position "
                            f"({_TRANSITION_LINEAR_SPEED} mm/s / {_TRANSITION_ANGULAR_SPEED} deg/s), "
                            f"or Ctrl+C to abort."
                        )
                        interface.move_linear_at(
                            cfg.robot.home_pose,
                            _TRANSITION_LINEAR_SPEED,
                            _TRANSITION_ANGULAR_SPEED,
                        )
                        interface.wait_until_stopped()
                        print("\nAt home position.")
                    else:
                        print(f"\n{label} level {level_num}/{n_levels} complete.")

                    log.info(
                        "=== Axis %d/%d  %s  Level %d/%d  done ===",
                        axis_num, n_axes, label, level_num, n_levels,
                    )

            print(
                "\n" + "═" * 62 + "\n"
                "  Phase 1 complete.\n"
                "  All DoFs characterised at all velocity levels.\n"
                + "═" * 62
            )

    except _Aborted:
        return 0
    except EmulationError as e:
        print(f"\nEmulation error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"\nUnexpected error: {e}", file=sys.stderr)
        return 1
    finally:
        bridge.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
