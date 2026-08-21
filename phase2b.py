"""Phase 2 — ship motion emulation from a random 3-minute window.

Loads CSV_FILE, draws a random contiguous window of WINDOW_DURATION seconds,
optionally applies a smooth fade-in (raised-cosine ramp), and streams Cartesian
velocities to the robot via speedL for smooth, gap-free trajectory following.

Edit the CONFIGURATION block below, then run:
    python -m ship_emulation.phase2

Set RANDOM_SEED to an integer to reproduce the same window across runs.
Set FADE_IN_ENABLED = False to start playback at full amplitude immediately.
"""
import logging
import random
import sys
from pathlib import Path
from typing import Iterator, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ship_emulation.config import AugmentationConfig, EmulatorConfig, SinusoidalOverlay
from ship_emulation.data_source import AugmentedSource, CsvFileSource, DataSource, ShipPose
from ship_emulation.emulator import EmulationError, servo_run
from ship_emulation.motion_primitives import FadeInSource, SinusoidalOverlaySource
from ship_emulation.robot_interface import UR16Interface
from ship_emulation.ros_bridge import RosBridge
from ship_emulation.safety import SafetyChecker

# ── CONFIGURATION ─────────────────────────────────────────────────────────────

#ROBOT_IP = "172.17.0.2" # sim
ROBOT_IP = "192.168.56.101" # real

# Path to the vessel motion CSV (relative to working directory or absolute).
# Available files in ship_emulation/:
#   1_vessel_motion_clean.csv
#   2_vessel_motion_clean.csv  /  2a_vessel_motion_clean.csv  /  2b_vessel_motion_clean.csv
#   3_vessel_motion_clean.csv
CSV_FILE = "ship_emulation/1_vessel_motion_com_1m_translated.csv"

WINDOW_DURATION = 180.0  # s — length of the sampled window (3 minutes)
RANDOM_SEED: Optional[int] = 42  # set to an int for reproducibility

# Fix the window start time (seconds in the CSV).  When set to a float the
# random sampler is bypassed and playback always begins at this timestamp.
# Set to None to use random selection (governed by RANDOM_SEED).
WINDOW_START: Optional[float] = None

# Playback speed relative to real time.  1.0 = real time; 0.1 = 10× slower.
# Values below 1.0 are useful for pre-flight checks: the full trajectory is
# executed at reduced velocity so singularities and self-collisions can be
# spotted before a full-speed run.  The safety pre-check always runs at the
# original (1.0×) rates, so it remains conservative at any slower speed.
PLAYBACK_SPEED = 0.3

# Unit conversion applied to raw CSV columns
LINEAR_SCALE  = 1000.0  # multiply x/y/z  (default: m → mm)
ANGULAR_SCALE =    1.0  # multiply roll/pitch/yaw

# Fade-in: smooth amplitude ramp from the initial pose at the start of playback
FADE_IN_ENABLED  = True
FADE_IN_DURATION = 10.0  # s — duration of the raised-cosine onset ramp

# Speed for the slow approach from home (0) to sweep start position (−A).
# Applies to roll, pitch, x, y only. Keep well below lowest TRAP_VELOCITIES.
APPROACH_VELOCITY = {
    "roll":  3.0,   # deg/s
    "pitch": 3.0,   # deg/s
    "x":     50.0,   # mm/s
    "y":     50.0,   # mm/s
}

# Speeds used for ALL slow positioning moves — both the initial approach from home
# and direct inter-axis transitions.  Derived from APPROACH_VELOCITY so there is
# one place to tune them.
_TRANSITION_LINEAR_SPEED  = min(APPROACH_VELOCITY[a] for a in ("x", "y"))        # mm/s
_TRANSITION_ANGULAR_SPEED = min(APPROACH_VELOCITY[a] for a in ("roll", "pitch"))  # deg/s

# Acceleration for velocity streaming (speedL).  Higher values let the robot
# track rapid velocity changes more faithfully; lower values give smoother but
# more lag-prone motion.  2000 mm/s² is a good default for ship-motion rates.
SERVO_ACCEL = 1000.0  # mm/s²

# Pose stride — use every Nth pose from the loaded trajectory.
# The RTDE command/ACK protocol requires one round-trip per speedL segment;
# each round-trip takes ~dt_streamed seconds.  With 10 Hz CSV data and
# PLAYBACK_SPEED=1.0, dt_streamed=0.1 s which is too tight for reliable
# operation.  A stride of 5 gives dt_streamed=0.5 s (stable) while keeping
# the correct real-time velocities.  Increase if the robot still stops
# unexpectedly; decrease for higher trajectory fidelity once stable.
POSE_STRIDE = 1

# Sinusoidal overlay — added on top of the CSV roll and pitch channels to
# excite them beyond their natural amplitude in the data.
# Set OVERLAY_ENABLED = False (or both amplitudes to 0) to disable.
OVERLAY_ENABLED        = True
OVERLAY_ROLL_AMP       = 10.0    # deg — peak amplitude added to roll
OVERLAY_ROLL_FREQ      = 0.1    # Hz
OVERLAY_PITCH_AMP      = 15.0    # deg — peak amplitude added to pitch
OVERLAY_PITCH_FREQ     = 0.07   # Hz

# ── END CONFIGURATION ─────────────────────────────────────────────────────────

log = logging.getLogger(__name__)


class _WindowedSource(DataSource):
    """Yields pre-loaded poses whose timestamps fall within [t_start, t_end]."""

    def __init__(self, all_poses: List[ShipPose], t_start: float, t_end: float):
        self._poses  = all_poses
        self._t_start = t_start
        self._t_end   = t_end

    def poses(self) -> Iterator[ShipPose]:
        for pose in self._poses:
            if pose.timestamp < self._t_start:
                continue
            if pose.timestamp > self._t_end:
                break
            yield pose

    def close(self):
        pass


def _build_source(
    csv_path: str,
    window_duration: float,
    seed: Optional[int],
    linear_scale: float,
    angular_scale: float,
    fade_in_enabled: bool,
    fade_in_duration: float,
    window_start: Optional[float] = None,
    overlay: Optional[SinusoidalOverlay] = None,
) -> tuple:
    with CsvFileSource(csv_path) as raw:
        all_poses = list(raw.poses())

    if not all_poses:
        raise ValueError(f"No poses loaded from {csv_path!r}")

    t_data_start = all_poses[0].timestamp
    t_data_end   = all_poses[-1].timestamp
    total_span   = t_data_end - t_data_start

    if total_span < window_duration:
        raise ValueError(
            f"CSV spans only {total_span:.1f} s — cannot sample a {window_duration:.1f} s window"
        )

    if window_start is not None:
        t_start = float(window_start)
        if t_start < t_data_start or t_start + window_duration > t_data_end:
            raise ValueError(
                f"WINDOW_START={t_start:.3f} s places the window outside the CSV range "
                f"[{t_data_start:.3f}, {t_data_end:.3f}] s"
            )
        log.info(
            "CSV data span:    t=%.3f s – %.3f s  (%.1f s total)",
            t_data_start, t_data_end, total_span,
        )
        log.info(
            "Fixed window:     t=%.3f s – %.3f s  (%.1f s, WINDOW_START override)",
            t_start, t_start + window_duration, window_duration,
        )
    else:
        rng     = random.Random(seed)
        t_start = t_data_start + rng.uniform(0.0, total_span - window_duration)
        log.info(
            "CSV data span:    t=%.3f s – %.3f s  (%.1f s total)",
            t_data_start, t_data_end, total_span,
        )
        log.info(
            "Selected window:  t=%.3f s – %.3f s  (%.1f s, seed=%s)",
            t_start, t_start + window_duration, window_duration, seed,
        )

    t_end     = t_start + window_duration
    windowed  = _WindowedSource(all_poses, t_start, t_end)
    augmented = AugmentedSource(
        windowed,
        AugmentationConfig(linear_scale=linear_scale, angular_scale=angular_scale),
    )
    faded: DataSource = FadeInSource(augmented, duration=fade_in_duration, enabled=fade_in_enabled)
    if overlay is not None:
        faded = SinusoidalOverlaySource(
            faded, overlay,
            fade_duration=fade_in_duration,
            fade_enabled=fade_in_enabled,
        )
    return faded, t_start



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


def main() -> int:
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    cfg = EmulatorConfig()
    cfg.robot.ip = ROBOT_IP

    overlay = (
        SinusoidalOverlay(
            roll_amplitude=OVERLAY_ROLL_AMP,
            roll_frequency=OVERLAY_ROLL_FREQ,
            pitch_amplitude=OVERLAY_PITCH_AMP,
            pitch_frequency=OVERLAY_PITCH_FREQ,
        )
        if OVERLAY_ENABLED else None
    )

    try:
        source, window_t_start = _build_source(
            CSV_FILE,
            WINDOW_DURATION,
            RANDOM_SEED,
            LINEAR_SCALE,
            ANGULAR_SCALE,
            FADE_IN_ENABLED,
            FADE_IN_DURATION,
            window_start=WINDOW_START,
            overlay=overlay,
        )
    except ValueError as e:
        print(f"Configuration error: {e}", file=sys.stderr)
        return 1

    window_mode  = f"fixed t={WINDOW_START:.1f} s" if WINDOW_START is not None else f"random (seed={RANDOM_SEED})"
    speed_note   = f"{PLAYBACK_SPEED:.2g}× (slow-check mode)" if PLAYBACK_SPEED != 1.0 else "1× (real time)"
    if overlay is not None:
        overlay_note = (
            f"roll {overlay.roll_amplitude:.1f}° @ {overlay.roll_frequency:.3f} Hz  "
            f"pitch {overlay.pitch_amplitude:.1f}° @ {overlay.pitch_frequency:.3f} Hz"
        )
    else:
        overlay_note = "disabled"
    print(
        "\n" + "═" * 62 + "\n"
        "  Phase 2 — Ship motion emulation\n"
        + "═" * 62 + "\n"
        f"  CSV          : {CSV_FILE}\n"
        f"  Window       : {WINDOW_DURATION:.0f} s  [{window_mode}]\n"
        f"  Playback     : {speed_note}\n"
        f"  Fade-in      : {FADE_IN_DURATION:.0f} s ({'enabled' if FADE_IN_ENABLED else 'DISABLED'})\n"
        f"  Overlay      : {overlay_note}\n"
        + "─" * 62 + "\n"
        "  Ensure the workspace is clear.\n"
        "  Keep the physical E-stop within reach at all times.\n"
        + "─" * 62
    )

    bridge = RosBridge("phase2")

    try:
        _enter("\nPress ENTER to connect to the robot, or Ctrl+C to abort.")

        with source, UR16Interface(cfg.robot) as interface:
            safety = SafetyChecker(cfg.safety)
            bridge.start_feedback(lambda: interface.current_pose)
            bridge.set_context("phase2")

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

            log.info("Loading trajectory ...")
            poses = list(source.poses())
            actual_duration = poses[-1].timestamp - poses[0].timestamp if poses else 0.0
            log.info("Loaded %d poses  (%.1f s)", len(poses), actual_duration)

            actual_wall = actual_duration / PLAYBACK_SPEED
            _enter(
                f"\nTrajectory loaded: {len(poses)} poses  ({actual_duration:.1f} s)\n"
                f"Window start     : t={window_t_start:.3f} s in CSV\n"
                f"Playback speed   : {PLAYBACK_SPEED:.2g}×  →  wall time ≈ {actual_wall:.0f} s\n"
                f"Fade-in          : {FADE_IN_DURATION:.0f} s ({'enabled' if FADE_IN_ENABLED else 'DISABLED'})\n"
                "Press ENTER to start emulation, or Ctrl+C to abort."
            )

            if not servo_run(poses, interface, safety, bridge, SERVO_ACCEL, speed_factor=PLAYBACK_SPEED, pose_stride=POSE_STRIDE):
                raise _Aborted()

            print("\nEmulation complete. Returning to home position.")
            interface.move_linear_at(
                            cfg.robot.home_pose,
                            _TRANSITION_LINEAR_SPEED,
                            _TRANSITION_ANGULAR_SPEED,
                        )
            interface.wait_until_stopped()
            print("At home position.")

    except _Aborted:
        return 0
    except EmulationError as e:
        print(f"Emulation error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        return 1
    finally:
        bridge.close()

    log.info("Phase 2 complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
