import logging
import signal
import time
from typing import List

from cri.ur.rtde_client import RTDEClient

from ship_emulation.config import EmulatorConfig
from ship_emulation.data_source import DataSource, ShipPose
from ship_emulation.robot_interface import UR16Interface
from ship_emulation.safety import SafetyChecker, SafetyViolation

log = logging.getLogger(__name__)

_SLEEP_MIN = 0.005  # skip sleep calls shorter than this to avoid OS overhead


class EmulationError(RuntimeError):
    pass


class ShipEmulator:
    def __init__(
        self,
        source: DataSource,
        interface: UR16Interface,
        safety: SafetyChecker,
        config: EmulatorConfig,
        on_move=None,
        home_at_start: bool = True,
        home_at_end: bool = True,
    ):
        self._source = source
        self._interface = interface
        self._safety = safety
        self._config = config
        self._on_move = on_move  # optional callable(ShipPose) called after each move_to
        self._home_at_start = home_at_start
        self._home_at_end = home_at_end

    def run(self):
        self._safety.reset()

        stop_requested = False

        def _handle_signal(sig, frame):
            nonlocal stop_requested
            log.info("Signal %s received — stopping", sig)
            stop_requested = True
            raise KeyboardInterrupt

        original_sigint = signal.getsignal(signal.SIGINT)
        original_sigterm = signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)

        try:
            if self._home_at_start:
                self._interface.move_home()
                self._interface.wait_until_stopped()

            pose_iter = iter(self._source.poses())
            first_pose = next(pose_iter, None)
            if first_pose is None:
                return

            try:
                self._safety.check(first_pose)
            except SafetyViolation as e:
                log.error("Safety violation on first pose: %s", e)
                self._interface.emergency_stop()
                raise EmulationError(f"Safety violation: {e}") from e

            log.info("Moving to start pose before beginning playback")
            self._interface.move_to(first_pose.as_robot_pose())
            self._interface.wait_for_motion()

            wall_origin = time.monotonic()
            sim_origin = first_pose.timestamp
            pose_count = 1
            late_count = 0

            for pose in pose_iter:
                if stop_requested:
                    break

                try:
                    self._safety.check(pose)
                except SafetyViolation as e:
                    log.error("Safety violation: %s", e)
                    self._interface.emergency_stop()
                    raise EmulationError(f"Safety violation: {e}") from e

                target_wall = wall_origin + (pose.timestamp - sim_origin)
                now = time.monotonic()
                sleep_s = target_wall - now
                if sleep_s > _SLEEP_MIN:
                    time.sleep(sleep_s)

                if not self._interface.is_motion_done():
                    self._interface.wait_for_motion()
                    now = time.monotonic()
                    if now > target_wall:
                        sim_t = pose.timestamp - sim_origin
                        lateness_ms = (now - target_wall) * 1000.0
                        n_skip = 0
                        while target_wall < time.monotonic():
                            nxt = next(pose_iter, None)
                            if nxt is None:
                                pose = None
                                break
                            n_skip += 1
                            pose = nxt
                            target_wall = wall_origin + (pose.timestamp - sim_origin)
                        late_count += 1
                        log.warning(
                            "pose %d  sim_t=%.3fs  late=%.1fms  skipped %d stale pose(s)",
                            pose_count, sim_t, lateness_ms, n_skip,
                        )
                        if pose is None:
                            break
                        try:
                            self._safety.check(pose)
                        except SafetyViolation as e:
                            log.error("Safety violation on catch-up pose: %s", e)
                            self._interface.emergency_stop()
                            raise EmulationError(f"Safety violation: {e}") from e

                self._interface.move_to(pose.as_robot_pose())
                if self._on_move is not None:
                    self._on_move(pose)
                pose_count += 1

            self._interface.wait_for_motion()
            if self._home_at_end:
                self._interface.move_home()

        except KeyboardInterrupt:
            log.info("Interrupted — closing connection")
            try:
                self._interface.close()
            except Exception:
                pass
            return False

        finally:
            signal.signal(signal.SIGINT, original_sigint)
            signal.signal(signal.SIGTERM, original_sigterm)

        log.info(
            "Emulation complete: %d poses replayed, %d late moves",
            pose_count,
            late_count,
        )
        return True


def servo_run(
    poses: List[ShipPose],
    interface: UR16Interface,
    safety: SafetyChecker,
    on_move,
    servo_accel: float,
    speed_factor: float = 1.0,
    pose_stride: int = 1,
) -> bool:
    """Stream Cartesian velocities derived from a pose list via speedL.

    For each consecutive pair of poses the finite-difference velocity is sent
    as a speedL segment of dt seconds.  Each segment executes on a URScript
    background thread and returns immediately; the next call joins that thread
    first, giving gapless back-to-back execution with no moveL stop-start
    overhead.

    speed_factor < 1 slows playback proportionally (velocity × speed_factor,
    segment duration ÷ speed_factor).  The safety pre-check always runs at
    the original 1× rates, so it remains conservative at any slower speed.

    pose_stride > 1 uses every Nth pose, increasing the segment duration by N×
    while keeping the same real-time velocities.  This trades trajectory
    temporal resolution for a more stable RTDE round-trip budget.  A stride of
    5 on 10 Hz CSV data gives 0.5 s segments, which is reliably stable with the
    command/ACK URScript protocol.  The safety pre-check still runs on ALL
    poses regardless of stride.

    Returns True on clean completion, False if interrupted by Ctrl+C.
    Raises EmulationError on safety violation.
    """
    if not poses:
        return True

    if speed_factor <= 0.0:
        raise ValueError(f"speed_factor must be positive, got {speed_factor}")

    safety.reset()

    # Pre-check entire trajectory at original (1×) rates.
    for pose in poses:
        try:
            safety.check(pose)
        except SafetyViolation as e:
            raise EmulationError(f"Safety violation in planned trajectory: {e}") from e

    # Stride the pose list to increase segment duration and stabilise the
    # RTDE round-trip budget.  Always include the final pose so the endpoint
    # is reached exactly.
    if pose_stride > 1:
        strided = poses[::pose_stride]
        if strided[-1] is not poses[-1]:
            strided = strided + [poses[-1]]
        poses = strided
        log.info(
            "Pose stride %d: %d poses → %d segments  (dt ~%.2f s per segment)",
            pose_stride, len(poses), len(poses) - 1,
            (poses[-1].timestamp - poses[0].timestamp) / (len(poses) - 1) if len(poses) > 1 else 0.0,
        )

    log.info("Moving to first trajectory pose ...")
    interface.move_linear_at(poses[0].as_robot_pose(), 100.0, 5.0)
    interface.wait_until_stopped()

    stop_requested = False

    orig_sigint  = signal.getsignal(signal.SIGINT)
    orig_sigterm = signal.getsignal(signal.SIGTERM)

    def _handle(sig, frame):
        nonlocal stop_requested
        stop_requested = True
        # Disarm immediately so subsequent Ctrl+C presses don't interrupt cleanup.
        signal.signal(signal.SIGINT, orig_sigint)
        signal.signal(signal.SIGTERM, orig_sigterm)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    _FEEDBACK_INTERVAL = 5.0   # s between progress log lines
    _JOINT_WARN_DEG    = 25.0  # deg — warn if any joint moves this much in one segment
                               # (normal ship motion: <10° per segment; a wrist flip: 90–180°)

    n_total     = len(poses) - 1
    n_segments  = 0
    t_start     = time.monotonic()
    t_last_fb   = t_start
    prev_joints = None

    try:
        for i in range(n_total):
            if stop_requested:
                break

            # ── Singularity detection ─────────────────────────────────────────
            joints = interface.current_joint_angles
            if joints is not None and prev_joints is not None:
                deltas  = [abs(j1 - j0) for j1, j0 in zip(joints, prev_joints)]
                max_d   = max(deltas)
                max_idx = deltas.index(max_d)
                if max_d > _JOINT_WARN_DEG:
                    log.warning(
                        "Large joint motion: joint %d moved %.1f° in one segment "
                        "(t=+%.1fs) — possible singularity",
                        max_idx + 1, max_d, time.monotonic() - t_start,
                    )
            prev_joints = joints

            # ── Periodic progress feedback ────────────────────────────────────
            now = time.monotonic()
            if now - t_last_fb >= _FEEDBACK_INTERVAL:
                elapsed  = now - t_start
                progress = i / n_total
                eta      = elapsed / progress * (1.0 - progress) if progress > 0 else 0.0
                tcp      = interface.current_pose
                rs       = interface.runtime_state
                _RS_NAMES = {0: "Stopping", 1: "Stopped", 2: "Playing", 3: "Paused"}
                rs_str   = _RS_NAMES.get(rs, f"Unknown({rs})")
                if tcp is not None:
                    pos_str = (
                        f"x={tcp[0]:+.0f}  y={tcp[1]:+.0f}  z={tcp[2]:+.0f} mm  "
                        f"roll={tcp[3]:+.1f}  pitch={tcp[4]:+.1f}  yaw={tcp[5]:+.1f}°"
                    )
                else:
                    pos_str = "unavailable"
                log.info(
                    "t=+%.0fs  %.0f%%  ETA ~%.0fs  runtime=%s  |  %s",
                    elapsed, progress * 100, eta, rs_str, pos_str,
                )
                if rs != 2:
                    raise EmulationError(
                        f"URScript program no longer Playing at t=+{elapsed:.1f}s "
                        f"segment={i}  runtime_state={rs_str}"
                    )
                t_last_fb = now

            # ── Send velocity segment ─────────────────────────────────────────
            p0, p1 = poses[i], poses[i + 1]
            dt_orig     = p1.timestamp - p0.timestamp
            dt_streamed = dt_orig / speed_factor
            r0, r1      = p0.as_robot_pose(), p1.as_robot_pose()
            velocity    = tuple((r1[j] - r0[j]) / dt_streamed for j in range(6))
            t_seg_start = time.monotonic()
            try:
                interface.servo_linear_velocity(velocity, servo_accel, dt_streamed)
            except RTDEClient.RTDECommandTimeout as e:
                elapsed  = time.monotonic() - t_start
                hang_s   = time.monotonic() - t_seg_start
                csv_t    = p0.timestamp
                log.error(
                    "RTDE command timeout at segment %d  t=+%.1fs  csv_t=%.3f s  "
                    "hang=%.1f s  v=(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f) mm/s·deg/s  "
                    "dt_streamed=%.3f s: %s",
                    i, elapsed, csv_t,
                    hang_s, *velocity, dt_streamed, e,
                )
                raise EmulationError(f"RTDE command timeout at segment {i}: {e}") from e
            if on_move is not None:
                on_move(p1)
            n_segments += 1

        interface.stop_linear(servo_accel)

    except KeyboardInterrupt:
        try:
            interface.stop_linear(5000)
            interface.wait_until_stopped()
        except Exception:
            pass
        return False
    finally:
        signal.signal(signal.SIGINT, orig_sigint)
        signal.signal(signal.SIGTERM, orig_sigterm)

    log.info("Velocity streaming complete: %d segments", n_segments)
    return not stop_requested
