"""Motion primitives for systematic UR16 controller characterisation (Phase 1).

Each primitive is a DataSource yielding ShipPose sequences on a single axis
with all other axes held at zero.  Timestamps start at t0 (default 0.0 s).

Primitives
----------
ChirpSource           – linear frequency sweep (f_start → f_end) on one axis
TrapezoidalMoveSource – ordered waypoints with trapezoidal velocity profile
DwellSource           – hold a fixed value for a given duration
FadeInSource          – smooth amplitude ramp from the initial pose (toggleable)
SequentialSource      – concatenate DataSources, stitching timestamps end-to-end

Units follow the rest of the project:
    linear  axes (x, y, z)        – mm and mm/s and mm/s²
    angular axes (roll, pitch, yaw) – deg and deg/s and deg/s²
"""
import logging
import math
from typing import Iterator, Sequence

log = logging.getLogger(__name__)

from ship_emulation.config import SinusoidalOverlay
from ship_emulation.data_source import DataSource, ShipPose

AXES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _pose(t: float, axis: str, value: float) -> ShipPose:
    """ShipPose with `value` on `axis` and zeros on all other axes."""
    kw: dict = dict(timestamp=t, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)
    kw[axis] = value
    return ShipPose(**kw)


def _trap_duration(distance: float, v_max: float, a: float) -> float:
    """Analytical duration of a trapezoidal (or triangular) move."""
    d = abs(distance)
    d_accel = v_max ** 2 / (2.0 * a)
    if 2.0 * d_accel >= d:
        return 2.0 * math.sqrt(d / a)  # triangular — never reaches v_max
    return 2.0 * (v_max / a) + (d - 2.0 * d_accel) / v_max


def _trap_position(tau: float, p0: float, p1: float, v_max: float, a: float) -> float:
    """Position along a trapezoidal move at segment-local time tau."""
    d = abs(p1 - p0)
    sign = math.copysign(1.0, p1 - p0)
    d_accel = v_max ** 2 / (2.0 * a)

    if 2.0 * d_accel >= d:
        t_a = math.sqrt(d / a)
        v_peak = a * t_a
        t_cruise = 0.0
    else:
        t_a = v_max / a
        v_peak = v_max
        t_cruise = (d - 2.0 * d_accel) / v_max

    t_total = 2.0 * t_a + t_cruise
    tau = min(tau, t_total)

    if tau <= t_a:
        pos = p0 + sign * 0.5 * a * tau ** 2
    elif tau <= t_a + t_cruise:
        pos = p0 + sign * (d_accel + v_peak * (tau - t_a))
    else:
        rem = t_total - tau
        pos = p1 - sign * 0.5 * a * rem ** 2

    # Numerical clamp to segment bounds
    lo, hi = min(p0, p1), max(p0, p1)
    return max(lo, min(hi, pos))


# ---------------------------------------------------------------------------
# Primitives
# ---------------------------------------------------------------------------

class DwellSource(DataSource):
    """Hold a fixed value on one axis for `duration` seconds.

    Args:
        duration:    how long to hold [s]
        axis:        which axis carries the value (default 'x')
        value:       position to hold (default 0.0)
        sample_rate: output frequency [Hz] (default 100)
        t0:          timestamp of first sample [s] (default 0.0)
    """

    def __init__(
        self,
        duration: float,
        axis: str = 'x',
        value: float = 0.0,
        sample_rate: float = 100.0,
        t0: float = 0.0,
    ):
        if axis not in AXES:
            raise ValueError(f"axis must be one of {AXES}, got {axis!r}")
        self.duration = duration
        self.axis = axis
        self.value = value
        self.sample_rate = sample_rate
        self.t0 = t0

    def poses(self) -> Iterator[ShipPose]:
        dt = 1.0 / self.sample_rate
        n_steps = int(math.ceil(self.duration * self.sample_rate))
        for step in range(n_steps + 1):
            yield _pose(self.t0 + step * dt, self.axis, self.value)

    def close(self):
        pass


class ChirpSource(DataSource):
    """Single-axis linear chirp: amplitude * sin(φ(t)).

    The instantaneous frequency increases linearly from f_start to f_end over
    `duration` seconds:

        φ(t) = 2π · [f_start · τ + ½ · (f_end - f_start) · τ² / duration]

    where τ = t - t0.

    Args:
        axis:        which axis to excite ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
        amplitude:   peak displacement [mm or deg]
        f_start:     start frequency [Hz]
        f_end:       end frequency [Hz]
        duration:    sweep duration [s]
        sample_rate: output frequency [Hz] (default 100)
        t0:          timestamp of first sample [s] (default 0.0)
    """

    def __init__(
        self,
        axis: str,
        amplitude: float,
        f_start: float,
        f_end: float,
        duration: float,
        sample_rate: float = 100.0,
        t0: float = 0.0,
    ):
        if axis not in AXES:
            raise ValueError(f"axis must be one of {AXES}, got {axis!r}")
        if f_start <= 0.0 or f_end <= 0.0:
            raise ValueError("f_start and f_end must be positive")
        self.axis = axis
        self.amplitude = amplitude
        self.f_start = f_start
        self.f_end = f_end
        self.duration = duration
        self.sample_rate = sample_rate
        self.t0 = t0

    def poses(self) -> Iterator[ShipPose]:
        dt = 1.0 / self.sample_rate
        n_steps = int(math.ceil(self.duration * self.sample_rate))
        for step in range(n_steps + 1):
            tau = min(step * dt, self.duration)
            phase = 2.0 * math.pi * (
                self.f_start * tau
                + 0.5 * (self.f_end - self.f_start) * tau ** 2 / self.duration
            )
            yield _pose(self.t0 + step * dt, self.axis, self.amplitude * math.sin(phase))

    def close(self):
        pass


class TrapezoidalMoveSource(DataSource):
    """Single-axis motion through a sequence of waypoints with a trapezoidal velocity profile.

    Between each consecutive pair of waypoints the robot:
      1. accelerates from rest at `acceleration` until reaching `max_velocity`
         (or until the midpoint if the segment is too short — triangular profile)
      2. cruises at `max_velocity` (if distance allows)
      3. decelerates back to rest

    An optional `dwell` hold is inserted at each intermediate waypoint.

    Args:
        axis:         which axis to move ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
        waypoints:    ordered positions to visit; first entry is the start [mm or deg]
        max_velocity: peak speed [mm/s or deg/s]
        acceleration: constant accel / decel magnitude [mm/s² or deg/s²]
        dwell:        hold time at each intermediate waypoint [s] (default 0.0)
        sample_rate:  output frequency [Hz] (default 100)
        t0:           timestamp of first sample [s] (default 0.0)

    Example — standard characterisation cycle (rest → +A → −A → rest):
        TrapezoidalMoveSource('x', [0, 50, -50, 0], max_velocity=30, acceleration=100)
    """

    def __init__(
        self,
        axis: str,
        waypoints: Sequence[float],
        max_velocity: float,
        acceleration: float,
        dwell: float = 0.0,
        sample_rate: float = 100.0,
        t0: float = 0.0,
    ):
        if axis not in AXES:
            raise ValueError(f"axis must be one of {AXES}, got {axis!r}")
        if len(waypoints) < 2:
            raise ValueError("Need at least two waypoints")
        if max_velocity <= 0.0:
            raise ValueError("max_velocity must be positive")
        if acceleration <= 0.0:
            raise ValueError("acceleration must be positive")
        self.axis = axis
        self.waypoints = list(waypoints)
        self.max_velocity = max_velocity
        self.acceleration = acceleration
        self.dwell = dwell
        self.sample_rate = sample_rate
        self.t0 = t0

    def poses(self) -> Iterator[ShipPose]:
        dt = 1.0 / self.sample_rate
        n_segs = len(self.waypoints) - 1
        # t_rel: time elapsed since t0, tracked in integer steps to avoid float drift
        global_step = 0

        for seg_idx, (p0, p1) in enumerate(zip(self.waypoints[:-1], self.waypoints[1:])):
            is_last = (seg_idx == n_segs - 1)
            d = abs(p1 - p0)

            if d < 1e-9:
                # Zero-length segment — emit a single hold sample (avoid duplicate on non-last)
                if is_last:
                    yield _pose(self.t0 + global_step * dt, self.axis, p0)
                continue

            t_total = _trap_duration(d, self.max_velocity, self.acceleration)
            n_steps = int(math.ceil(t_total / dt))

            # Emit steps 0 … n_steps-1 for non-last segments; 0 … n_steps for the last.
            # This avoids emitting the boundary sample twice (it becomes step-0 of next segment).
            end_step = n_steps if is_last else n_steps - 1
            for step in range(end_step + 1):
                tau = min(step * dt, t_total)
                pos = _trap_position(tau, p0, p1, self.max_velocity, self.acceleration)
                yield _pose(self.t0 + (global_step + step) * dt, self.axis, pos)

            global_step += n_steps

            # Dwell at intermediate waypoints
            if self.dwell > 0.0 and not is_last:
                n_dwell = int(math.ceil(self.dwell / dt))
                for dstep in range(n_dwell):
                    yield _pose(self.t0 + global_step * dt, self.axis, p1)
                    global_step += 1

    def close(self):
        pass


class SequentialSource(DataSource):
    """Concatenate multiple DataSources, stitching their timestamps end-to-end.

    The first source's timestamps are emitted as-is.  Each subsequent source
    is time-shifted so its first sample follows the last sample of the previous
    source by `gap` seconds (default 0.0 — back-to-back, no pause).

    This lets you chain primitives without manually managing t0 values:

        seq = SequentialSource([
            ChirpSource('x', amplitude=10, f_start=0.05, f_end=2.0, duration=60),
            DwellSource(duration=5, axis='x'),
            TrapezoidalMoveSource('x', [0, 50, -50, 0], max_velocity=30, acceleration=100),
        ])

    Args:
        sources: ordered list of DataSource instances
        gap:     pause inserted between consecutive sources [s] (default 0.0)
    """

    def __init__(self, sources: Sequence[DataSource], gap: float = 0.0):
        self._sources = list(sources)
        self.gap = gap

    def poses(self) -> Iterator[ShipPose]:
        t_last = None    # absolute time of the most recent emitted sample
        t_prev = None    # absolute time of the second-to-last sample (used to infer dt)
        t_shift = 0.0

        for source in self._sources:
            if source.name:
                log.info(">> %s", source.name)
            src_t0 = None
            for pose in source.poses():
                if src_t0 is None:
                    src_t0 = pose.timestamp
                    if t_last is None:
                        t_shift = 0.0  # keep first source's t0 as-is
                    else:
                        # Advance by one natural sample period (inferred from last two
                        # samples of the previous source) plus any explicit gap.
                        natural_dt = (t_last - t_prev) if t_prev is not None else 0.0
                        t_shift = (t_last + natural_dt + self.gap) - src_t0

                adjusted = ShipPose(
                    timestamp=pose.timestamp + t_shift,
                    x=pose.x,
                    y=pose.y,
                    z=pose.z,
                    roll=pose.roll,
                    pitch=pose.pitch,
                    yaw=pose.yaw,
                )
                t_prev = t_last
                t_last = adjusted.timestamp
                yield adjusted

    def close(self):
        for s in self._sources:
            s.close()


class FadeInSource(DataSource):
    """Smooth amplitude ramp at the start of any DataSource.

    Scales deviations from the first pose by a raised-cosine (Hann) envelope
    that rises from 0 to 1 over `duration` seconds.  The robot therefore starts
    exactly at the initial pose and gradually builds to full amplitude — avoiding
    the velocity step that occurs when playback begins mid-motion.

    Set ``enabled=False`` to bypass the fade entirely and pass poses through
    unchanged.  This makes it easy to toggle fade-in without restructuring the
    source pipeline.

    Args:
        source:   underlying DataSource to wrap
        duration: fade-in duration [s]
        enabled:  False → pass through unchanged (default True)
    """

    def __init__(self, source: DataSource, duration: float, enabled: bool = True):
        self._source = source
        self.duration = duration
        self.enabled = enabled

    def poses(self) -> Iterator[ShipPose]:
        if not self.enabled or self.duration <= 0.0:
            yield from self._source.poses()
            return

        t0: float | None = None
        p0: ShipPose | None = None

        for pose in self._source.poses():
            if t0 is None:
                t0 = pose.timestamp
                p0 = pose

            tau = pose.timestamp - t0
            if tau >= self.duration:
                yield pose
                continue

            # Raised-cosine ramp: 0 at tau=0, 1 at tau=duration
            envelope = 0.5 * (1.0 - math.cos(math.pi * tau / self.duration))
            yield ShipPose(
                timestamp=pose.timestamp,
                x=p0.x + envelope * (pose.x - p0.x),
                y=p0.y + envelope * (pose.y - p0.y),
                z=p0.z + envelope * (pose.z - p0.z),
                roll=p0.roll + envelope * (pose.roll - p0.roll),
                pitch=p0.pitch + envelope * (pose.pitch - p0.pitch),
                yaw=p0.yaw + envelope * (pose.yaw - p0.yaw),
            )

    def close(self):
        self._source.close()


class SinusoidalOverlaySource(DataSource):
    """Add sinusoidal roll/pitch/yaw overlays on top of any DataSource.

    The overlay amplitude is scaled by the same raised-cosine envelope used by
    FadeInSource, so the added signal grows smoothly from zero over
    ``fade_duration`` seconds rather than switching on at its arbitrary initial
    phase.  Set ``fade_enabled=False`` to apply the overlay at full amplitude
    from the first sample.

    Args:
        source:        underlying DataSource to wrap
        overlay:       SinusoidalOverlay configuration (amplitudes and frequencies)
        fade_duration: ramp-up duration [s] (default 0.0 → no ramp)
        fade_enabled:  False → full amplitude from t=0
    """

    def __init__(
        self,
        source: DataSource,
        overlay: SinusoidalOverlay,
        fade_duration: float = 0.0,
        fade_enabled: bool = True,
    ):
        self._source = source
        self._overlay = overlay
        self._fade_duration = fade_duration
        self._fade_enabled = fade_enabled

    def poses(self) -> Iterator[ShipPose]:
        ov = self._overlay
        t0: float | None = None

        for pose in self._source.poses():
            if t0 is None:
                t0 = pose.timestamp

            if self._fade_enabled and self._fade_duration > 0.0:
                tau = pose.timestamp - t0
                if tau < self._fade_duration:
                    envelope = 0.5 * (1.0 - math.cos(math.pi * tau / self._fade_duration))
                else:
                    envelope = 1.0
            else:
                envelope = 1.0

            t = pose.timestamp
            yield ShipPose(
                timestamp=t,
                x=pose.x,
                y=pose.y,
                z=pose.z,
                roll=pose.roll + envelope * ov.roll_amplitude * math.sin(2.0 * math.pi * ov.roll_frequency * t),
                pitch=pose.pitch + envelope * ov.pitch_amplitude * math.sin(2.0 * math.pi * ov.pitch_frequency * t),
                yaw=pose.yaw + envelope * ov.yaw_amplitude * math.sin(2.0 * math.pi * ov.yaw_frequency * t),
            )

    def close(self):
        self._source.close()
