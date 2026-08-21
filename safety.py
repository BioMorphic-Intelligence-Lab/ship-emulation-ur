import math
from typing import Optional

from ship_emulation.config import SafetyConfig
from ship_emulation.data_source import ShipPose


class SafetyViolation(Exception):
    pass


class SafetyChecker:
    def __init__(self, config: SafetyConfig):
        self._config = config
        self._prev: Optional[ShipPose] = None

    def reset(self):
        self._prev = None

    def check(self, pose: ShipPose) -> None:
        self._check_workspace(pose)
        if self._prev is not None:
            self._check_rates(pose, self._prev)
            self._check_timestamp(pose, self._prev)
        self._prev = pose

    def _check_workspace(self, pose: ShipPose) -> None:
        lim = self._config.workspace
        checks = [
            ("x", pose.x, lim.x_min, lim.x_max),
            ("y", pose.y, lim.y_min, lim.y_max),
            ("z", pose.z, lim.z_min, lim.z_max),
            ("roll", pose.roll, lim.roll_min, lim.roll_max),
            ("pitch", pose.pitch, lim.pitch_min, lim.pitch_max),
            ("yaw", pose.yaw, lim.yaw_min, lim.yaw_max),
        ]
        for name, val, lo, hi in checks:
            if not (lo <= val <= hi):
                raise SafetyViolation(
                    f"{name}={val:.3f} out of workspace bounds [{lo}, {hi}]"
                )

    def _check_rates(self, pose: ShipPose, prev: ShipPose) -> None:
        dt = pose.timestamp - prev.timestamp
        linear_dist = math.sqrt(
            (pose.x - prev.x) ** 2
            + (pose.y - prev.y) ** 2
            + (pose.z - prev.z) ** 2
        )
        linear_rate = linear_dist / dt
        if linear_rate > self._config.max_linear_rate:
            raise SafetyViolation(
                f"linear rate {linear_rate:.1f} mm/s exceeds limit "
                f"{self._config.max_linear_rate:.1f} mm/s"
            )

        worst_angular = max(
            abs(pose.roll - prev.roll),
            abs(pose.pitch - prev.pitch),
            abs(pose.yaw - prev.yaw),
        )
        angular_rate = worst_angular / dt
        if angular_rate > self._config.max_angular_rate:
            raise SafetyViolation(
                f"angular rate {angular_rate:.1f} deg/s exceeds limit "
                f"{self._config.max_angular_rate:.1f} deg/s"
            )

    def _check_timestamp(self, pose: ShipPose, prev: ShipPose) -> None:
        dt = pose.timestamp - prev.timestamp
        if dt <= 0:
            raise SafetyViolation(
                f"timestamp not monotonically increasing: "
                f"prev={prev.timestamp:.6f}, current={pose.timestamp:.6f}"
            )
        if dt < self._config.min_dt:
            raise SafetyViolation(
                f"timestamp step {dt:.6f} s below minimum {self._config.min_dt:.6f} s"
            )
