import csv
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Iterator, List

import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.spatial.transform import Rotation

from ship_emulation.config import AugmentationConfig


@dataclass(frozen=True)
class ShipPose:
    timestamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def as_robot_pose(self):
        return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)


class DataSource(ABC):
    name: str = ""  # optional label used by SequentialSource for transition logging

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    @abstractmethod
    def poses(self) -> Iterator[ShipPose]:
        pass

    @abstractmethod
    def close(self):
        pass


class CsvFileSource(DataSource):
    def __init__(self, path: str, skip_header: bool = True):
        self._path = path
        self._skip_header = skip_header

    def poses(self) -> Iterator[ShipPose]:
        with open(self._path, newline="") as f:
            reader = csv.reader(f)
            if self._skip_header:
                next(reader, None)
            for lineno, row in enumerate(reader, start=2 if self._skip_header else 1):
                if len(row) != 7:
                    raise ValueError(
                        f"{self._path}:{lineno}: expected 7 columns, got {len(row)}"
                    )
                try:
                    values = [float(v) for v in row]
                except ValueError as e:
                    raise ValueError(f"{self._path}:{lineno}: {e}") from e
                yield ShipPose(*values)

    def close(self):
        pass


class AugmentedSource(DataSource):
    """Wraps any DataSource, applying per-channel scaling and an optional sinusoidal overlay."""

    def __init__(self, source: DataSource, config: AugmentationConfig):
        self._source = source
        self._config = config

    def poses(self) -> Iterator[ShipPose]:
        cfg = self._config
        for pose in self._source.poses():
            roll = pose.roll * cfg.angular_scale
            pitch = pose.pitch * cfg.angular_scale
            yaw = pose.yaw * cfg.angular_scale
            if cfg.overlay is not None:
                ov = cfg.overlay
                t = pose.timestamp
                roll += ov.roll_amplitude * math.sin(2 * math.pi * ov.roll_frequency * t)
                pitch += ov.pitch_amplitude * math.sin(2 * math.pi * ov.pitch_frequency * t)
                yaw += ov.yaw_amplitude * math.sin(2 * math.pi * ov.yaw_frequency * t)
            yield ShipPose(
                timestamp=pose.timestamp,
                x=pose.x * cfg.linear_scale,
                y=pose.y * cfg.linear_scale,
                z=pose.z * cfg.linear_scale,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
            )

    def close(self):
        self._source.close()


class SmoothedSource(DataSource):
    """Wraps any DataSource, fitting a smoothing spline to each channel.

    Buffers the full dataset upfront, then re-emits poses at the original
    timestamps with spline-smoothed values.  The smoothing_factor s is passed
    directly to scipy UnivariateSpline: s=0 interpolates exactly, larger
    values smooth more aggressively.  A good starting point for mm-scale
    ship motion data is s=1e4–1e6; tune until peak linear rate drops to a
    robot-safe level.
    """

    def __init__(self, source: DataSource, s_linear: float, s_angular: float):
        self._source = source
        self._s_linear = s_linear
        self._s_angular = s_angular

    def poses(self) -> Iterator[ShipPose]:
        all_poses: List[ShipPose] = list(self._source.poses())
        if len(all_poses) < 5:
            yield from all_poses
            return

        t = np.array([p.timestamp for p in all_poses])
        linear_channels = dict(
            x=np.array([p.x for p in all_poses]),
            y=np.array([p.y for p in all_poses]),
            z=np.array([p.z for p in all_poses]),
        )
        angular_channels = dict(
            roll=np.array([p.roll for p in all_poses]),
            pitch=np.array([p.pitch for p in all_poses]),
            yaw=np.array([p.yaw for p in all_poses]),
        )
        smoothed = {
            name: UnivariateSpline(t, values, s=self._s_linear)(t)
            for name, values in linear_channels.items()
        }
        smoothed.update({
            name: UnivariateSpline(t, values, s=self._s_angular)(t)
            for name, values in angular_channels.items()
        })

        for i, pose in enumerate(all_poses):
            yield ShipPose(
                timestamp=pose.timestamp,
                x=float(smoothed['x'][i]),
                y=float(smoothed['y'][i]),
                z=float(smoothed['z'][i]),
                roll=float(smoothed['roll'][i]),
                pitch=float(smoothed['pitch'][i]),
                yaw=float(smoothed['yaw'][i]),
            )

    def close(self):
        self._source.close()


class RigidBodyOffsetSource(DataSource):
    """Wraps any DataSource, translating COG motion to a fixed body-frame offset point.

    Given an offset (ox, oy, oz) in the body frame (same units as the incoming
    pose data), computes the world-frame position of that point at each timestep:

        p_point = p_cog + R(roll, pitch, yaw) @ [ox, oy, oz]

    Orientation channels are unchanged — all points on a rigid body share the
    same orientation.  Euler convention: intrinsic XYZ (roll→pitch→yaw).
    """

    def __init__(self, source: DataSource, ox: float, oy: float, oz: float):
        self._source = source
        self._offset = np.array([ox, oy, oz], dtype=float)

    def poses(self) -> Iterator[ShipPose]:
        for pose in self._source.poses():
            R = Rotation.from_euler('xyz', [pose.roll, pose.pitch, pose.yaw], degrees=True).as_matrix()
            delta = R @ self._offset
            yield ShipPose(
                timestamp=pose.timestamp,
                x=pose.x + delta[0],
                y=pose.y + delta[1],
                z=pose.z + delta[2],
                roll=pose.roll,
                pitch=pose.pitch,
                yaw=pose.yaw,
            )

    def close(self):
        self._source.close()

