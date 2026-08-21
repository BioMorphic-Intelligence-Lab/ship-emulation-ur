import logging
import math
import time
from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation
from cri.robot import SyncRobot, AsyncRobot
from cri.controller import RTDEController

from ship_emulation.config import RobotConfig

log = logging.getLogger(__name__)


class UR16Interface:
    def __init__(self, config: RobotConfig):
        self._config = config
        self._controller: RTDEController = None
        self._robot: AsyncRobot = None
        self._motion_pending = False
        self._R_work_to_base = np.eye(3)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.close()

    def connect(self):
        cfg = self._config
        self._controller = RTDEController(ip=cfg.ip)
        self._controller.linear_accel = cfg.linear_accel
        self._controller.angular_accel = cfg.angular_accel
        sync = SyncRobot(self._controller)
        self._robot = AsyncRobot(sync)
        self._robot.tcp = cfg.tcp
        self._robot.coord_frame = cfg.work_frame
        self._robot.linear_speed = cfg.linear_speed
        self._robot.angular_speed = cfg.angular_speed
        self._robot.blend_radius = cfg.blend_radius
        self._motion_pending = False

        # Precompute work-frame rotation matrix for velocity transformation.
        # speedL operates in the base frame — CRI's coord_frame is not applied.
        # We must rotate work-frame velocities into the base frame ourselves.
        # CRI default Euler convention is sxyz (static/extrinsic XYZ = scipy 'xyz').
        rx, ry, rz = cfg.work_frame[3], cfg.work_frame[4], cfg.work_frame[5]
        self._R_work_to_base = Rotation.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()

        log.info("Connected to UR16 at %s — %s", cfg.ip, self._robot.info)

    def close(self):
        if self._robot is not None:
            try:
                if self._motion_pending:
                    self._robot.async_result()
                    self._motion_pending = False
            except Exception:
                pass
            self._robot.close()
            self._robot = None
            self._controller = None
            self._motion_pending = False

    def move_home(self):
        if self._motion_pending:
            self._robot.async_result()
            self._motion_pending = False
        self._robot.move_linear(self._config.home_pose)
        log.info("At home position")

    def servo_linear_velocity(
        self,
        velocity: Tuple[float, ...],
        accel: float,
        return_time: float,
    ) -> None:
        """Stream one Cartesian velocity segment via speedL.

        velocity must be expressed in the **work frame** (same convention as
        all other interface methods).  It is rotated into the base frame here
        before being passed to speedL, which always operates in the base frame.

        Returns almost immediately — the robot executes the velocity on a
        background thread for return_time seconds.  The next call joins that
        thread first, so consecutive calls produce gapless back-to-back motion.

        velocity = (vx, vy, vz, v_roll, v_pitch, v_yaw)  mm/s and deg/s
        accel    = linear acceleration  mm/s²
        return_time = segment duration  s
        """
        if self._motion_pending:
            self._robot.async_result()
            self._motion_pending = False
        v = np.asarray(velocity, dtype=np.float64)
        v_base = np.concatenate([
            self._R_work_to_base @ v[:3],
            self._R_work_to_base @ v[3:],
        ])
        self._controller.move_linear_velocity(v_base, accel, return_time)

    def stop_linear(self, linear_accel: float = 2000.0) -> None:
        """Decelerate TCP smoothly to rest.  Does not close the connection."""
        if self._motion_pending:
            self._robot.async_result()
            self._motion_pending = False
        self._controller.stop_linear_velocity(linear_accel)

    def move_linear_at(
        self,
        pose: Tuple[float, ...],
        linear_speed: float,
        angular_speed: float,
    ) -> None:
        """Blocking linear move at explicitly specified speeds.

        Temporarily overrides the robot's configured speeds for this move,
        then restores them.  Use for slow positioning moves where the default
        configured speed is too fast.
        """
        if self._motion_pending:
            self._robot.async_result()
            self._motion_pending = False
        orig_linear  = self._robot.linear_speed
        orig_angular = self._robot.angular_speed
        self._robot.linear_speed  = linear_speed
        self._robot.angular_speed = angular_speed
        try:
            self._robot.move_linear(pose)
        finally:
            self._robot.linear_speed  = orig_linear
            self._robot.angular_speed = orig_angular

    def move_to(self, pose: Tuple[float, ...]):
        if self._motion_pending:
            self._robot.async_result()
            self._motion_pending = False
        log.debug("move_to: x=%.1f y=%.1f z=%.1f roll=%.3f pitch=%.3f yaw=%.3f", *pose)
        self._robot.async_move_linear(pose)
        self._motion_pending = True

    def wait_for_motion(self):
        if self._motion_pending:
            self._robot.async_result()
            self._motion_pending = False

    def wait_until_stopped(self, threshold_mm_s: float = 2.0, timeout_s: float = 10.0) -> None:
        """Poll actual TCP speed until the robot is physically at rest."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            vel = self._controller.linear_velocity
            if math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2) < threshold_mm_s:
                return
            time.sleep(0.05)
        log.warning("wait_until_stopped: robot did not stop within %.1f s", timeout_s)

    def is_motion_done(self) -> bool:
        if not self._motion_pending:
            return True
        if self._robot.async_done():
            self._robot.async_result()
            self._motion_pending = False
            return True
        return False

    def emergency_stop(self):
        log.warning("Emergency stop triggered — software stop only; use physical E-stop as primary")
        try:
            self._controller.stop_linear_velocity(5000)
        except Exception as e:
            log.error("Emergency stop command failed: %s", e)
        finally:
            self.close()

    @property
    def current_pose(self):
        return self._robot.pose if self._robot else None

    @property
    def current_joint_angles(self):
        return self._robot.joint_angles if self._robot else None

    @property
    def runtime_state(self) -> int:
        """URScript program state: 0=Stopping 1=Stopped 2=Playing 3=Paused."""
        try:
            return self._controller._client.get_runtime_state()
        except Exception:
            return -1
