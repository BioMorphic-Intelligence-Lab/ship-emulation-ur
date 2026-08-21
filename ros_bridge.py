"""ROS 2 bridge — publishes commanded and actual robot poses for bag logging.

Drop-in for any ShipEmulator-based experiment:

    bridge = RosBridge("phase1")
    bridge.start_feedback(lambda: interface.current_pose, rate_hz=50.0)
    bridge.set_context("level_1/shear-x")   # optional tag, appears in frame_id
    emulator = ShipEmulator(..., on_move=bridge)
    emulator.run()
    bridge.close()

Topics
------
    /robot/cmd_pose    — geometry_msgs/PoseStamped  commanded pose (work frame, m)
    /robot/actual_pose — geometry_msgs/PoseStamped  actual FK pose (work frame, m)

Both topics express poses in the **robot work frame** (i.e. the frame whose
origin and orientation are defined by RobotConfig.work_frame relative to the
robot base).  They are NOT in the robot base frame.

Unit conventions:
    position  : metres  (converted from mm internally)
    orientation: quaternion derived from intrinsic-XYZ Euler angles in degrees,
                 matching the cri / ShipPose convention.

frame_id format: <coord_frame>/<experiment>[/<context>]
    coord_frame : name of the coordinate frame the pose is expressed in
                  (default "work_frame" — set via RosBridge(coord_frame=...))
    experiment  : node/experiment label passed to RosBridge()
    context     : optional sub-label set via set_context(), e.g. "level_1/shear-x"

Degrades silently if rclpy is not importable (no-op publishes).
"""
import logging
import math
import threading
from typing import Callable, Optional, Tuple

log = logging.getLogger(__name__)

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    _HAS_ROS2 = True
except ImportError:
    _HAS_ROS2 = False

_PoseTuple = Tuple[float, float, float, float, float, float]  # x y z roll pitch yaw


def _euler_xyz_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float, float]:
    """Intrinsic XYZ Euler angles (degrees) → quaternion (x, y, z, w)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        sr * cp * cy + cr * sp * sy,   # x
        cr * sp * cy - sr * cp * sy,   # y
        cr * cp * sy + sr * sp * cy,   # z
        cr * cp * cy - sr * sp * sy,   # w
    )


class RosBridge:
    """Publishes commanded and actual robot poses to ROS 2 topics.

    Can be used as the ``on_move`` callable for ShipEmulator — it accepts
    either a ShipPose or a raw (x, y, z, roll, pitch, yaw) tuple.

    Args:
        experiment: node name and prefix for the frame_id (e.g. "phase1")
        feedback_rate_hz: polling rate for actual pose (default 50 Hz)
    """

    CMD_TOPIC    = "/robot/cmd_pose"
    ACTUAL_TOPIC = "/robot/actual_pose"

    def __init__(self, experiment: str = "robot", coord_frame: str = "work_frame"):
        self._experiment   = experiment
        self._coord_frame  = coord_frame
        self._context      = ""
        self._available    = _HAS_ROS2

        if not _HAS_ROS2:
            log.warning("rclpy not available — RosBridge is a no-op")
            return

        rclpy.init()
        self._node      = Node(f"{experiment}_ros_bridge")
        self._cmd_pub   = self._node.create_publisher(PoseStamped, self.CMD_TOPIC,    10)
        self._act_pub   = self._node.create_publisher(PoseStamped, self.ACTUAL_TOPIC, 10)

        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

        self._stop_feedback   = threading.Event()
        self._feedback_thread: Optional[threading.Thread] = None

        log.info(
            "RosBridge started (experiment=%r)  cmd=%s  actual=%s",
            experiment, self.CMD_TOPIC, self.ACTUAL_TOPIC,
        )

    # ── context ───────────────────────────────────────────────────────────────

    def set_context(self, context: str) -> None:
        """Tag subsequent messages with a sub-label (e.g. 'level_1/shear-x')."""
        self._context = context

    def _frame_id(self) -> str:
        parts = [self._coord_frame, self._experiment, self._context]
        return "/".join(p for p in parts if p)

    # ── publishing ────────────────────────────────────────────────────────────

    def _make_msg(self, x_mm: float, y_mm: float, z_mm: float,
                  roll: float, pitch: float, yaw: float) -> "PoseStamped":
        msg = PoseStamped()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id()
        msg.pose.position.x = x_mm / 1000.0
        msg.pose.position.y = y_mm / 1000.0
        msg.pose.position.z = z_mm / 1000.0
        qx, qy, qz, qw = _euler_xyz_to_quat(roll, pitch, yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def __call__(self, pose) -> None:
        """Called by ShipEmulator after each move_to.  Accepts ShipPose or tuple."""
        if not self._available:
            return
        if hasattr(pose, "x"):
            self._cmd_pub.publish(
                self._make_msg(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw)
            )
        else:
            self._cmd_pub.publish(self._make_msg(*pose))

    # ── actual-pose feedback loop ─────────────────────────────────────────────

    def start_feedback(
        self,
        get_pose: Callable[[], Optional[_PoseTuple]],
        rate_hz: float = 50.0,
    ) -> None:
        """Poll get_pose() at rate_hz and publish to ACTUAL_TOPIC.

        Args:
            get_pose: zero-arg callable returning (x, y, z, roll, pitch, yaw)
                      in mm/deg, or None if unavailable.  Pass
                      ``lambda: interface.current_pose`` for UR16Interface.
            rate_hz:  polling frequency (default 50 Hz).
        """
        if not self._available:
            return

        self._stop_feedback.clear()
        dt = 1.0 / rate_hz

        def _loop() -> None:
            while not self._stop_feedback.wait(dt):
                try:
                    pose = get_pose()
                except Exception:
                    continue
                if pose is None:
                    continue
                try:
                    self._act_pub.publish(self._make_msg(*pose))
                except Exception:
                    pass

        self._feedback_thread = threading.Thread(target=_loop, daemon=True)
        self._feedback_thread.start()

    def stop_feedback(self) -> None:
        self._stop_feedback.set()
        if self._feedback_thread is not None:
            self._feedback_thread.join(timeout=2.0)
            self._feedback_thread = None

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def close(self) -> None:
        if not self._available:
            return
        self.stop_feedback()
        self._node.destroy_node()
        rclpy.shutdown()
