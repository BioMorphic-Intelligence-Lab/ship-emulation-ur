from dataclasses import dataclass, field
from typing import Optional, Tuple


@dataclass
class WorkspaceLimits:
    x_min: float = -400.0
    x_max: float = 700.0
    y_min: float = -950.0
    y_max: float = 950.0
    z_min: float = -310.0
    z_max: float = 310.0
    roll_min: float = -90.0
    roll_max: float = 90.0
    pitch_min: float = -90.0
    pitch_max: float = 90.0
    yaw_min: float = -30.0
    yaw_max: float = 30.0


@dataclass
class SafetyConfig:
    workspace: WorkspaceLimits = field(default_factory=WorkspaceLimits)
    max_linear_rate: float = 850.0   # mm/s
    max_angular_rate: float = 50.0   # deg/s
    min_dt: float = 0.005            # s


@dataclass
class RobotConfig:
    #ip: str = "172.17.0.2" # Simulated robot
    ip: str = "192.168.56.101" # Real robot
    tcp: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    work_frame: Tuple[float, ...] = (-650.0, -150.0, 400.0, -90.0, 0.0, 90.0) # Extrinsic rotations
    home_pose: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # work-frame (x y z roll pitch yaw) in mm/deg
    linear_speed: float = 500.0      # mm/s
    angular_speed: float = 20.0      # deg/s
    linear_accel: float = 500.0      # mm/s²
    angular_accel: float = 50.0      # deg/s²
    blend_radius: float = 0.0        # mm


@dataclass
class SinusoidalOverlay:
    """Per-axis sinusoidal signal added on top of rotation channels."""
    roll_amplitude: float = 0.0   # deg
    roll_frequency: float = 0.0   # Hz
    pitch_amplitude: float = 0.0  # deg
    pitch_frequency: float = 0.0  # Hz
    yaw_amplitude: float = 0.0    # deg
    yaw_frequency: float = 0.0    # Hz


@dataclass
class AugmentationConfig:
    linear_scale: float = 1000.0          # unit conversion applied to x/y/z (default: m → mm)
    angular_scale: float = 1.0            # multiplicative scale applied to roll/pitch/yaw
    overlay: Optional[SinusoidalOverlay] = None


@dataclass
class EmulatorConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    augmentation: AugmentationConfig = field(default_factory=AugmentationConfig)
