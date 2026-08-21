"""Entry point: python -m ship_emulation.run --help"""

import argparse
import logging
import sys

from ship_emulation.config import (
    AugmentationConfig,
    EmulatorConfig,
    RobotConfig,
    SafetyConfig,
    SinusoidalOverlay,
    WorkspaceLimits,
)
from ship_emulation.data_source import AugmentedSource, CsvFileSource, SmoothedSource
from ship_emulation.emulator import EmulationError, ShipEmulator
from ship_emulation.robot_interface import UR16Interface
from ship_emulation.safety import SafetyChecker


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python -m ship_emulation.run",
        description="Replay ship simulation data on a UR16 robot arm.",
    )
    p.add_argument("--csv-file", metavar="PATH", required=True)
    p.add_argument("--robot-ip", default="192.168.1.100", metavar="IP")
    p.add_argument("--x-range", type=float, nargs=2, default=[-500, 500], metavar=("MIN", "MAX"))
    p.add_argument("--y-range", type=float, nargs=2, default=[-500, 500], metavar=("MIN", "MAX"))
    p.add_argument("--z-range", type=float, nargs=2, default=[-500, 500], metavar=("MIN", "MAX"))
    p.add_argument("--linear-scale", type=float, default=1000.0, metavar="FACTOR",
                   help="Scale applied to x/y/z (default 1000 converts m → mm)")
    p.add_argument("--angular-scale", type=float, default=1.0, metavar="FACTOR",
                   help="Scale applied to roll/pitch/yaw (default 1.0)")
    p.add_argument("--overlay-roll-amp", type=float, default=0.0, metavar="DEG")
    p.add_argument("--overlay-roll-freq", type=float, default=0.1, metavar="HZ")
    p.add_argument("--overlay-pitch-amp", type=float, default=0.0, metavar="DEG")
    p.add_argument("--overlay-pitch-freq", type=float, default=0.1, metavar="HZ")
    p.add_argument("--overlay-yaw-amp", type=float, default=0.0, metavar="DEG")
    p.add_argument("--overlay-yaw-freq", type=float, default=0.1, metavar="HZ")
    p.add_argument("--smooth-linear", type=float, default=0.0, metavar="S",
                   help="Spline smoothing factor for linear channels in mm² (0 = disabled)")
    p.add_argument("--smooth-angular", type=float, default=0.0, metavar="S",
                   help="Spline smoothing factor for angular channels in deg² (0 = disabled)")
    p.add_argument("--dry-run", action="store_true", help="Validate data only, no robot connection")
    p.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return p


def main(argv=None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    workspace = WorkspaceLimits(
        x_min=args.x_range[0], x_max=args.x_range[1],
        y_min=args.y_range[0], y_max=args.y_range[1],
        z_min=args.z_range[0], z_max=args.z_range[1],
    )
    safety_cfg = SafetyConfig(workspace=workspace)
    robot_cfg = RobotConfig(ip=args.robot_ip)
    config = EmulatorConfig(robot=robot_cfg, safety=safety_cfg)
    safety = SafetyChecker(safety_cfg)

    has_overlay = any([
        args.overlay_roll_amp, args.overlay_pitch_amp, args.overlay_yaw_amp,
    ])
    augmentation = AugmentationConfig(
        linear_scale=args.linear_scale,
        angular_scale=args.angular_scale,
        overlay=SinusoidalOverlay(
            roll_amplitude=args.overlay_roll_amp,
            roll_frequency=args.overlay_roll_freq,
            pitch_amplitude=args.overlay_pitch_amp,
            pitch_frequency=args.overlay_pitch_freq,
            yaw_amplitude=args.overlay_yaw_amp,
            yaw_frequency=args.overlay_yaw_freq,
        ) if has_overlay else None,
    )

    raw_source = CsvFileSource(args.csv_file)
    augmented = AugmentedSource(raw_source, augmentation)
    source_ctx = SmoothedSource(augmented, args.smooth_linear, args.smooth_angular) if (args.smooth_linear > 0 or args.smooth_angular > 0) else augmented

    if args.dry_run:
        print("Dry run — validating data only, no robot connection.")
        try:
            with source_ctx:
                for pose in source_ctx.poses():
                    safety.check(pose)
            print("Validation complete — no safety violations.")
        except Exception as e:
            print(f"Validation failed: {e}", file=sys.stderr)
            return 1
        return 0

    print(
        "\nWARNING: This will move the UR16 robot arm.\n"
        "Ensure the workspace is clear and the physical E-stop is within reach.\n"
        "Press ENTER to connect and start, or Ctrl+C to abort.\n"
    )
    try:
        input()
    except KeyboardInterrupt:
        print("\nAborted.")
        return 0

    try:
        with source_ctx, UR16Interface(config.robot) as interface:
            emulator = ShipEmulator(source_ctx, interface, safety, config)
            emulator.run()
    except EmulationError as e:
        print(f"Emulation error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main([
        "--csv-file", "ship_emulation/vessel_motion_clean.csv",
        "--robot-ip", "172.17.0.2",
        "--x-range", "-500", "500",
        "--y-range", "-500", "500",
        "--z-range", "-500", "500",
        "--linear-scale", "1000.0",
        "--angular-scale", "1.0",
        "--overlay-roll-amp", "0.0",
        "--overlay-roll-freq", "0.1",
        "--overlay-pitch-amp", "0.0",
        "--overlay-pitch-freq", "0.1",
        "--overlay-yaw-amp", "0.0",
        "--overlay-yaw-freq", "0.1",
        # "--smooth-linear", "1e5",
        # "--smooth-angular", "1e2",
        "--log-level", "DEBUG",
        # "--dry-run",
    ]))
