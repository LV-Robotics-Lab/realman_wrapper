from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path

from .calibration import ArmCalibration
from .config import RM75_DOF, ArmConfig, load_bimanual_config
from .controller import FollowerArm
from .fakes import FakeFollowerBackend
from .model import Side


def _fake_doctor() -> int:
    with tempfile.TemporaryDirectory(prefix="realman-wrapper-doctor-") as directory:
        config = ArmConfig(
            side=Side.LEFT,
            robot_serial="SYNTHETIC-DOCTOR-ONLY",
            calibration_file=Path(directory) / "not-created.json",
            max_joint_step=(0.01,) * RM75_DOF,
        )
        backend = FakeFollowerBackend((0.0,) * RM75_DOF)
        arm = FollowerArm(config, backend)
        try:
            arm.connect_read_only()
            observation = arm.observe()
        finally:
            arm.disconnect()
    print(
        json.dumps(
            {
                "hardware_touched": False,
                "mode": "synthetic-read-only",
                "motion_exercised": False,
                "observed_joint_count": len(observation.raw_joint_sample.positions),
                "status": "ok",
            },
            sort_keys=True,
        )
    )
    return 0


def _config_doctor(path: Path, require_motion_ready: bool) -> int:
    config = load_bimanual_config(path)
    arms = (config.left, config.right)
    reports: list[dict[str, object]] = []
    motion_ready = True
    for arm in arms:
        calibration_status = "missing"
        if arm.calibration_file.is_file():
            calibration = ArmCalibration.load(arm.calibration_file)
            calibration.assert_compatible(arm)
            calibration_status = "valid"
        else:
            motion_ready = False
        endpoint_present = arm.follower_endpoint is not None
        motion_ready = motion_ready and endpoint_present
        reports.append(
            {
                "side": arm.side.value,
                "robot_serial": arm.robot_serial,
                "follower_endpoint_present": endpoint_present,
                "gripper_backend_required": arm.gripper_endpoint is not None,
                "calibration": calibration_status,
                "stream_count": len(arm.streams),
            }
        )
    result = {
        "hardware_touched": False,
        "mode": "static-config",
        "motion_ready_metadata": motion_ready,
        "note": "No vendor backend was imported and no endpoint was opened.",
        "arms": reports,
        "status": "ok" if motion_ready or not require_motion_ready else "not_motion_ready",
    }
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == "ok" else 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline/read-only diagnostics for the backend-neutral RealMan wrapper"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    doctor = subparsers.add_parser("doctor")
    doctor.add_argument(
        "--config",
        type=Path,
        help="statically validate a bimanual config; never connects to its endpoints",
    )
    doctor.add_argument(
        "--require-motion-ready",
        action="store_true",
        help="fail unless both explicit follower endpoints and matching calibrations exist",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.command == "doctor":
        if args.require_motion_ready and args.config is None:
            raise SystemExit("--require-motion-ready requires --config")
        return (
            _config_doctor(args.config, args.require_motion_ready)
            if args.config is not None
            else _fake_doctor()
        )
    raise AssertionError(f"unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
