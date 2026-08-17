import json
from pathlib import Path

import pytest

from realman_wrapper import (
    ArmCalibration,
    ArmConfig,
    CalibrationProvenance,
    GripperCalibration,
    JointUnit,
    NetworkEndpoint,
    Side,
    load_bimanual_config,
)


def provenance() -> CalibrationProvenance:
    return CalibrationProvenance(
        source="lab-calibration-log",
        source_revision="sha256:0123456789abcdef",
        source_path="records/rm75-left.json",
        recorded_at="2026-08-17T00:00:00Z",
        operator="offline-test",
        method="fixture-measurement",
    )


def calibration(side: Side = Side.LEFT, serial: str = "RM75-TEST-L") -> ArmCalibration:
    return ArmCalibration(
        side=side,
        robot_serial=serial,
        joint_names=tuple(f"joint_{index}" for index in range(1, 8)),
        joint_unit=JointUnit.RADIANS,
        offsets=(0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        directions=(1, -1, 1, 1, 1, 1, 1),
        hard_min=(-1.0,) * 7,
        hard_max=(1.0,) * 7,
        provenance=provenance(),
        gripper=GripperCalibration(raw_open=100.0, raw_closed=900.0),
    )


def test_calibration_roundtrip_is_atomic_and_preserves_provenance(tmp_path: Path) -> None:
    path = tmp_path / "left.json"
    expected = calibration()
    expected.save(path)
    assert ArmCalibration.load(path) == expected
    assert not list(tmp_path.glob("tmp*"))


def test_calibration_applies_sign_and_offset_both_directions() -> None:
    item = calibration()
    raw = (0.2, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0)
    canonical = item.to_canonical(raw)
    assert canonical[:2] == pytest.approx((0.3, -0.4))
    assert item.to_raw(canonical) == pytest.approx(raw)


def test_calibration_identity_is_bound_to_exact_side_and_serial(tmp_path: Path) -> None:
    config = ArmConfig(
        side=Side.RIGHT,
        robot_serial="RM75-TEST-R",
        calibration_file=tmp_path / "right.json",
        max_joint_step=(0.1,) * 7,
    )
    with pytest.raises(ValueError, match="does not match"):
        calibration().assert_compatible(config)


def test_network_endpoint_has_no_runnable_wildcard_default() -> None:
    with pytest.raises(ValueError, match="explicit remote"):
        NetworkEndpoint("0.0.0.0", 8080)
    with pytest.raises(ValueError, match="port"):
        NetworkEndpoint("rm75.example.invalid", 0)


def test_load_bimanual_config_resolves_calibration_paths(tmp_path: Path) -> None:
    payload = {
        "schema": "lv_robotics.realman_bimanual_config.v1",
        "left": {
            "side": "left",
            "robot_serial": "LEFT-SERIAL",
            "calibration_file": "calibration/left.json",
            "max_joint_step": [0.1] * 7,
        },
        "right": {
            "side": "right",
            "robot_serial": "RIGHT-SERIAL",
            "calibration_file": "calibration/right.json",
            "max_joint_step": [0.2] * 7,
        },
    }
    path = tmp_path / "site.json"
    path.write_text(json.dumps(payload))
    config = load_bimanual_config(path)
    assert config.left.calibration_file == tmp_path / "calibration/left.json"
    assert config.right.follower_endpoint is None
    assert config.left.max_joint_step == (0.1,) * 7


def test_bimanual_config_rejects_swapped_side(tmp_path: Path) -> None:
    payload = {
        "schema": "lv_robotics.realman_bimanual_config.v1",
        "left": {
            "side": "right",
            "robot_serial": "LEFT-SERIAL",
            "calibration_file": "left.json",
            "max_joint_step": [0.1] * 7,
        },
        "right": {
            "side": "left",
            "robot_serial": "RIGHT-SERIAL",
            "calibration_file": "right.json",
            "max_joint_step": [0.1] * 7,
        },
    }
    path = tmp_path / "site.json"
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="expected left"):
        load_bimanual_config(path)
