from pathlib import Path

import pytest

from realman_wrapper import (
    ArmCalibration,
    ArmConfig,
    BimanualConfig,
    BimanualRig,
    CalibrationProvenance,
    FakeFollowerBackend,
    FakeForwardKinematicsBackend,
    FakeFrameStreamBackend,
    FakeUGripperBackend,
    FollowerArm,
    FrameSample,
    GripperCalibration,
    JointUnit,
    LifecycleState,
    MotionAuthorizationError,
    MotionTarget,
    SafetyViolation,
    Side,
    StreamConfig,
)


def provenance() -> CalibrationProvenance:
    return CalibrationProvenance(
        source="test-fixture",
        source_revision="fixture-v1",
        source_path="tests/test_controller.py",
        recorded_at="2026-08-17T00:00:00Z",
        operator="pytest",
        method="synthetic-fake-only",
    )


def config(tmp_path: Path, side: Side = Side.LEFT, **overrides: object) -> ArmConfig:
    values: dict[str, object] = {
        "side": side,
        "robot_serial": f"RM75-TEST-{side.value.upper()}",
        "calibration_file": tmp_path / f"{side.value}.json",
        "max_joint_step": (0.2,) * 7,
        "max_state_age_s": 0.5,
        "max_command_age_s": 0.2,
        "authorization_ttl_s": 0.4,
    }
    values.update(overrides)
    return ArmConfig(**values)  # type: ignore[arg-type]


def calibration(item: ArmConfig, *, with_gripper: bool = False) -> ArmCalibration:
    return ArmCalibration(
        side=item.side,
        robot_serial=item.robot_serial,
        joint_names=item.joint_names,
        joint_unit=item.joint_unit,
        offsets=(0.0,) * 7,
        directions=(1,) * 7,
        hard_min=(-1.0,) * 7,
        hard_max=(1.0,) * 7,
        provenance=provenance(),
        gripper=(
            GripperCalibration(raw_open=0.0, raw_closed=100.0)
            if with_gripper
            else None
        ),
    )


def calibrated_arm(
    tmp_path: Path,
    now: list[float],
    *,
    side: Side = Side.LEFT,
    backend: FakeFollowerBackend | None = None,
    gripper: FakeUGripperBackend | None = None,
    streams: dict[str, FakeFrameStreamBackend] | None = None,
    **overrides: object,
) -> tuple[FollowerArm, FakeFollowerBackend]:
    item = config(tmp_path, side, **overrides)
    follower = backend or FakeFollowerBackend((0.0,) * 7, clock=lambda: now[0])
    arm = FollowerArm(
        item,
        follower,
        gripper=gripper,
        streams=streams,
        clock=lambda: now[0],
    )
    arm.set_calibration(calibration(item, with_gripper=gripper is not None), persist=False)
    arm.connect_read_only()
    return arm, follower


def test_connect_is_read_only_and_observation_is_fresh(tmp_path: Path) -> None:
    now = [10.0]
    arm, backend = calibrated_arm(tmp_path, now)
    observation = arm.observe()
    assert observation.joint_positions == (0.0,) * 7
    assert backend.motion_enabled is False
    assert backend.writes == []
    assert arm.state is LifecycleState.READ_ONLY


def test_missing_calibration_blocks_motion_but_not_observation(tmp_path: Path) -> None:
    now = [10.0]
    item = config(tmp_path)
    backend = FakeFollowerBackend((0.0,) * 7, clock=lambda: now[0])
    arm = FollowerArm(item, backend, clock=lambda: now[0])
    arm.connect_read_only()
    assert arm.observe().joint_positions is None
    target = MotionTarget((0.1,) * 7, now[0])
    with pytest.raises(SafetyViolation, match="calibration"):
        arm.authorize_next_motion(
            target,
            reason="fake test",
            acknowledgement=item.acknowledgement,
        )
    assert backend.writes == []


def test_authorization_is_target_bound_one_shot_and_disarms(tmp_path: Path) -> None:
    now = [10.0]
    arm, backend = calibrated_arm(tmp_path, now)
    target = MotionTarget((0.1,) * 7, now[0])
    authorization = arm.authorize_next_motion(
        target,
        reason="fake unit test",
        acknowledgement=arm.config.acknowledgement,
    )
    arm.execute_authorized(target, authorization)
    assert backend.positions == pytest.approx((0.1,) * 7)
    assert backend.motion_enabled is False
    assert arm.state is LifecycleState.READ_ONLY
    with pytest.raises(MotionAuthorizationError, match="already-consumed"):
        arm.execute_authorized(target, authorization)


def test_wrong_acknowledgement_and_modified_target_never_write(tmp_path: Path) -> None:
    now = [10.0]
    arm, backend = calibrated_arm(tmp_path, now)
    target = MotionTarget((0.1,) * 7, now[0])
    with pytest.raises(MotionAuthorizationError, match="identity"):
        arm.authorize_next_motion(target, reason="test", acknowledgement="yes")
    authorization = arm.authorize_next_motion(
        target,
        reason="test",
        acknowledgement=arm.config.acknowledgement,
    )
    modified = MotionTarget((0.05,) * 7, now[0])
    with pytest.raises(MotionAuthorizationError, match="differs"):
        arm.execute_authorized(modified, authorization)
    assert backend.writes == []


def test_stale_and_oversize_targets_fail_closed_before_write(tmp_path: Path) -> None:
    now = [10.0]
    arm, backend = calibrated_arm(tmp_path, now)
    stale = MotionTarget((0.1,) * 7, 9.0)
    with pytest.raises(Exception, match="stale"):
        arm.authorize_next_motion(
            stale,
            reason="test",
            acknowledgement=arm.config.acknowledgement,
        )
    oversize = MotionTarget((0.3,) * 7, now[0])
    with pytest.raises(SafetyViolation, match="delta"):
        arm.authorize_next_motion(
            oversize,
            reason="test",
            acknowledgement=arm.config.acknowledgement,
        )
    assert backend.writes == []


def test_gripper_is_separate_calibrated_backend(tmp_path: Path) -> None:
    now = [10.0]
    gripper = FakeUGripperBackend(0.0, clock=lambda: now[0])
    arm, backend = calibrated_arm(
        tmp_path,
        now,
        gripper=gripper,
        max_gripper_step=0.2,
    )
    target = MotionTarget((0.05,) * 7, now[0], gripper_position=0.1)
    authorization = arm.authorize_next_motion(
        target,
        reason="fake gripper test",
        acknowledgement=arm.config.acknowledgement,
    )
    arm.execute_authorized(target, authorization)
    assert backend.writes
    assert gripper.writes == [10.0]
    assert gripper.motion_enabled is False


class StaleStream(FakeFrameStreamBackend):
    def read_latest(self) -> FrameSample:
        return FrameSample(b"fake", 0.0, 1, "fake", (1, 1, 1))


def test_configured_stale_stream_faults_arm(tmp_path: Path) -> None:
    now = [10.0]
    stream = StaleStream(b"fake", encoding="fake", shape=(1, 1, 1))
    item = config(
        tmp_path,
        streams=(StreamConfig(name="left_flux_0", kind="dmrobotics_flux", max_age_s=0.1),),
    )
    backend = FakeFollowerBackend((0.0,) * 7, clock=lambda: now[0])
    arm = FollowerArm(item, backend, streams={"left_flux_0": stream}, clock=lambda: now[0])
    with pytest.raises(Exception, match="stale"):
        arm.connect_read_only()
    assert arm.state is LifecycleState.FAULT
    assert backend.motion_enabled is False


def test_fk_receives_explicit_side_order_unit_and_frames(tmp_path: Path) -> None:
    now = [10.0]
    arm, _ = calibrated_arm(tmp_path, now, side=Side.RIGHT)
    fk = FakeForwardKinematicsBackend()
    pose = arm.forward_kinematics(fk, base_frame="right_base", tool_frame="right_flange")
    assert pose.base_frame == "right_base"
    request = fk.requests[0]
    assert request.side is Side.RIGHT
    assert request.joint_names == arm.config.joint_names
    assert request.joint_unit is JointUnit.RADIANS


class FailingWriteBackend(FakeFollowerBackend):
    def write_joint_target(self, positions, unit):  # type: ignore[no-untyped-def]
        raise RuntimeError("injected write failure")


def test_bimanual_failure_emergency_stops_both_sides(tmp_path: Path) -> None:
    now = [10.0]
    left_config = config(tmp_path, Side.LEFT)
    right_config = config(tmp_path, Side.RIGHT)
    left_backend = FakeFollowerBackend((0.0,) * 7, clock=lambda: now[0])
    right_backend = FailingWriteBackend((0.0,) * 7, clock=lambda: now[0])
    left = FollowerArm(left_config, left_backend, clock=lambda: now[0])
    right = FollowerArm(right_config, right_backend, clock=lambda: now[0])
    left.set_calibration(calibration(left_config), persist=False)
    right.set_calibration(calibration(right_config), persist=False)
    rig_config = BimanualConfig(left_config, right_config)
    rig = BimanualRig(rig_config, left, right, clock=lambda: now[0])
    rig.connect_read_only()
    observation = rig.observe()
    assert observation.joint_vector((Side.RIGHT, Side.LEFT)) == (0.0,) * 14

    targets = {
        Side.LEFT: MotionTarget((0.05,) * 7, now[0]),
        Side.RIGHT: MotionTarget((0.05,) * 7, now[0]),
    }
    authorization = rig.authorize_next_motion(
        targets,
        reason="bimanual fake test",
        acknowledgement=rig_config.acknowledgement,
    )
    with pytest.raises(RuntimeError, match="injected write failure"):
        rig.execute_authorized(targets, authorization)
    assert left_backend.emergency_stops == 1
    assert right_backend.emergency_stops == 1
    assert left.state is LifecycleState.FAULT
    assert right.state is LifecycleState.FAULT
