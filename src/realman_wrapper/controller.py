from __future__ import annotations

import hashlib
import json
import secrets
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass

from .backends import (
    FollowerArmBackend,
    ForwardKinematicsBackend,
    FrameStreamBackend,
    LeaderArmBackend,
    UGripperBackend,
)
from .calibration import ArmCalibration
from .config import ArmConfig, BimanualConfig, LeaderConfig
from .errors import MotionAuthorizationError, SafetyViolation, StaleSampleError
from .model import (
    CartesianPose,
    ForwardKinematicsRequest,
    FrameSample,
    JointSample,
    LifecycleState,
    MotionAuthorization,
    MotionTarget,
    Side,
)


def _assert_fresh(timestamp: float, max_age_s: float, now: float, label: str) -> None:
    age = now - timestamp
    if age < -0.010:
        raise StaleSampleError(f"{label} timestamp is from a different or future monotonic clock")
    if age > max_age_s:
        raise StaleSampleError(f"{label} is stale ({age:.6f}s > {max_age_s:.6f}s)")


def _target_payload(target: MotionTarget) -> dict[str, object]:
    return {
        "joint_positions": list(target.joint_positions),
        "gripper_position": target.gripper_position,
        "monotonic_timestamp": target.monotonic_timestamp,
    }


def _digest(payload: object) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class ArmObservation:
    side: Side
    raw_joint_sample: JointSample
    joint_positions: tuple[float, ...] | None
    raw_gripper_position: float | None
    gripper_position: float | None
    frames: dict[str, FrameSample]


@dataclass(frozen=True)
class BimanualObservation:
    arms: dict[Side, ArmObservation]

    def joint_vector(self, side_order: Sequence[Side]) -> tuple[float, ...]:
        """Flatten calibrated joints only with a caller-supplied side order."""
        order = tuple(side_order)
        if set(order) != set(self.arms) or len(order) != len(self.arms):
            raise ValueError("side_order must contain every observed side exactly once")
        output: list[float] = []
        for side in order:
            positions = self.arms[side].joint_positions
            if positions is None:
                raise RuntimeError(f"{side.value} arm has no loaded calibration")
            output.extend(positions)
        return tuple(output)


class FollowerArm:
    """Read-mostly RM75 lifecycle with target-bound, one-shot motion grants."""

    def __init__(
        self,
        config: ArmConfig,
        backend: FollowerArmBackend,
        *,
        gripper: UGripperBackend | None = None,
        streams: Mapping[str, FrameStreamBackend] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self.backend = backend
        self.gripper = gripper
        self.streams = dict(streams or {})
        self.clock = clock
        self.state = LifecycleState.DISCONNECTED
        self.fault_reason: str | None = None
        self.calibration: ArmCalibration | None = None
        self._authorization: MotionAuthorization | None = None

        configured_streams = {stream.name for stream in config.streams}
        if set(self.streams) != configured_streams:
            raise ValueError(
                "injected streams must exactly match configured streams "
                f"(injected={sorted(self.streams)}, configured={sorted(configured_streams)})"
            )
        if config.gripper_endpoint is not None and gripper is None:
            raise ValueError("gripper_endpoint requires an injected UGripperBackend")
        if config.calibration_file.is_file():
            self.set_calibration(ArmCalibration.load(config.calibration_file), persist=False)

    @property
    def is_connected(self) -> bool:
        return self.state is LifecycleState.READ_ONLY

    @property
    def is_calibrated(self) -> bool:
        return self.calibration is not None

    def set_calibration(self, calibration: ArmCalibration, *, persist: bool = True) -> None:
        calibration.assert_compatible(self.config)
        self.calibration = calibration
        if persist:
            calibration.save(self.config.calibration_file)

    def connect_read_only(self) -> None:
        if self.state is not LifecycleState.DISCONNECTED:
            raise RuntimeError(f"cannot connect from state {self.state.value}")
        connected_streams: list[str] = []
        gripper_connected = False
        follower_connected = False
        try:
            self.backend.connect(self.config.follower_endpoint, read_only=True)
            follower_connected = True
            self.backend.set_motion_enabled(False)
            if self.gripper is not None:
                self.gripper.connect(self.config.gripper_endpoint, read_only=True)
                gripper_connected = True
                self.gripper.set_motion_enabled(False)
            specs = {stream.name: stream for stream in self.config.streams}
            for name, stream in self.streams.items():
                spec = specs[name]
                stream.connect(spec.remote_endpoint, spec.local_endpoint)
                connected_streams.append(name)
            self.state = LifecycleState.READ_ONLY
            self.observe()
        except Exception as error:
            self._authorization = None
            for name in reversed(connected_streams):
                try:
                    self.streams[name].disconnect()
                except Exception:
                    pass
            if gripper_connected and self.gripper is not None:
                try:
                    self.gripper.set_motion_enabled(False)
                    self.gripper.disconnect()
                except Exception:
                    pass
            if follower_connected:
                try:
                    self.backend.set_motion_enabled(False)
                    self.backend.disconnect()
                except Exception:
                    pass
            self.fault_reason = f"connect/read-only probe failed: {error}"
            self.state = LifecycleState.FAULT
            raise

    def observe(self) -> ArmObservation:
        if self.state is not LifecycleState.READ_ONLY:
            raise RuntimeError(f"cannot observe from state {self.state.value}")
        now = self.clock()
        try:
            sample = self.backend.read_joint_state()
            if sample.unit is not self.config.joint_unit:
                raise SafetyViolation(
                    f"backend unit {sample.unit.value} does not match {self.config.joint_unit.value}"
                )
            if len(sample.positions) != len(self.config.joint_names):
                raise SafetyViolation("backend joint count does not match configured RM75 order")
            _assert_fresh(sample.monotonic_timestamp, self.config.max_state_age_s, now, "joint state")
            calibrated = (
                self.calibration.to_canonical(sample.positions)
                if self.calibration is not None
                else None
            )

            raw_gripper: float | None = None
            canonical_gripper: float | None = None
            if self.gripper is not None:
                gripper_sample = self.gripper.read_position()
                _assert_fresh(
                    gripper_sample.monotonic_timestamp,
                    self.config.max_state_age_s,
                    now,
                    "gripper state",
                )
                raw_gripper = gripper_sample.value
                gripper_calibration = self.calibration.gripper if self.calibration else None
                if gripper_calibration is not None:
                    if gripper_sample.unit != gripper_calibration.unit:
                        raise SafetyViolation(
                            "gripper backend unit does not match calibration provenance"
                        )
                    canonical_gripper = gripper_calibration.to_canonical(raw_gripper)

            frames: dict[str, FrameSample] = {}
            specs = {stream.name: stream for stream in self.config.streams}
            for name, stream in self.streams.items():
                frame = stream.read_latest()
                _assert_fresh(
                    frame.monotonic_timestamp,
                    specs[name].max_age_s,
                    now,
                    f"stream {name}",
                )
                frames[name] = frame
            return ArmObservation(
                side=self.config.side,
                raw_joint_sample=sample,
                joint_positions=calibrated,
                raw_gripper_position=raw_gripper,
                gripper_position=canonical_gripper,
                frames=frames,
            )
        except Exception as error:
            self.emergency_stop(f"observation failure: {error}")
            raise

    def forward_kinematics(
        self,
        fk: ForwardKinematicsBackend,
        *,
        base_frame: str,
        tool_frame: str,
    ) -> CartesianPose:
        observation = self.observe()
        if observation.joint_positions is None:
            raise RuntimeError("a matching calibration is required before FK")
        request = ForwardKinematicsRequest(
            side=self.config.side,
            joint_names=self.config.joint_names,
            joint_unit=self.config.joint_unit,
            base_frame=base_frame,
            tool_frame=tool_frame,
        )
        pose = fk.forward(observation.joint_positions, request)
        if pose.base_frame != base_frame or pose.tool_frame != tool_frame:
            self.emergency_stop("FK backend returned unexpected frames")
            raise SafetyViolation("FK backend returned unexpected frames")
        return pose

    def authorize_next_motion(
        self,
        target: MotionTarget,
        *,
        reason: str,
        acknowledgement: str,
    ) -> MotionAuthorization:
        if self.state is not LifecycleState.READ_ONLY:
            raise MotionAuthorizationError(f"cannot authorize from state {self.state.value}")
        if acknowledgement != self.config.acknowledgement:
            raise MotionAuthorizationError("motion acknowledgement does not match this arm identity")
        if not reason.strip():
            raise MotionAuthorizationError("motion reason must not be empty")
        self._prepare_target(target)
        authorization = MotionAuthorization(
            token=secrets.token_urlsafe(32),
            target_digest=_digest(_target_payload(target)),
            expires_at=self.clock() + self.config.authorization_ttl_s,
            scope=f"{self.config.side.value}:{self.config.robot_serial}:{reason.strip()}",
        )
        self._authorization = authorization
        return authorization

    def execute_authorized(
        self,
        target: MotionTarget,
        authorization: MotionAuthorization,
    ) -> None:
        self._consume_authorization(target, authorization)
        raw_joints, raw_gripper = self._prepare_target(target)
        error: Exception | None = None
        try:
            self._arm_motion()
            self._write_raw(raw_joints, raw_gripper)
        except Exception as caught:
            error = caught
        try:
            self._disarm_motion()
        except Exception as caught:
            error = error or caught
        if error is not None:
            self.emergency_stop(f"one-shot motion failed: {error}")
            raise error

    def _consume_authorization(
        self,
        target: MotionTarget,
        authorization: MotionAuthorization,
    ) -> None:
        expected = self._authorization
        self._authorization = None
        if expected is None or authorization != expected:
            raise MotionAuthorizationError("unknown, superseded, or already-consumed authorization")
        if self.clock() > authorization.expires_at:
            raise MotionAuthorizationError("motion authorization expired")
        if authorization.target_digest != _digest(_target_payload(target)):
            raise MotionAuthorizationError("motion target differs from authorized target")

    def _prepare_target(self, target: MotionTarget) -> tuple[tuple[float, ...], float | None]:
        now = self.clock()
        _assert_fresh(
            target.monotonic_timestamp,
            self.config.max_command_age_s,
            now,
            "motion target",
        )
        if len(target.joint_positions) != len(self.config.joint_names):
            raise SafetyViolation("motion target has the wrong joint count")
        if self.calibration is None:
            raise SafetyViolation("motion requires a provenance-bearing calibration")
        if not self.calibration.contains(target.joint_positions):
            raise SafetyViolation("motion target exceeds calibrated hard limits")
        observation = self.observe()
        current = observation.joint_positions
        if current is None:
            raise SafetyViolation("motion requires calibrated current state")
        violations = {
            name: abs(goal - present)
            for name, goal, present, maximum in zip(
                self.config.joint_names,
                target.joint_positions,
                current,
                self.config.max_joint_step,
                strict=True,
            )
            if abs(goal - present) > maximum
        }
        if violations:
            raise SafetyViolation(f"one-shot joint delta exceeds configured limit: {violations}")

        raw_gripper: float | None = None
        if target.gripper_position is not None:
            if self.gripper is None:
                raise SafetyViolation("target includes a gripper but no backend is injected")
            calibration = self.calibration.gripper
            if calibration is None:
                raise SafetyViolation("gripper motion requires a provenance-bearing calibration")
            if self.config.max_gripper_step is None:
                raise SafetyViolation("max_gripper_step must be explicit before gripper motion")
            if not calibration.contains(target.gripper_position):
                raise SafetyViolation("gripper target exceeds calibrated endpoints")
            if observation.gripper_position is None:
                raise SafetyViolation("gripper motion requires calibrated current state")
            delta = abs(target.gripper_position - observation.gripper_position)
            if delta > self.config.max_gripper_step:
                raise SafetyViolation(
                    f"one-shot gripper delta {delta} exceeds {self.config.max_gripper_step}"
                )
            raw_gripper = calibration.to_raw(target.gripper_position)
        return self.calibration.to_raw(target.joint_positions), raw_gripper

    def _arm_motion(self) -> None:
        self.backend.set_motion_enabled(True)
        if self.gripper is not None:
            self.gripper.set_motion_enabled(True)

    def _write_raw(self, raw_joints: tuple[float, ...], raw_gripper: float | None) -> None:
        self.backend.write_joint_target(raw_joints, self.config.joint_unit)
        if raw_gripper is not None and self.gripper is not None and self.calibration is not None:
            assert self.calibration.gripper is not None
            self.gripper.write_position(raw_gripper, self.calibration.gripper.unit)

    def _disarm_motion(self) -> None:
        gripper_error: Exception | None = None
        if self.gripper is not None:
            try:
                self.gripper.set_motion_enabled(False)
            except Exception as error:
                gripper_error = error
        self.backend.set_motion_enabled(False)
        if gripper_error is not None:
            raise gripper_error

    def emergency_stop(self, reason: str) -> None:
        self._authorization = None
        self.fault_reason = reason
        try:
            self.backend.emergency_stop()
        except Exception:
            pass
        if self.gripper is not None:
            try:
                self.gripper.emergency_stop()
            except Exception:
                pass
        self.state = LifecycleState.FAULT

    def disconnect(self) -> None:
        if self.state is LifecycleState.DISCONNECTED:
            return
        self._authorization = None
        errors: list[Exception] = []
        try:
            self._disarm_motion()
        except Exception as error:
            errors.append(error)
        for stream in reversed(tuple(self.streams.values())):
            try:
                stream.disconnect()
            except Exception as error:
                errors.append(error)
        if self.gripper is not None:
            try:
                self.gripper.disconnect()
            except Exception as error:
                errors.append(error)
        try:
            self.backend.disconnect()
        except Exception as error:
            errors.append(error)
        self.state = LifecycleState.DISCONNECTED
        if errors:
            raise RuntimeError(f"disconnect encountered {len(errors)} backend error(s)") from errors[0]


class LeaderArm:
    """Read-only leader device. Failed reads never return cached or zero commands."""

    def __init__(
        self,
        config: LeaderConfig,
        backend: LeaderArmBackend,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self.backend = backend
        self.clock = clock
        self.state = LifecycleState.DISCONNECTED
        self.fault_reason: str | None = None

    def connect_read_only(self) -> None:
        if self.state is not LifecycleState.DISCONNECTED:
            raise RuntimeError(f"cannot connect from state {self.state.value}")
        try:
            self.backend.connect(self.config.endpoint)
            self.state = LifecycleState.READ_ONLY
            self.read_state()
        except Exception as error:
            try:
                self.backend.disconnect()
            except Exception:
                pass
            self.fault_reason = f"leader probe failed: {error}"
            self.state = LifecycleState.FAULT
            raise

    def read_state(self) -> JointSample:
        if self.state is not LifecycleState.READ_ONLY:
            raise RuntimeError(f"cannot read leader from state {self.state.value}")
        try:
            sample = self.backend.read_joint_state()
            if sample.unit is not self.config.joint_unit:
                raise SafetyViolation("leader backend unit does not match config")
            if len(sample.positions) != len(self.config.joint_names):
                raise SafetyViolation("leader backend joint order does not match config")
            _assert_fresh(
                sample.monotonic_timestamp,
                self.config.max_state_age_s,
                self.clock(),
                "leader state",
            )
            return sample
        except Exception as error:
            self.fault_reason = f"leader read failed: {error}"
            self.state = LifecycleState.FAULT
            raise

    def disconnect(self) -> None:
        if self.state is LifecycleState.DISCONNECTED:
            return
        self.backend.disconnect()
        self.state = LifecycleState.DISCONNECTED


class BimanualRig:
    """Two-arm coordinator without an implicit left/right observation layout."""

    def __init__(
        self,
        config: BimanualConfig,
        left: FollowerArm,
        right: FollowerArm,
        *,
        leaders: Mapping[Side, LeaderArm] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if left.config != config.left or right.config != config.right:
            raise ValueError("follower configs do not match bimanual config")
        self.config = config
        self.arms = {Side.LEFT: left, Side.RIGHT: right}
        self.leaders = dict(leaders or {})
        if any(side not in {Side.LEFT, Side.RIGHT} for side in self.leaders):
            raise ValueError("leader map may contain only explicit left/right keys")
        if any(leader.config.side is not side for side, leader in self.leaders.items()):
            raise ValueError("leader map key must match each leader config side")
        self.clock = clock
        self._authorization: MotionAuthorization | None = None

    @property
    def state(self) -> LifecycleState:
        states = {arm.state for arm in self.arms.values()} | {
            leader.state for leader in self.leaders.values()
        }
        if LifecycleState.FAULT in states:
            return LifecycleState.FAULT
        if states == {LifecycleState.READ_ONLY}:
            return LifecycleState.READ_ONLY
        if states == {LifecycleState.DISCONNECTED}:
            return LifecycleState.DISCONNECTED
        return LifecycleState.FAULT

    def connect_read_only(self) -> None:
        connected: list[FollowerArm | LeaderArm] = []
        try:
            for side in (Side.LEFT, Side.RIGHT):
                self.arms[side].connect_read_only()
                connected.append(self.arms[side])
            for side in (Side.LEFT, Side.RIGHT):
                if side in self.leaders:
                    self.leaders[side].connect_read_only()
                    connected.append(self.leaders[side])
        except Exception:
            for device in reversed(connected):
                try:
                    device.disconnect()
                except Exception:
                    pass
            raise

    def observe(self) -> BimanualObservation:
        return BimanualObservation(
            arms={side: self.arms[side].observe() for side in (Side.LEFT, Side.RIGHT)}
        )

    def read_leaders(self) -> dict[Side, JointSample]:
        return {side: leader.read_state() for side, leader in self.leaders.items()}

    def authorize_next_motion(
        self,
        targets: Mapping[Side, MotionTarget],
        *,
        reason: str,
        acknowledgement: str,
    ) -> MotionAuthorization:
        if self.state is not LifecycleState.READ_ONLY:
            raise MotionAuthorizationError(f"cannot authorize from state {self.state.value}")
        if set(targets) != {Side.LEFT, Side.RIGHT}:
            raise MotionAuthorizationError("bimanual target must contain left and right exactly once")
        if acknowledgement != self.config.acknowledgement:
            raise MotionAuthorizationError("bimanual acknowledgement does not match robot identities")
        if not reason.strip():
            raise MotionAuthorizationError("motion reason must not be empty")
        for side in (Side.LEFT, Side.RIGHT):
            self.arms[side]._prepare_target(targets[side])
        payload = {side.value: _target_payload(targets[side]) for side in (Side.LEFT, Side.RIGHT)}
        ttl = min(arm.config.authorization_ttl_s for arm in self.arms.values())
        authorization = MotionAuthorization(
            token=secrets.token_urlsafe(32),
            target_digest=_digest(payload),
            expires_at=self.clock() + ttl,
            scope=(
                f"left:{self.config.left.robot_serial}+right:{self.config.right.robot_serial}:"
                f"{reason.strip()}"
            ),
        )
        self._authorization = authorization
        return authorization

    def execute_authorized(
        self,
        targets: Mapping[Side, MotionTarget],
        authorization: MotionAuthorization,
    ) -> None:
        expected = self._authorization
        self._authorization = None
        payload = {side.value: _target_payload(targets[side]) for side in (Side.LEFT, Side.RIGHT)}
        if expected is None or authorization != expected:
            raise MotionAuthorizationError("unknown, superseded, or already-consumed authorization")
        if self.clock() > authorization.expires_at:
            raise MotionAuthorizationError("bimanual authorization expired")
        if authorization.target_digest != _digest(payload):
            raise MotionAuthorizationError("bimanual target differs from authorized target")

        prepared = {
            side: self.arms[side]._prepare_target(targets[side])
            for side in (Side.LEFT, Side.RIGHT)
        }
        error: Exception | None = None
        try:
            for side in (Side.LEFT, Side.RIGHT):
                self.arms[side]._arm_motion()
            for side in (Side.LEFT, Side.RIGHT):
                self.arms[side]._write_raw(*prepared[side])
        except Exception as caught:
            error = caught
        for side in (Side.RIGHT, Side.LEFT):
            try:
                self.arms[side]._disarm_motion()
            except Exception as caught:
                error = error or caught
        if error is not None:
            self.emergency_stop(f"bimanual one-shot motion failed: {error}")
            raise error

    def emergency_stop(self, reason: str) -> None:
        self._authorization = None
        for arm in self.arms.values():
            arm.emergency_stop(reason)

    def disconnect(self) -> None:
        self._authorization = None
        errors: list[Exception] = []
        for side in (Side.RIGHT, Side.LEFT):
            if side in self.leaders:
                try:
                    self.leaders[side].disconnect()
                except Exception as error:
                    errors.append(error)
        for side in (Side.RIGHT, Side.LEFT):
            try:
                self.arms[side].disconnect()
            except Exception as error:
                errors.append(error)
        if errors:
            raise RuntimeError(f"bimanual disconnect encountered {len(errors)} error(s)") from errors[0]
