from __future__ import annotations

import time
from collections.abc import Callable, Sequence

from .config import NetworkEndpoint, SerialEndpoint
from .model import (
    CartesianPose,
    ForwardKinematicsRequest,
    FrameSample,
    JointSample,
    JointUnit,
    ScalarSample,
)


class FakeFollowerBackend:
    def __init__(
        self,
        positions: Sequence[float],
        *,
        unit: JointUnit = JointUnit.RADIANS,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.positions = tuple(float(value) for value in positions)
        self.unit = unit
        self.clock = clock
        self.connected = False
        self.motion_enabled = False
        self.sequence = 0
        self.writes: list[tuple[float, ...]] = []
        self.emergency_stops = 0
        self.endpoint: NetworkEndpoint | None = None

    def connect(self, endpoint: NetworkEndpoint | None, *, read_only: bool) -> None:
        if not read_only:
            raise RuntimeError("fake follower must initially connect read-only")
        if self.connected:
            raise RuntimeError("fake follower already connected")
        self.endpoint = endpoint
        self.connected = True
        self.motion_enabled = False

    def disconnect(self) -> None:
        self.motion_enabled = False
        self.connected = False

    def read_joint_state(self) -> JointSample:
        if not self.connected:
            raise RuntimeError("fake follower is disconnected")
        self.sequence += 1
        return JointSample(self.positions, self.clock(), self.sequence, self.unit)

    def set_motion_enabled(self, enabled: bool) -> None:
        if not self.connected:
            raise RuntimeError("fake follower is disconnected")
        self.motion_enabled = enabled

    def write_joint_target(self, positions: Sequence[float], unit: JointUnit) -> None:
        if not self.connected or not self.motion_enabled:
            raise RuntimeError("fake follower motion is not enabled")
        if unit is not self.unit:
            raise RuntimeError("fake follower unit mismatch")
        self.positions = tuple(float(value) for value in positions)
        self.writes.append(self.positions)

    def emergency_stop(self) -> None:
        self.emergency_stops += 1
        self.motion_enabled = False


class FakeLeaderBackend:
    def __init__(
        self,
        positions: Sequence[float],
        *,
        unit: JointUnit = JointUnit.RADIANS,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.positions = tuple(float(value) for value in positions)
        self.unit = unit
        self.clock = clock
        self.connected = False
        self.sequence = 0
        self.endpoint: SerialEndpoint | None = None

    def connect(self, endpoint: SerialEndpoint | None) -> None:
        if self.connected:
            raise RuntimeError("fake leader already connected")
        self.endpoint = endpoint
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def read_joint_state(self) -> JointSample:
        if not self.connected:
            raise RuntimeError("fake leader is disconnected")
        self.sequence += 1
        return JointSample(self.positions, self.clock(), self.sequence, self.unit)


class FakeUGripperBackend:
    def __init__(
        self,
        position: float,
        *,
        unit: str = "backend_raw",
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.position = float(position)
        self.unit = unit
        self.clock = clock
        self.connected = False
        self.motion_enabled = False
        self.sequence = 0
        self.writes: list[float] = []
        self.emergency_stops = 0

    def connect(self, endpoint: NetworkEndpoint | None, *, read_only: bool) -> None:
        if not read_only:
            raise RuntimeError("fake gripper must initially connect read-only")
        self.connected = True
        self.motion_enabled = False

    def disconnect(self) -> None:
        self.motion_enabled = False
        self.connected = False

    def read_position(self) -> ScalarSample:
        if not self.connected:
            raise RuntimeError("fake gripper is disconnected")
        self.sequence += 1
        return ScalarSample(self.position, self.clock(), self.sequence, self.unit)

    def set_motion_enabled(self, enabled: bool) -> None:
        if not self.connected:
            raise RuntimeError("fake gripper is disconnected")
        self.motion_enabled = enabled

    def write_position(self, position: float, unit: str) -> None:
        if not self.connected or not self.motion_enabled:
            raise RuntimeError("fake gripper motion is not enabled")
        if unit != self.unit:
            raise RuntimeError("fake gripper unit mismatch")
        self.position = float(position)
        self.writes.append(self.position)

    def emergency_stop(self) -> None:
        self.emergency_stops += 1
        self.motion_enabled = False


class FakeFrameStreamBackend:
    def __init__(
        self,
        payload: object,
        *,
        encoding: str,
        shape: tuple[int, ...],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.payload = payload
        self.encoding = encoding
        self.shape = shape
        self.clock = clock
        self.connected = False
        self.sequence = 0

    def connect(
        self,
        remote_endpoint: NetworkEndpoint | None,
        local_endpoint: NetworkEndpoint | None,
    ) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def read_latest(self) -> FrameSample:
        if not self.connected:
            raise RuntimeError("fake stream is disconnected")
        self.sequence += 1
        return FrameSample(
            payload=self.payload,
            monotonic_timestamp=self.clock(),
            sequence=self.sequence,
            encoding=self.encoding,
            shape=self.shape,
        )


class FakeForwardKinematicsBackend:
    def __init__(self) -> None:
        self.requests: list[ForwardKinematicsRequest] = []

    def forward(
        self,
        positions: Sequence[float],
        request: ForwardKinematicsRequest,
    ) -> CartesianPose:
        if len(tuple(positions)) != len(request.joint_names):
            raise ValueError("FK position count does not match request joint order")
        self.requests.append(request)
        return CartesianPose(
            position_m=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            base_frame=request.base_frame,
            tool_frame=request.tool_frame,
        )
