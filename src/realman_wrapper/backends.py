from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

from .config import NetworkEndpoint, SerialEndpoint
from .model import (
    CartesianPose,
    ForwardKinematicsRequest,
    FrameSample,
    JointSample,
    JointUnit,
    ScalarSample,
)


class FollowerArmBackend(Protocol):
    """Injected adapter for a vendor RM75 SDK; this package ships no SDK."""

    def connect(self, endpoint: NetworkEndpoint | None, *, read_only: bool) -> None: ...

    def disconnect(self) -> None: ...

    def read_joint_state(self) -> JointSample: ...

    def set_motion_enabled(self, enabled: bool) -> None: ...

    def write_joint_target(self, positions: Sequence[float], unit: JointUnit) -> None: ...

    def emergency_stop(self) -> None: ...


class LeaderArmBackend(Protocol):
    """Read-only leader-arm interface; deliberately has no command method."""

    def connect(self, endpoint: SerialEndpoint | None) -> None: ...

    def disconnect(self) -> None: ...

    def read_joint_state(self) -> JointSample: ...


class UGripperBackend(Protocol):
    """Injected Lingkong uGripper boundary; no TacClaw compatibility is implied."""

    def connect(self, endpoint: NetworkEndpoint | None, *, read_only: bool) -> None: ...

    def disconnect(self) -> None: ...

    def read_position(self) -> ScalarSample: ...

    def set_motion_enabled(self, enabled: bool) -> None: ...

    def write_position(self, position: float, unit: str) -> None: ...

    def emergency_stop(self) -> None: ...


class FrameStreamBackend(Protocol):
    """Read-only injected frame source for fisheye, Flux, or another sensor."""

    def connect(
        self,
        remote_endpoint: NetworkEndpoint | None,
        local_endpoint: NetworkEndpoint | None,
    ) -> None: ...

    def disconnect(self) -> None: ...

    def read_latest(self) -> FrameSample: ...


class FisheyeStreamBackend(FrameStreamBackend, Protocol):
    """Nominal injected boundary for the historical fisheye gRPC/UDP stream."""


class FluxTactileBackend(FrameStreamBackend, Protocol):
    """Nominal injected boundary for a DM Robotics Flux frame stream."""


class ForwardKinematicsBackend(Protocol):
    """Injected FK implementation with explicit side, order, unit, and frames."""

    def forward(
        self,
        positions: Sequence[float],
        request: ForwardKinematicsRequest,
    ) -> CartesianPose: ...
