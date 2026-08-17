from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any


class Side(str, Enum):
    LEFT = "left"
    RIGHT = "right"


class JointUnit(str, Enum):
    RADIANS = "radians"
    DEGREES = "degrees"


class LifecycleState(str, Enum):
    DISCONNECTED = "disconnected"
    READ_ONLY = "read_only"
    FAULT = "fault"


def finite_tuple(values: tuple[float, ...] | list[float], *, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not result or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    return result


@dataclass(frozen=True)
class JointSample:
    positions: tuple[float, ...]
    monotonic_timestamp: float
    sequence: int
    unit: JointUnit

    def __post_init__(self) -> None:
        object.__setattr__(self, "positions", finite_tuple(self.positions, name="positions"))
        object.__setattr__(self, "unit", JointUnit(self.unit))
        if not math.isfinite(self.monotonic_timestamp) or self.monotonic_timestamp < 0:
            raise ValueError("monotonic_timestamp must be finite and non-negative")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")


@dataclass(frozen=True)
class ScalarSample:
    value: float
    monotonic_timestamp: float
    sequence: int
    unit: str

    def __post_init__(self) -> None:
        if not math.isfinite(self.value):
            raise ValueError("value must be finite")
        if not math.isfinite(self.monotonic_timestamp) or self.monotonic_timestamp < 0:
            raise ValueError("monotonic_timestamp must be finite and non-negative")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")
        if not self.unit:
            raise ValueError("unit must not be empty")


@dataclass(frozen=True)
class FrameSample:
    payload: Any
    monotonic_timestamp: float
    sequence: int
    encoding: str
    shape: tuple[int, ...]

    def __post_init__(self) -> None:
        if not math.isfinite(self.monotonic_timestamp) or self.monotonic_timestamp < 0:
            raise ValueError("monotonic_timestamp must be finite and non-negative")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")
        if not self.encoding:
            raise ValueError("encoding must not be empty")
        if not self.shape or any(dimension <= 0 for dimension in self.shape):
            raise ValueError("shape must contain positive dimensions")


@dataclass(frozen=True)
class MotionTarget:
    joint_positions: tuple[float, ...]
    monotonic_timestamp: float
    gripper_position: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "joint_positions",
            finite_tuple(self.joint_positions, name="joint_positions"),
        )
        if not math.isfinite(self.monotonic_timestamp) or self.monotonic_timestamp < 0:
            raise ValueError("monotonic_timestamp must be finite and non-negative")
        if self.gripper_position is not None and not math.isfinite(self.gripper_position):
            raise ValueError("gripper_position must be finite when set")


@dataclass(frozen=True)
class MotionAuthorization:
    token: str
    target_digest: str
    expires_at: float
    scope: str


@dataclass(frozen=True)
class ForwardKinematicsRequest:
    side: Side
    joint_names: tuple[str, ...]
    joint_unit: JointUnit
    base_frame: str
    tool_frame: str

    def __post_init__(self) -> None:
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be non-empty and unique")
        if not self.base_frame or not self.tool_frame:
            raise ValueError("base_frame and tool_frame must not be empty")


@dataclass(frozen=True)
class CartesianPose:
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    base_frame: str
    tool_frame: str

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in self.position_m + self.quaternion_xyzw):
            raise ValueError("pose values must be finite")
        norm = math.sqrt(sum(value * value for value in self.quaternion_xyzw))
        if not 0.999 <= norm <= 1.001:
            raise ValueError("quaternion_xyzw must be normalized")
        if not self.base_frame or not self.tool_frame:
            raise ValueError("base_frame and tool_frame must not be empty")
