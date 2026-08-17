from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .model import JointUnit, Side

RM75_DOF = 7
RM75_JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, RM75_DOF + 1))


@dataclass(frozen=True)
class NetworkEndpoint:
    host: str
    port: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "host", self.host.strip())
        if not self.host or self.host in {"0.0.0.0", "::", "*"}:
            raise ValueError("host must name one explicit remote endpoint")
        if not 1 <= self.port <= 65535:
            raise ValueError("port must be in [1, 65535]")


@dataclass(frozen=True)
class SerialEndpoint:
    path: str
    baudrate: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "path", self.path.strip())
        if not self.path:
            raise ValueError("serial path must not be empty")
        if self.baudrate <= 0:
            raise ValueError("baudrate must be positive")


@dataclass(frozen=True)
class StreamConfig:
    name: str
    kind: str
    remote_endpoint: NetworkEndpoint | None = None
    local_endpoint: NetworkEndpoint | None = None
    max_age_s: float = 0.25

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", self.name.strip())
        object.__setattr__(self, "kind", self.kind.strip())
        if not self.name or not self.kind:
            raise ValueError("stream name and kind must not be empty")
        if not math.isfinite(self.max_age_s) or self.max_age_s <= 0:
            raise ValueError("stream max_age_s must be positive")


@dataclass(frozen=True)
class ArmConfig:
    side: Side
    robot_serial: str
    calibration_file: Path
    max_joint_step: tuple[float, ...]
    max_gripper_step: float | None = None
    follower_endpoint: NetworkEndpoint | None = None
    gripper_endpoint: NetworkEndpoint | None = None
    joint_names: tuple[str, ...] = RM75_JOINT_NAMES
    joint_unit: JointUnit = JointUnit.RADIANS
    max_state_age_s: float = 0.25
    max_command_age_s: float = 0.10
    authorization_ttl_s: float = 0.50
    streams: tuple[StreamConfig, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        object.__setattr__(self, "side", Side(self.side))
        object.__setattr__(self, "robot_serial", self.robot_serial.strip())
        object.__setattr__(self, "calibration_file", Path(self.calibration_file))
        object.__setattr__(self, "max_joint_step", tuple(float(v) for v in self.max_joint_step))
        object.__setattr__(self, "joint_names", tuple(str(name) for name in self.joint_names))
        object.__setattr__(self, "joint_unit", JointUnit(self.joint_unit))
        object.__setattr__(self, "streams", tuple(self.streams))
        if not self.robot_serial:
            raise ValueError("robot_serial must not be empty")
        if len(self.joint_names) != RM75_DOF or len(set(self.joint_names)) != RM75_DOF:
            raise ValueError(f"joint_names must contain exactly {RM75_DOF} unique names")
        if len(self.max_joint_step) != RM75_DOF:
            raise ValueError(f"max_joint_step must contain exactly {RM75_DOF} values")
        if any(not math.isfinite(value) or value <= 0 for value in self.max_joint_step):
            raise ValueError("max_joint_step values must be finite and positive")
        if self.max_gripper_step is not None and (
            not math.isfinite(self.max_gripper_step) or self.max_gripper_step <= 0
        ):
            raise ValueError("max_gripper_step must be finite and positive when set")
        for name, value in (
            ("max_state_age_s", self.max_state_age_s),
            ("max_command_age_s", self.max_command_age_s),
            ("authorization_ttl_s", self.authorization_ttl_s),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        stream_names = [stream.name for stream in self.streams]
        if len(set(stream_names)) != len(stream_names):
            raise ValueError("stream names must be unique within one arm")

    @property
    def acknowledgement(self) -> str:
        return f"AUTHORIZE ONE MOTION: {self.side.value}:{self.robot_serial}"


@dataclass(frozen=True)
class LeaderConfig:
    side: Side
    device_serial: str
    endpoint: SerialEndpoint | None = None
    joint_names: tuple[str, ...] = RM75_JOINT_NAMES + ("gripper",)
    joint_unit: JointUnit = JointUnit.RADIANS
    max_state_age_s: float = 0.25

    def __post_init__(self) -> None:
        object.__setattr__(self, "side", Side(self.side))
        object.__setattr__(self, "device_serial", self.device_serial.strip())
        object.__setattr__(self, "joint_names", tuple(str(name) for name in self.joint_names))
        object.__setattr__(self, "joint_unit", JointUnit(self.joint_unit))
        if not self.device_serial:
            raise ValueError("device_serial must not be empty")
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("leader joint_names must be non-empty and unique")
        if not math.isfinite(self.max_state_age_s) or self.max_state_age_s <= 0:
            raise ValueError("max_state_age_s must be finite and positive")


@dataclass(frozen=True)
class BimanualConfig:
    left: ArmConfig
    right: ArmConfig

    def __post_init__(self) -> None:
        if self.left.side is not Side.LEFT or self.right.side is not Side.RIGHT:
            raise ValueError("bimanual config must contain explicit left and right arms")
        if self.left.robot_serial == self.right.robot_serial:
            raise ValueError("left and right robot_serial values must differ")

    @property
    def acknowledgement(self) -> str:
        return (
            "AUTHORIZE ONE BIMANUAL MOTION: "
            f"left:{self.left.robot_serial}+right:{self.right.robot_serial}"
        )


def _network_endpoint(payload: Any) -> NetworkEndpoint | None:
    if payload is None:
        return None
    if not isinstance(payload, dict):
        raise ValueError("network endpoint must be an object or null")
    return NetworkEndpoint(host=str(payload["host"]), port=int(payload["port"]))


def _arm_config(payload: dict[str, Any], expected_side: Side, root: Path) -> ArmConfig:
    side = Side(payload["side"])
    if side is not expected_side:
        raise ValueError(f"expected {expected_side.value} arm, got {side.value}")
    streams = tuple(
        StreamConfig(
            name=str(item["name"]),
            kind=str(item["kind"]),
            remote_endpoint=_network_endpoint(item.get("remote_endpoint")),
            local_endpoint=_network_endpoint(item.get("local_endpoint")),
            max_age_s=float(item.get("max_age_s", 0.25)),
        )
        for item in payload.get("streams", [])
    )
    calibration_file = Path(payload["calibration_file"])
    if not calibration_file.is_absolute():
        calibration_file = root / calibration_file
    return ArmConfig(
        side=side,
        robot_serial=str(payload["robot_serial"]),
        calibration_file=calibration_file,
        max_joint_step=tuple(float(value) for value in payload["max_joint_step"]),
        max_gripper_step=(
            float(payload["max_gripper_step"])
            if payload.get("max_gripper_step") is not None
            else None
        ),
        follower_endpoint=_network_endpoint(payload.get("follower_endpoint")),
        gripper_endpoint=_network_endpoint(payload.get("gripper_endpoint")),
        joint_names=tuple(payload.get("joint_names", RM75_JOINT_NAMES)),
        joint_unit=JointUnit(payload.get("joint_unit", JointUnit.RADIANS.value)),
        max_state_age_s=float(payload.get("max_state_age_s", 0.25)),
        max_command_age_s=float(payload.get("max_command_age_s", 0.10)),
        authorization_ttl_s=float(payload.get("authorization_ttl_s", 0.50)),
        streams=streams,
    )


def load_bimanual_config(path: Path) -> BimanualConfig:
    path = Path(path)
    payload = json.loads(path.read_text())
    if payload.get("schema") != "lv_robotics.realman_bimanual_config.v1":
        raise ValueError("unsupported or missing bimanual config schema")
    return BimanualConfig(
        left=_arm_config(payload["left"], Side.LEFT, path.parent),
        right=_arm_config(payload["right"], Side.RIGHT, path.parent),
    )
