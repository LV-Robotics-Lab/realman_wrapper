from __future__ import annotations

import json
import math
import os
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .config import RM75_DOF, ArmConfig
from .model import JointUnit, Side

CALIBRATION_SCHEMA = "lv_robotics.realman_arm_calibration.v1"


@dataclass(frozen=True)
class CalibrationProvenance:
    source: str
    source_revision: str
    source_path: str
    recorded_at: str
    operator: str
    method: str

    def __post_init__(self) -> None:
        for field_name in (
            "source",
            "source_revision",
            "source_path",
            "recorded_at",
            "operator",
            "method",
        ):
            if not getattr(self, field_name).strip():
                raise ValueError(f"calibration provenance {field_name} must not be empty")


@dataclass(frozen=True)
class GripperCalibration:
    raw_open: float
    raw_closed: float
    canonical_open: float = 0.0
    canonical_closed: float = 1.0
    unit: str = "backend_raw"

    def __post_init__(self) -> None:
        object.__setattr__(self, "unit", self.unit.strip())
        values = (self.raw_open, self.raw_closed, self.canonical_open, self.canonical_closed)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("gripper calibration values must be finite")
        if self.raw_open == self.raw_closed:
            raise ValueError("raw gripper endpoints must differ")
        if self.canonical_open == self.canonical_closed:
            raise ValueError("canonical gripper endpoints must differ")
        if not self.unit:
            raise ValueError("gripper calibration unit must not be empty")

    def to_canonical(self, raw: float) -> float:
        fraction = (raw - self.raw_open) / (self.raw_closed - self.raw_open)
        return self.canonical_open + fraction * (self.canonical_closed - self.canonical_open)

    def to_raw(self, canonical: float) -> float:
        fraction = (canonical - self.canonical_open) / (
            self.canonical_closed - self.canonical_open
        )
        return self.raw_open + fraction * (self.raw_closed - self.raw_open)

    def contains(self, canonical: float) -> bool:
        lower = min(self.canonical_open, self.canonical_closed)
        upper = max(self.canonical_open, self.canonical_closed)
        return lower <= canonical <= upper


@dataclass(frozen=True)
class ArmCalibration:
    side: Side
    robot_serial: str
    joint_names: tuple[str, ...]
    joint_unit: JointUnit
    offsets: tuple[float, ...]
    directions: tuple[int, ...]
    hard_min: tuple[float, ...]
    hard_max: tuple[float, ...]
    provenance: CalibrationProvenance
    gripper: GripperCalibration | None = None
    schema: str = CALIBRATION_SCHEMA

    def __post_init__(self) -> None:
        object.__setattr__(self, "side", Side(self.side))
        object.__setattr__(self, "robot_serial", self.robot_serial.strip())
        object.__setattr__(self, "joint_names", tuple(str(name) for name in self.joint_names))
        object.__setattr__(self, "joint_unit", JointUnit(self.joint_unit))
        object.__setattr__(self, "offsets", tuple(float(value) for value in self.offsets))
        object.__setattr__(self, "directions", tuple(int(value) for value in self.directions))
        object.__setattr__(self, "hard_min", tuple(float(value) for value in self.hard_min))
        object.__setattr__(self, "hard_max", tuple(float(value) for value in self.hard_max))
        if self.schema != CALIBRATION_SCHEMA:
            raise ValueError(f"unsupported calibration schema: {self.schema}")
        if not self.robot_serial:
            raise ValueError("robot_serial must not be empty")
        vectors = (self.joint_names, self.offsets, self.directions, self.hard_min, self.hard_max)
        if any(len(values) != RM75_DOF for values in vectors):
            raise ValueError(f"all joint calibration vectors must contain {RM75_DOF} values")
        if len(set(self.joint_names)) != RM75_DOF:
            raise ValueError("joint_names must be unique")
        if any(direction not in {-1, 1} for direction in self.directions):
            raise ValueError("directions must contain only -1 or 1")
        numeric = self.offsets + self.hard_min + self.hard_max
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("joint calibration values must be finite")
        if any(lower >= upper for lower, upper in zip(self.hard_min, self.hard_max, strict=True)):
            raise ValueError("every hard_min value must be below hard_max")

    def assert_compatible(self, config: ArmConfig) -> None:
        expected = (config.side, config.robot_serial, config.joint_names, config.joint_unit)
        actual = (self.side, self.robot_serial, self.joint_names, self.joint_unit)
        if actual != expected:
            raise ValueError(
                "calibration identity does not match arm config "
                f"(calibration={actual!r}, config={expected!r})"
            )

    def to_canonical(self, raw_positions: tuple[float, ...]) -> tuple[float, ...]:
        if len(raw_positions) != RM75_DOF:
            raise ValueError(f"raw_positions must contain {RM75_DOF} values")
        return tuple(
            direction * float(raw) + offset
            for raw, offset, direction in zip(
                raw_positions, self.offsets, self.directions, strict=True
            )
        )

    def to_raw(self, canonical_positions: tuple[float, ...]) -> tuple[float, ...]:
        if len(canonical_positions) != RM75_DOF:
            raise ValueError(f"canonical_positions must contain {RM75_DOF} values")
        return tuple(
            direction * (float(value) - offset)
            for value, offset, direction in zip(
                canonical_positions, self.offsets, self.directions, strict=True
            )
        )

    def contains(self, canonical_positions: tuple[float, ...]) -> bool:
        if len(canonical_positions) != RM75_DOF:
            return False
        return all(
            lower <= value <= upper
            for value, lower, upper in zip(
                canonical_positions, self.hard_min, self.hard_max, strict=True
            )
        )

    @classmethod
    def load(cls, path: Path) -> ArmCalibration:
        payload: dict[str, Any] = json.loads(Path(path).read_text())
        provenance = CalibrationProvenance(**payload["provenance"])
        gripper_payload = payload.get("gripper")
        return cls(
            schema=str(payload.get("schema", "")),
            side=Side(payload["side"]),
            robot_serial=str(payload["robot_serial"]),
            joint_names=tuple(str(value) for value in payload["joint_names"]),
            joint_unit=JointUnit(payload["joint_unit"]),
            offsets=tuple(float(value) for value in payload["offsets"]),
            directions=tuple(int(value) for value in payload["directions"]),
            hard_min=tuple(float(value) for value in payload["hard_min"]),
            hard_max=tuple(float(value) for value in payload["hard_max"]),
            provenance=provenance,
            gripper=GripperCalibration(**gripper_payload) if gripper_payload else None,
        )

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload: dict[str, Any] = {
            "schema": self.schema,
            "side": self.side.value,
            "robot_serial": self.robot_serial,
            "joint_names": list(self.joint_names),
            "joint_unit": self.joint_unit.value,
            "offsets": list(self.offsets),
            "directions": list(self.directions),
            "hard_min": list(self.hard_min),
            "hard_max": list(self.hard_max),
            "provenance": asdict(self.provenance),
        }
        if self.gripper is not None:
            payload["gripper"] = asdict(self.gripper)
        with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            temporary = Path(handle.name)
        os.replace(temporary, path)
