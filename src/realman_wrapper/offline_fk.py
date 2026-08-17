from __future__ import annotations

import hashlib
import importlib
import math
import platform
import sys
import threading
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from importlib import metadata
from pathlib import Path
from typing import Protocol

from .config import RM75_DOF

TAC_INFRA_FK_SCHEMA = "lv_robotics.offline_fk_backend.v1"
VENDOR_DISTRIBUTION = "robotic-arm"
VENDOR_VERSION = "1.1.6"
VENDOR_SOURCE_REPOSITORY = "https://github.com/RealManRobot/RM_API2"
VENDOR_SOURCE_REVISION = "9d75cc995f52095837dddca594531621be18cf7b"
VENDOR_WHEEL_SHA256 = "c564f4809836382ce5d58fc6bfc28b0f21f7ebad9ca56038d4da952910cbedd3"
VENDOR_SDIST_SHA256 = "743db0f64f5e46d130ba096d0bb94b09664fc5deec53c7870d68c2bef47822bb"

_VENDOR_FILE_SHA256 = {
    "Robotic_Arm/rm_robot_interface.py": (
        "3fffdcbc4448b24d8dd9ad0a415988f892ed4daa7e5d5c78b39830b06ba13420"
    ),
    "Robotic_Arm/rm_ctypes_wrap.py": (
        "6d31db234d0f390ec2106d088765245f0705d17c8bfd756e51ff48f69e193ac6"
    ),
    "Robotic_Arm/libs/linux_x86/libapi_c.so": (
        "6a5d96ff8144d04a5058e77ef37bf7da72904e4a336921eb1917c72a3772a036"
    ),
    "Robotic_Arm/libs/linux_arm/libapi_c.so": (
        "5b9d236a5cf901cdf05418d9ef5815a77a8c717af0ff037e7aad9247beb76fb9"
    ),
    "Robotic_Arm/libs/win_32/api_c.dll": (
        "3549051ebecdd0886f75dde42ca1e99d3fc1225c7550312af763bedab4571ab1"
    ),
    "Robotic_Arm/libs/win_64/api_c.dll": (
        "c0c33d16aef9f8a07711c78adf2f664258061a77ec909ecd56a340dd4a97cae7"
    ),
}

TAC_INFRA_JOINT_NAMES = {
    side: tuple(f"{side}_main_joint{index}" for index in range(1, RM75_DOF + 1))
    for side in ("left", "right")
}


class VendorAlgorithm(Protocol):
    """Smallest callable surface used from the separately installed vendor SDK."""

    def rm_algo_forward_kinematics(
        self, joint: list[float], flag: int = 1
    ) -> Sequence[float]: ...


@dataclass(frozen=True)
class FrameConvention:
    base_frame: str
    tool_frame: str

    def __post_init__(self) -> None:
        if not self.base_frame.strip() or not self.tool_frame.strip():
            raise ValueError("base_frame and tool_frame must be explicit and non-empty")


TAC_INFRA_FRAMES = {
    side: FrameConvention(base_frame=f"{side}_base", tool_frame=f"{side}_flange")
    for side in ("left", "right")
}


@dataclass(frozen=True)
class OfflineForwardKinematicsResult:
    """Structural result consumed by tac-infra without importing that project."""

    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    base_frame: str
    tool_frame: str


@dataclass(frozen=True)
class _PublisherBindings:
    algorithm: VendorAlgorithm
    reset_identity_frames: Callable[[], None]


class TacInfraRM75OfflineFKBackend:
    """Offline RM75 FK adapter for tac-infra's explicit ``MODULE:FACTORY`` contract.

    The injected algorithm is never asked to create a robot handle. Every call first resets the
    algorithm installation angle, work frame, and tool frame to identity through the supplied
    callback. Inputs must use the exact per-side joint order and radians. The publisher API is
    called with degrees and quaternion output, which it returns as ``wxyz``; this adapter exposes
    a normalized ``xyzw`` quaternion.
    """

    def __init__(
        self,
        algorithm: VendorAlgorithm,
        *,
        reset_identity_frames: Callable[[], None],
        source_revision: str,
        joint_names_by_side: Mapping[str, Sequence[str]] = TAC_INFRA_JOINT_NAMES,
        frames_by_side: Mapping[str, FrameConvention] = TAC_INFRA_FRAMES,
    ) -> None:
        revision = source_revision.strip()
        if not revision:
            raise ValueError("source_revision must be explicit and non-empty")
        if not callable(reset_identity_frames):
            raise TypeError("reset_identity_frames must be callable")

        self._algorithm = algorithm
        self._reset_identity_frames = reset_identity_frames
        self._source_revision = revision
        self._joint_names = self._validate_joint_names(joint_names_by_side)
        self._frames = self._validate_frames(frames_by_side)
        self._lock = threading.Lock()

    @staticmethod
    def _validate_joint_names(
        value: Mapping[str, Sequence[str]],
    ) -> dict[str, tuple[str, ...]]:
        if set(value) != {"left", "right"}:
            raise ValueError("joint_names_by_side must contain exactly left and right")
        result: dict[str, tuple[str, ...]] = {}
        for side in ("left", "right"):
            names = tuple(str(name) for name in value[side])
            if len(names) != RM75_DOF or len(set(names)) != RM75_DOF:
                raise ValueError(f"{side} joint order must contain seven unique names")
            result[side] = names
        return result

    @staticmethod
    def _validate_frames(
        value: Mapping[str, FrameConvention],
    ) -> dict[str, FrameConvention]:
        if set(value) != {"left", "right"}:
            raise ValueError("frames_by_side must contain exactly left and right")
        result: dict[str, FrameConvention] = {}
        for side in ("left", "right"):
            frame = value[side]
            if not isinstance(frame, FrameConvention):
                raise TypeError(f"{side} frame must be a FrameConvention")
            result[side] = frame
        return result

    @property
    def provenance(self) -> Mapping[str, object]:
        return {
            "schema": TAC_INFRA_FK_SCHEMA,
            "backend_id": "lv_robotics.realman_wrapper.rm75_vendor_algo_fk",
            "source_revision": self._source_revision,
            "hardware_touched": False,
            "robot_model": "RM75",
            "input_joint_unit": "radians",
            "vendor_input_joint_unit": "degrees",
            "vendor_quaternion_order": "wxyz",
            "output_quaternion_order": "xyzw",
            "joint_order": {side: list(names) for side, names in self._joint_names.items()},
            "frames": {
                side: {
                    "base_frame": frame.base_frame,
                    "tool_frame": frame.tool_frame,
                }
                for side, frame in self._frames.items()
            },
        }

    @staticmethod
    def _enum_value(value: object) -> str:
        enum_value = getattr(value, "value", value)
        return enum_value if isinstance(enum_value, str) else str(enum_value)

    def forward(
        self,
        joints: Sequence[float],
        *,
        side: str,
        joint_names: tuple[str, ...],
        joint_unit: str,
    ) -> OfflineForwardKinematicsResult:
        side_value = self._enum_value(side)
        if side_value not in {"left", "right"}:
            raise ValueError(f"side must be 'left' or 'right', got {side_value!r}")
        if self._enum_value(joint_unit) != "radians":
            raise ValueError("RM75 offline FK accepts only radians")

        names = tuple(str(name) for name in joint_names)
        expected_names = self._joint_names[side_value]
        if names != expected_names:
            raise ValueError(
                f"{side_value} joint order mismatch: expected {expected_names}, got {names}"
            )

        try:
            positions = tuple(float(value) for value in joints)
        except (TypeError, ValueError) as error:
            raise ValueError("joints must be a finite seven-value sequence") from error
        if len(positions) != RM75_DOF or not all(math.isfinite(value) for value in positions):
            raise ValueError("joints must be a finite seven-value sequence")
        degrees = [math.degrees(value) for value in positions]

        with self._lock:
            self._reset_identity_frames()
            raw_pose = self._algorithm.rm_algo_forward_kinematics(degrees, flag=0)

        try:
            pose = tuple(float(value) for value in raw_pose)
        except (TypeError, ValueError) as error:
            raise ValueError("vendor FK must return seven finite pose values") from error
        if len(pose) != 7 or not all(math.isfinite(value) for value in pose):
            raise ValueError("vendor FK must return seven finite pose values")

        x, y, z, qw, qx, qy, qz = pose
        norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-3):
            raise ValueError(f"vendor FK returned a non-unit quaternion (norm={norm})")
        quaternion = (qx / norm, qy / norm, qz / norm, qw / norm)
        frame = self._frames[side_value]
        return OfflineForwardKinematicsResult(
            position_m=(x, y, z),
            quaternion_xyzw=quaternion,
            base_frame=frame.base_frame,
            tool_frame=frame.tool_frame,
        )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _native_library_path() -> str:
    machine = platform.machine().lower()
    if sys.platform == "linux":
        if machine in {"x86_64", "amd64"}:
            return "Robotic_Arm/libs/linux_x86/libapi_c.so"
        if machine in {"aarch64", "arm64", "armv7l", "armv8l"}:
            return "Robotic_Arm/libs/linux_arm/libapi_c.so"
    elif sys.platform == "win32":
        return (
            "Robotic_Arm/libs/win_64/api_c.dll"
            if sys.maxsize > 2**32
            else "Robotic_Arm/libs/win_32/api_c.dll"
        )
    raise RuntimeError(
        f"robotic-arm {VENDOR_VERSION} has no audited offline library for "
        f"platform={sys.platform!r}, machine={machine!r}"
    )


def _verify_publisher_distribution() -> None:
    try:
        distribution = metadata.distribution(VENDOR_DISTRIBUTION)
    except metadata.PackageNotFoundError as error:
        raise RuntimeError(
            f"install the separately distributed {VENDOR_DISTRIBUTION}=={VENDOR_VERSION}; "
            "realman_wrapper does not bundle the vendor SDK"
        ) from error
    if distribution.version != VENDOR_VERSION:
        raise RuntimeError(
            f"expected {VENDOR_DISTRIBUTION}=={VENDOR_VERSION}, got {distribution.version}"
        )
    if distribution.metadata.get("License") != "MIT":
        raise RuntimeError("publisher package metadata no longer declares the audited MIT license")

    required_paths = (
        "Robotic_Arm/rm_robot_interface.py",
        "Robotic_Arm/rm_ctypes_wrap.py",
        _native_library_path(),
    )
    for relative_path in required_paths:
        resolved = Path(distribution.locate_file(relative_path)).resolve()
        if not resolved.is_file():
            raise RuntimeError(f"audited vendor file is missing: {relative_path}")
        actual = _sha256(resolved)
        expected = _VENDOR_FILE_SHA256[relative_path]
        if actual != expected:
            raise RuntimeError(
                f"vendor file hash mismatch for {relative_path}: expected {expected}, got {actual}"
            )


def _load_publisher_bindings() -> _PublisherBindings:
    """Load the exact audited publisher artifact without creating a robot handle."""

    _verify_publisher_distribution()
    interface = importlib.import_module("Robotic_Arm.rm_robot_interface")
    ctypes_wrap = importlib.import_module("Robotic_Arm.rm_ctypes_wrap")
    algorithm = interface.Algo(
        ctypes_wrap.rm_robot_arm_model_e.RM_MODEL_RM_75_E,
        ctypes_wrap.rm_force_type_e.RM_MODEL_RM_B_E,
    )

    def reset_identity_frames() -> None:
        identity_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        algorithm.rm_algo_set_angle(0.0, 0.0, 0.0)
        algorithm.rm_algo_set_workframe(ctypes_wrap.rm_frame_t("", identity_pose))
        algorithm.rm_algo_set_toolframe(
            ctypes_wrap.rm_frame_t("", identity_pose, 0.0, 0.0, 0.0, 0.0)
        )

    return _PublisherBindings(
        algorithm=algorithm,
        reset_identity_frames=reset_identity_frames,
    )


def make_tac_infra_rm75_backend() -> TacInfraRM75OfflineFKBackend:
    """Factory for tac-infra's ``realman_wrapper.offline_fk:...`` backend spec.

    Calling this factory is explicit opt-in to load the separately installed publisher package.
    It performs no network, serial, CAN, robot-handle, or motion operation.
    """

    bindings = _load_publisher_bindings()
    return TacInfraRM75OfflineFKBackend(
        bindings.algorithm,
        reset_identity_frames=bindings.reset_identity_frames,
        source_revision=VENDOR_SOURCE_REVISION,
    )
