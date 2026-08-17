import math

import pytest

from realman_wrapper.offline_fk import (
    TAC_INFRA_FK_SCHEMA,
    TAC_INFRA_FRAMES,
    TAC_INFRA_JOINT_NAMES,
    VENDOR_SOURCE_REVISION,
    TacInfraRM75OfflineFKBackend,
    _PublisherBindings,
    make_tac_infra_rm75_backend,
)


class FixtureAlgorithm:
    def __init__(self, responses=None):
        self.calls = []
        self.responses = list(
            responses
            or [
                [0.25, -0.5, 0.75, 0.5, 0.5, -0.5, 0.5],
                [-0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
            ]
        )

    def rm_algo_forward_kinematics(self, joint, flag=1):
        self.calls.append((tuple(joint), flag))
        response = self.responses[min(len(self.calls) - 1, len(self.responses) - 1)]
        if isinstance(response, Exception):
            raise response
        return response


def backend(algorithm=None, resets=None):
    fixture = algorithm or FixtureAlgorithm()
    reset_calls = resets if resets is not None else []
    item = TacInfraRM75OfflineFKBackend(
        fixture,
        reset_identity_frames=lambda: reset_calls.append("identity"),
        source_revision="fixture-revision-001",
    )
    return item, fixture, reset_calls


def test_known_left_and_right_poses_preserve_frames_order_and_units():
    item, fixture, reset_calls = backend()
    left_joints = (0.0, math.pi / 2, -math.pi / 2, math.pi, 0.25, -0.5, 1.0)
    left = item.forward(
        left_joints,
        side="left",
        joint_names=TAC_INFRA_JOINT_NAMES["left"],
        joint_unit="radians",
    )
    assert fixture.calls[0][0] == pytest.approx(
        (0.0, 90.0, -90.0, 180.0, 14.3239449, -28.6478898, 57.2957795)
    )
    assert fixture.calls[0][1] == 0
    assert left.position_m == pytest.approx((0.25, -0.5, 0.75))
    assert left.quaternion_xyzw == pytest.approx((0.5, -0.5, 0.5, 0.5))
    assert (left.base_frame, left.tool_frame) == (
        TAC_INFRA_FRAMES["left"].base_frame,
        TAC_INFRA_FRAMES["left"].tool_frame,
    )

    right = item.forward(
        (0.0,) * 7,
        side="right",
        joint_names=TAC_INFRA_JOINT_NAMES["right"],
        joint_unit="radians",
    )
    assert fixture.calls[1] == ((0.0,) * 7, 0)
    assert right.position_m == pytest.approx((-0.1, 0.2, 0.3))
    assert right.quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert (right.base_frame, right.tool_frame) == (
        TAC_INFRA_FRAMES["right"].base_frame,
        TAC_INFRA_FRAMES["right"].tool_frame,
    )
    assert reset_calls == ["identity", "identity"]


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"side": "center"}, "side must"),
        ({"joint_unit": "degrees"}, "only radians"),
        ({"joint_names": TAC_INFRA_JOINT_NAMES["right"]}, "order mismatch"),
        ({"joints": (0.0,) * 6}, "seven-value"),
        ({"joints": (0.0,) * 6 + (float("nan"),)}, "seven-value"),
    ],
)
def test_input_contract_fails_closed(overrides, message):
    item, fixture, reset_calls = backend()
    values = {
        "joints": (0.0,) * 7,
        "side": "left",
        "joint_names": TAC_INFRA_JOINT_NAMES["left"],
        "joint_unit": "radians",
    }
    values.update(overrides)
    with pytest.raises(ValueError, match=message):
        item.forward(**values)
    assert fixture.calls == []
    assert reset_calls == []


@pytest.mark.parametrize(
    ("response", "message"),
    [
        ([0.0] * 6, "seven finite"),
        ([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, float("inf")], "seven finite"),
        ([0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0], "non-unit"),
        (RuntimeError("vendor failure"), "vendor failure"),
    ],
)
def test_vendor_errors_are_never_reinterpreted_as_a_pose(response, message):
    item, _, _ = backend(FixtureAlgorithm([response]))
    with pytest.raises((ValueError, RuntimeError), match=message):
        item.forward(
            (0.0,) * 7,
            side="left",
            joint_names=TAC_INFRA_JOINT_NAMES["left"],
            joint_unit="radians",
        )


def test_identity_frame_reset_failure_stops_before_vendor_call():
    fixture = FixtureAlgorithm()

    def fail_reset():
        raise RuntimeError("identity frame failure")

    item = TacInfraRM75OfflineFKBackend(
        fixture,
        reset_identity_frames=fail_reset,
        source_revision="fixture-revision-001",
    )
    with pytest.raises(RuntimeError, match="identity frame failure"):
        item.forward(
            (0.0,) * 7,
            side="right",
            joint_names=TAC_INFRA_JOINT_NAMES["right"],
            joint_unit="radians",
        )
    assert fixture.calls == []


def test_factory_provenance_is_exact_and_hardware_free(monkeypatch):
    fixture = FixtureAlgorithm()
    resets = []
    monkeypatch.setattr(
        "realman_wrapper.offline_fk._load_publisher_bindings",
        lambda: _PublisherBindings(fixture, lambda: resets.append("identity")),
    )
    item = make_tac_infra_rm75_backend()
    provenance = item.provenance
    assert provenance["schema"] == TAC_INFRA_FK_SCHEMA
    assert provenance["source_revision"] == VENDOR_SOURCE_REVISION
    assert provenance["hardware_touched"] is False
    assert provenance["input_joint_unit"] == "radians"
    assert provenance["joint_order"]["right"] == list(TAC_INFRA_JOINT_NAMES["right"])
    assert provenance["frames"]["left"] == {
        "base_frame": "left_base",
        "tool_frame": "left_flange",
    }
