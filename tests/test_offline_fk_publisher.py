"""Opt-in test for the exact publisher artifact; never creates a robot handle."""

import os

import pytest

from realman_wrapper.offline_fk import (
    TAC_INFRA_JOINT_NAMES,
    VENDOR_SOURCE_REVISION,
    make_tac_infra_rm75_backend,
)

pytestmark = pytest.mark.skipif(
    os.environ.get("REALMAN_WRAPPER_RUN_PUBLISHER_FK") != "1",
    reason="publisher artifact test is opt-in and isolated in its own CI job",
)


@pytest.mark.parametrize("side", ["left", "right"])
def test_publisher_zero_pose_matches_audited_offline_reference(side):
    backend = make_tac_infra_rm75_backend()
    pose = backend.forward(
        (0.0,) * 7,
        side=side,
        joint_names=TAC_INFRA_JOINT_NAMES[side],
        joint_unit="radians",
    )
    assert pose.position_m == pytest.approx((0.0, 0.0, 0.8504999876022339))
    assert pose.quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert pose.base_frame == f"{side}_base"
    assert pose.tool_frame == f"{side}_flange"
    assert backend.provenance["source_revision"] == VENDOR_SOURCE_REVISION
    assert backend.provenance["hardware_touched"] is False
