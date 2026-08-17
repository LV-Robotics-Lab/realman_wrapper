"""Opt-in test for the exact publisher artifact; never creates a robot handle."""

import json
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
def test_publisher_zero_pose_is_finite_offline_and_records_provenance(side, capsys):
    backend = make_tac_infra_rm75_backend()
    pose = backend.forward(
        (0.0,) * 7,
        side=side,
        joint_names=TAC_INFRA_JOINT_NAMES[side],
        joint_unit="radians",
    )
    print(
        json.dumps(
            {
                "side": side,
                "position_m": pose.position_m,
                "quaternion_xyzw": pose.quaternion_xyzw,
            },
            sort_keys=True,
        )
    )
    assert len(pose.position_m) == 3
    assert len(pose.quaternion_xyzw) == 4
    assert pose.base_frame == f"{side}_base"
    assert pose.tool_frame == f"{side}_flange"
    assert backend.provenance["source_revision"] == VENDOR_SOURCE_REVISION
    assert backend.provenance["hardware_touched"] is False
