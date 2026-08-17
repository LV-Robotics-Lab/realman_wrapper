import json
import sys
from pathlib import Path

import pytest

from realman_wrapper import FakeLeaderBackend, JointSample, JointUnit, LeaderArm, LeaderConfig, Side
from realman_wrapper.cli import main


def test_leader_is_read_only_and_reports_fresh_state() -> None:
    now = [10.0]
    backend = FakeLeaderBackend((0.0,) * 8, clock=lambda: now[0])
    leader = LeaderArm(
        LeaderConfig(side=Side.LEFT, device_serial="LEADER-TEST"),
        backend,
        clock=lambda: now[0],
    )
    leader.connect_read_only()
    assert leader.read_state().positions == (0.0,) * 8
    assert not hasattr(leader, "write_joint_target")


class StaleLeaderBackend(FakeLeaderBackend):
    def read_joint_state(self) -> JointSample:
        return JointSample((0.0,) * 8, 0.0, 1, JointUnit.RADIANS)


def test_stale_leader_state_raises_instead_of_returning_cached_action() -> None:
    backend = StaleLeaderBackend((0.0,) * 8)
    leader = LeaderArm(
        LeaderConfig(side=Side.LEFT, device_serial="LEADER-TEST", max_state_age_s=0.1),
        backend,
        clock=lambda: 10.0,
    )
    with pytest.raises(Exception, match="stale"):
        leader.connect_read_only()


def test_default_doctor_is_synthetic_and_read_only(monkeypatch, capsys) -> None:  # type: ignore[no-untyped-def]
    monkeypatch.setattr(sys, "argv", ["realman-wrapper", "doctor"])
    assert main() == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["hardware_touched"] is False
    assert payload["motion_exercised"] is False
    assert payload["status"] == "ok"


def test_static_doctor_never_opens_endpoint(tmp_path: Path, monkeypatch, capsys) -> None:  # type: ignore[no-untyped-def]
    config = {
        "schema": "lv_robotics.realman_bimanual_config.v1",
        "left": {
            "side": "left",
            "robot_serial": "LEFT-SERIAL",
            "calibration_file": "missing-left.json",
            "max_joint_step": [0.1] * 7,
            "follower_endpoint": {"host": "left.example.invalid", "port": 8080},
        },
        "right": {
            "side": "right",
            "robot_serial": "RIGHT-SERIAL",
            "calibration_file": "missing-right.json",
            "max_joint_step": [0.1] * 7,
            "follower_endpoint": {"host": "right.example.invalid", "port": 8080},
        },
    }
    path = tmp_path / "site.json"
    path.write_text(json.dumps(config))
    monkeypatch.setattr(sys, "argv", ["realman-wrapper", "doctor", "--config", str(path)])
    assert main() == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["hardware_touched"] is False
    assert payload["mode"] == "static-config"
    assert payload["motion_ready_metadata"] is False
