# Source provenance

This repository is a clean, backend-neutral reimplementation of behavior found
in two LV Robotics Lab code snapshots. It does not copy or vendor any ignored,
private, generated, or third-party SDK directory.

## LeRobot archive

- Repository: `https://github.com/LV-Robotics-Lab/lerobot`
- Primary archive commit:
  `1a0f42d7799d6dbdfb6c5df2ee1c2a53a8a5552e`
- Equivalent initial import tag target:
  `fd0198a9b013b5f2b67e341e12f4ae5ff885c3f2`
- License evidence: root `LICENSE` at both snapshots is Apache License 2.0;
  source files also carry Hugging Face Apache-2.0 headers.

Inspected paths:

- `src/lerobot/robots/realman_ugripper_dual/config_realman_ugripper_dual.py`
- `src/lerobot/robots/realman_ugripper_dual/lingkong_gripper.py`
- `src/lerobot/robots/realman_ugripper_dual/realman_ugripper_dual.py`
- `src/lerobot/robots/realman_ugripper_dual/stream_receivers.py`
- `src/lerobot/teleoperators/bi_realman_ugripper_leader/config_bi_realman_ugripper_leader.py`
- `src/lerobot/teleoperators/bi_realman_ugripper_leader/bi_realman_ugripper_leader.py`
- `tests/robots/test_realman_ugripper_dual.py`
- `tests/teleoperators/test_bi_realman_ugripper_leader.py`

These paths established the historical seven-joint-per-side RM75 identity,
leader/follower split, optional gripper and streams, and the previous combined
joint+gripper observation/action shape. This wrapper keeps those devices
separate and has no implicit flattened policy shape.

## tac-infra snapshot

- Repository: `https://github.com/LV-Robotics-Lab/tac-infra`
- Commit: `29195f45dc2b7e19ce715ac7cf17a79cf6b3dbdf`
- Repository-level license evidence: no root `LICENSE` exists at this commit.
- File-level license evidence: every Python path listed below carries a
  Hugging Face copyright header and an Apache License 2.0 notice.

Inspected paths:

- `deployment/hardware/_sdk_paths.py`
- `deployment/hardware/calibration.py`
- `deployment/hardware/follower_arms/base.py`
- `deployment/hardware/follower_arms/realman_tcp.py`
- `deployment/hardware/grippers/base.py`
- `deployment/hardware/grippers/lingkong.py`
- `deployment/hardware/leader_arms/base.py`
- `deployment/hardware/leader_arms/realman.py`
- `deployment/hardware/tactile_sensors/base.py`
- `deployment/hardware/tactile_sensors/dmrobotics_flux.py`
- `deployment/hardware/wrist_cameras/base.py`
- `deployment/hardware/wrist_cameras/fisheye_grpc.py`
- `deployment/robots/realman_ugripper_dual/config_realman_ugripper_dual.py`
- `deployment/robots/realman_ugripper_dual/realman_ugripper_dual.py`
- `deployment/teleoperators/bi_realman_ugripper_leader/config_bi_realman_ugripper_leader.py`
- `deployment/teleoperators/bi_realman_ugripper_leader/bi_realman_ugripper_leader.py`
- `deployment/teleoperators/realman_rm75b_leader/config_realman_rm75b_leader.py`
- `deployment/teleoperators/realman_rm75b_leader/realman_rm75b_leader.py`

This snapshot established the newer decomposed hardware interfaces, asynchronous
state/stream intent, FK boundary, Lingkong uGripper service, separate fisheye
stream, and separate DM Robotics Flux stream. The source also contained
site-specific SDK discovery and runnable network defaults. Neither is carried
into this repository.

## Reimplementation decisions

- The public package uses only Python standard-library code and Protocol-based
  injected backends.
- No vendor protocol bytes, gRPC stubs, UDP decoder, SDK call sequence, site IP,
  CAN interface, camera intrinsic, measured gripper itinerary, or calibration
  value was copied.
- The new project is licensed Apache-2.0 by LV Robotics Lab. The source map above
  is retained so future adapter work can review both copyright and behavior at
  the exact revisions.
- Training-specific 16D and 20D layouts are intentionally not promoted to the
  hardware API. Side order and FK frames remain explicit at every boundary.

## Offline publisher FK evidence

The isolated tac-infra FK adapter targets the public RealMan `Algo` API at
`RealManRobot/RM_API2@9d75cc995f52095837dddca594531621be18cf7b` and the
byte-matched PyPI publisher artifact `robotic-arm==1.1.6`. Exact artifact and
file hashes, the incomplete standalone-license evidence, and the non-redistribution
decision are recorded in [`OFFLINE_FK.md`](OFFLINE_FK.md). No SDK source or binary
is copied into this repository.
