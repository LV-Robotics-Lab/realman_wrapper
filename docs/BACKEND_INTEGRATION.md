# Backend integration contract

The wrapper contains no RealMan, Lingkong, fish-camera, or DM Robotics SDK.
Downstream code injects adapters conforming to the protocols in
`realman_wrapper.backends`.

## RM75 follower

`FollowerArmBackend.connect(endpoint, read_only=True)` must:

- reject `endpoint=None` for a live adapter;
- use exactly that endpoint, with no hard-coded or SDK fallback address;
- create handles without enabling motion;
- return only after a state read can be timestamped in the host monotonic clock.

`read_joint_state()` must name its unit through `JointSample.unit`, preserve the
configured seven-joint order, and never return cached zeros after a read error.
If cached state is returned, retain its original monotonic timestamp so the
wrapper can reject it as stale.

`set_motion_enabled(False)` must prevent new writes. `emergency_stop()` must
document whether it only rejects new commands or also arrests a target already
accepted by the controller. `write_joint_target()` must raise on every nonzero
vendor status code.

## Leader

`LeaderArmBackend` is deliberately read-only. It has no write or torque method.
A short or malformed serial frame must raise; it must not become a zero action or
a reissued last action. Serial framing and checksums belong in the adapter.

## uGripper

`UGripperBackend` represents the Lingkong gRPC/CAN path seen in the historical
integration. The adapter owns SDK discovery and CAN setup. It must reject a
missing explicit endpoint and report a stable unit in every `ScalarSample`.
That unit must exactly match the gripper calibration.

No TacClaw compatibility is asserted. Such an adapter may be added only after
the command range, transport, firmware, homing/calibration procedure, stop
semantics, and returned state have all been matched on authoritative sources and
validated on the intended hardware.

## Fisheye and DM Robotics Flux

Both are instances of `FrameStreamBackend`. The adapter must preserve frame
timestamps and declare `encoding` and `shape`; the core does not assume RGB,
depth/deformation channel order, bit depth, gRPC service, or UDP framing.

The historical implementation used separate gRPC/UDP services for fisheye and
Flux data. Keeping them injected prevents the RM75 arm lifecycle from owning
private SDKs or implicit host routing.

## Forward kinematics

An FK adapter receives a `ForwardKinematicsRequest` containing side, exact joint
order, joint unit, base frame, and tool frame. It must return the requested frame
names and an `xyzw` unit quaternion. The core does not import the RealMan
algorithm library and does not assume a policy's left-first/right-first layout.

The separately reviewed tac-infra adapter is documented in
[`OFFLINE_FK.md`](OFFLINE_FK.md). Its explicit factory verifies and lazily imports
an operator-installed publisher artifact; the package remains absent from the
wrapper's normal dependencies.

## Adapter validation checklist

- unit tests with a fake vendor client and every nonzero status path;
- disconnected, duplicate-connect, stale-state, and malformed-state tests;
- read-only connect proof and motion-disable proof;
- site-config parsing with no fallback endpoints;
- one-shot write, disable, and emergency-stop tests behind a physical clearance
  gate;
- calibration and FK comparison against a separately measured reference;
- bimanual partial-write fault injection.
