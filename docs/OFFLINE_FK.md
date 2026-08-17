# Offline RM75 forward kinematics

`realman_wrapper.offline_fk:make_tac_infra_rm75_backend` is the reviewed
`MODULE:FACTORY` entry point for tac-infra dataset conversion. It is deliberately
separate from every follower, leader, network, serial, CAN, and motion path.

## Contract

- Inputs are exactly seven finite joint values in radians.
- `side` is explicit (`left` or `right`) and the names must exactly match
  `left_main_joint1..7` or `right_main_joint1..7` in that order.
- The publisher algorithm receives degrees and is requested to return
  `[x, y, z, qw, qx, qy, qz]` (`flag=0`). The adapter returns metres and a
  normalized `xyzw` quaternion.
- Before every FK call, mounting angle, work frame, and tool frame are reset to
  identity. Results are labelled `<side>_base` to `<side>_flange`.
- Provenance uses schema `lv_robotics.offline_fk_backend.v1`, records the exact
  source revision, and declares `hardware_touched=false`.

No IP address, robot handle, serial path, CAN interface, calibration, IK, or
motion command exists in this module.

## Publisher artifact and license evidence

The adapter code was written against public API documentation and does not copy
the publisher SDK or native library. The optional factory verifies before import:

- source repository: [`RealManRobot/RM_API2`](https://github.com/RealManRobot/RM_API2)
- source revision: `9d75cc995f52095837dddca594531621be18cf7b`
- PyPI distribution: [`robotic-arm==1.1.6`](https://pypi.org/project/robotic-arm/1.1.6/)
- wheel SHA-256:
  `c564f4809836382ce5d58fc6bfc28b0f21f7ebad9ca56038d4da952910cbedd3`
- sdist SHA-256:
  `743db0f64f5e46d130ba096d0bb94b09664fc5deec53c7870d68c2bef47822bb`

At the audited revision, the two Python binding files in the PyPI artifact are
byte-identical to `RM_API2/Python/Robotic_Arm`. PyPI metadata names an
`@realman-robot.com` author identity and declares MIT, but neither the PyPI
archive nor the GitHub root contains a standalone license text. For that reason
this repository does not redistribute the package. A deployment owner must
separately approve and install the exact publisher artifact. Runtime checks
enforce its version, metadata, Python-source hashes, and platform-native-library
hash before any import.

The [official Python API documentation](https://develop.realman-robotics.com/robot/apipython/classes/algo/)
states that `Algo` can be used independently of a robot connection, that FK
input is degrees, and that `flag=0` returns quaternion pose order
`[x,y,z,w,x,y,z]`. The [official algorithm demo](https://develop.realman-robotics.com/en/robot/demo/python/algoInterface/)
also initializes `Algo` without connecting a robot and sets identity
mounting/work/tool frames before FK.

## Installation and use

Install the wrapper normally. Only on an approved offline conversion machine,
install the publisher wheel from its own distribution using the reviewed lock:

```bash
python -m pip install --require-hashes -r requirements/vendor-rm-api2.lock
python tools/convert_joints_to_eepose.py \
  --fk-backend realman_wrapper.offline_fk:make_tac_infra_rm75_backend \
  --root /absolute/path/to/copied-dataset
```

The factory supports the publisher's audited Linux x86, Linux ARM, and Windows
artifacts. It fails closed on macOS and on unknown architectures.

## Validation boundary

CI covers known injected poses, both sides, exact joint order, radians-to-degrees
conversion, quaternion order, identity-frame reset, malformed inputs, malformed
vendor results, source/license/hash checks, and an opt-in publisher-library smoke
test on Linux. No physical RM75 or endpoint was used. The publisher smoke proves
only deterministic offline library execution and locks the audited zero-joint
reference pose `(0, 0, 0.8504999876 m; identity quaternion)`; it is not
calibration or live-robot validation.
