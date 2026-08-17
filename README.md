# realman_wrapper

`realman_wrapper` is LV Robotics Lab's backend-neutral, fail-closed hardware
boundary for RealMan RM75-family follower and leader devices. It extracts the
hardware contract from the lab's historical LeRobot and tac-infra integrations
without bundling a vendor SDK, an IP address, a serial path, or a calibration.

The package supports one follower arm, a read-only leader, or an explicitly
keyed left/right bimanual rig. Fisheye cameras, DM Robotics Flux tactile
sensors, and Lingkong uGripper devices remain injected interfaces rather than
being folded into the arm driver.

## Validated scope

- The package, fake backends, lifecycle, calibration serialization, FK routing,
  motion gate, and offline doctor are covered by CI.
- No real robot, network endpoint, CAN interface, or serial device was accessed
  while producing this repository.
- A vendor adapter and an operator-reviewed device calibration are still
  required for any hardware use.
- The historical uGripper path used a Lingkong gRPC/CAN service. There is no
  evidence that its command or calibration contract is compatible with
  TacClaw, so this repository makes no such claim.

## Safety contract

- `connect_read_only()` passes `read_only=True` to every motion-capable backend
  and immediately requests motion-disable.
- Missing calibration does not block observation, but it blocks all motion.
- Calibration identity includes side, robot serial, seven-joint order, units,
  hard limits, and source provenance.
- `authorize_next_motion()` requires an exact device-specific acknowledgement,
  checks fresh state and bounded deltas, and returns a target-bound token.
- A token expires quickly, is invalidated by a newer token, and is consumed
  before the write. It cannot be replayed or used for another target.
- A write or disarm failure calls every available emergency-stop hook and moves
  the controller into `FAULT`.
- Bimanual writes cannot be made atomic by a generic Python wrapper. If either
  backend fails after the other accepted a target, both emergency-stop hooks
  run; downstream vendor adapters must document controller-side stop behavior.

See [docs/SAFETY.md](docs/SAFETY.md) before implementing a live adapter.

## Install and offline doctor

```bash
python -m pip install -e '.[test]'
realman-wrapper doctor
```

The default doctor uses a synthetic, read-only fake and reports
`"hardware_touched": false`. To check a deployment manifest without opening
its endpoints:

```bash
realman-wrapper doctor --config /path/to/site-owned-bimanual.json
```

The JSON shape is defined by
[`examples/bimanual-config.schema.json`](examples/bimanual-config.schema.json).
The repository intentionally provides a schema instead of a runnable example:
all addresses, serial paths, joint-step limits, and calibrations must come from
the exact lab installation.

## Minimal fake-backed use

```python
from pathlib import Path

from realman_wrapper import ArmConfig, FakeFollowerBackend, FollowerArm, Side

config = ArmConfig(
    side=Side.LEFT,
    robot_serial="SYNTHETIC-TEST-ONLY",
    calibration_file=Path("/nonexistent/synthetic.json"),
    max_joint_step=(0.01,) * 7,
)
arm = FollowerArm(config, FakeFollowerBackend((0.0,) * 7))
arm.connect_read_only()
print(arm.observe().raw_joint_sample.positions)
arm.disconnect()
```

This example intentionally has no calibration and therefore cannot authorize
motion. Tests show the calibrated, fake-only motion path.

## Integration boundaries

- Implement the protocols in `realman_wrapper.backends` in a separate adapter
  package that owns the legally obtained RealMan, Lingkong, camera, or Flux SDK.
- Do not add SDK directories, `sys.path` discovery, credentials, calibration
  payloads, local routing information, or generated data to this repository.
- Keep policy tensors outside this hardware package. In particular, historical
  `16D` joint+gripper and `20D` EE layouts are application schemas, not wrapper
  defaults. `BimanualObservation.joint_vector()` requires an explicit side
  order for this reason.

See [docs/BACKEND_INTEGRATION.md](docs/BACKEND_INTEGRATION.md) for the adapter
contract and [docs/PROVENANCE.md](docs/PROVENANCE.md) for the exact source map.

## Tests

```bash
python -m pytest -q
ruff check .
```
