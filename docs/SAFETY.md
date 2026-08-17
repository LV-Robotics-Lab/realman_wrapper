# Safety contract

This package is an offline-validated software boundary, not a certificate that
an RM75 installation is safe to move.

## Required live gates

Before a vendor backend may be used with a real arm, the deployment owner must
verify and record all of the following for each physical side:

1. robot and leader serial identity;
2. explicit network/serial/CAN routing with no SDK fallback address;
3. controller firmware and vendor SDK revision;
4. joint order, unit, sign, offset, hard limits, and maximum one-shot delta;
5. tool and work frames plus the FK adapter's frame convention;
6. emergency-stop behavior while idle, while accepting a target, and after a
   partial bimanual write;
7. state and sensor timestamp clocks, update rate, and stale-data behavior;
8. gripper endpoints and unit from that exact mechanism, when fitted;
9. operator clearance and a physical motion envelope.

Store calibration in the versioned `lv_robotics.realman_arm_calibration.v1`
format. Do not reuse demonstration values or another arm's calibration.

## Lifecycle

`DISCONNECTED -> READ_ONLY` is the only connection transition. There is no
persistent active state. A motion-capable backend is enabled only inside one
target-bound execution and is asked to disable immediately afterward.

Motion requires:

- a calibration matching side, serial, joint names, and joint unit;
- a fresh current joint sample and fresh optional gripper/stream samples;
- a fresh target within calibrated hard limits;
- a per-axis delta within the site-owned config;
- an exact acknowledgement naming the physical serial;
- a non-empty reason and an unexpired, unconsumed authorization.

Any observation, write, or disable failure fails closed. `disconnect()` also
requests motion-disable before releasing backends.

## Bimanual limitation

The two controller writes are necessarily sequential unless a downstream
vendor adapter supplies an atomic controller-side operation. This wrapper
pre-validates both targets and enables both sides before either write, then
disables both in reverse order. On error it calls both emergency-stop hooks.
The live commissioning record must measure the residual partial-write risk.

## Sensors and accessories

Configured streams are part of the observation freshness gate. Do not configure
a stream whose failure should not stop motion; instead expose it through a
separate diagnostic consumer.

`UGripperBackend`, `FrameStreamBackend`, and the FK interface are protocols, not
implementations. A type name does not establish electrical, protocol, or
calibration compatibility with TacClaw or any other device.
