<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal 0019: Set middle position

**Status:** Rejected
**Author:** Joel Shprentz
**Created:** 2026-05-26
**Rejected:** 2026-06-03 — already solved by `Feetech.write/5`

---

## Summary

Add a function to the `feetech` library to set the middle position of an ST3215
servo: designate the servo's current physical position as step 2048, the centre
of its 0–4095 range.

---

## Motivation

ST3215 servos report and accept positions on a 0–4095 step scale, which the
`feetech` library converts to and from radians. Mounting a servo into a
mechanism rarely leaves its electrical centre aligned with the mechanism's
neutral pose, so the two have to be reconciled somewhere.

The servos support this directly: writing `128` to a servo's `torque_enable`
register (40) makes it recalculate its position offset so the current position
reads as step 2048.

---

## Why it was rejected

The capability already exists. `feetech` exposes the underlying register write,
and `bb_so101`'s calibration task already uses it:

```elixir
Feetech.write(pid, servo_id, :position_offset, offset, await_response: true)
```

See `mix bb_so101.calibrate` in `beam-bots/bb_so101` for a worked example of
centring a servo against a mechanism.

Adding a second, narrower entry point for the same register would give the
library two ways to do one thing, and the general form — write an arbitrary
offset — is strictly more capable than the specific one, which can only ever
centre on the current position.

Nothing in the closing discussion suggested the underlying need was unmet. If
`position_offset` turns out not to cover a real case, that belongs as an issue
on `beam-bots/feetech` rather than as a proposal — this is a single function on
one vendor's servo protocol, well below the threshold where the proposal process
earns its overhead.

---

## References

- [`beam-bots/feetech`](https://github.com/beam-bots/feetech) — the driver
- [`beam-bots/bb_so101`](https://github.com/beam-bots/bb_so101) — `mix bb_so101.calibrate`
