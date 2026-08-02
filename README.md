<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

<img src="https://github.com/beam-bots/bb/blob/main/logos/beam_bots_logo.png?raw=true" alt="Beam Bots Logo" width="250" />

# Beam Bots Proposals

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![REUSE status](https://api.reuse.software/badge/github.com/beam-bots/proposals)](https://api.reuse.software/info/github.com/beam-bots/proposals)

Feature proposals for the [Beam Bots](https://github.com/beam-bots/bb) robotics framework.

## Process

1. **Draft** — Create a proposal using the template
2. **Discuss** — Open a PR, gather feedback
3. **Accept** — Merge when ready to implement
4. **Implement** — Build it
5. **Close** — Update status when done

That's it. No committees, no final comment periods, no bureaucracy.

## Structure

```
proposals/
├── README.md
├── template.md
├── accepted/
│   └── 0001-feature-name.md
├── implemented/
│   └── ...
└── rejected/
    └── ...
```

Proposals start in `accepted/` when merged. Move to `implemented/` when the feature ships. Move to `rejected/` if we change our minds (with a note explaining why).

A proposal is a snapshot of intent at the time it was written, not a
specification the code is held to. Implementations diverge. Where they have,
the proposal carries an **As shipped** section recording what actually landed —
read it before treating anything above it as current. Proposal 0021 is the model
to follow when marking one implemented.

## Statuses

| Status | Meaning |
|---|---|
| **Draft** | Open PR, under discussion. Not merged. |
| **Accepted** | Merged and agreed, in `accepted/`. Not built yet. |
| **Implemented** | Shipped, in `implemented/`. Check its "As shipped" section for divergences. |
| **Rejected** | Not proceeding, in `rejected/`, with the reasoning recorded. |

## Numbering

Proposals are numbered sequentially: `0001`, `0002`, etc. Use the next available number when opening a PR — check open PRs too, not just merged ones.

Numbers are never reused, including by proposals that were withdrawn. `0010`–`0013` were never allocated.

## Template

See [template.md](template.md) for the proposal format.

## Proposals

| # | Name | Status | Package |
|---|------|--------|---------|
| [0001](accepted/0001-bb-teleop.md) | bb_teleop | Accepted | `bb_teleop` |
| [0002](implemented/0002-bb-policy.md) | bb_policy | Implemented | [`bb_policy`](https://github.com/beam-bots/bb_policy) |
| [0003](accepted/0003-bb-dataset.md) | bb_dataset | Accepted | `bb_dataset` |
| [0004](implemented/0004-bb-mcp.md) | bb_mcp | Implemented | [`bb_mcp`](https://hex.pm/packages/bb_mcp) |
| [0005](implemented/0005-bb-tui.md) | bb_tui | Implemented | [`bb_tui`](https://hex.pm/packages/bb_tui) — community-owned |
| [0006](accepted/0006-bb-motion-planning.md) | bb_motion_planning | Accepted | `bb_motion_planning` |
| [0007](implemented/0007-usage-rules.md) | Usage Rules for Beam Bots Packages | Implemented | all packages |
| [0008](https://github.com/beam-bots/proposals/pull/9) | bb_mavlink | Draft | `bb_mavlink` |
| [0009](implemented/0009-bb-jido.md) | bb_jido | Implemented | [`bb_jido`](https://hex.pm/packages/bb_jido) |
| [0014](https://github.com/beam-bots/proposals/pull/14) | Handler-level argument introspection | Draft | `bb` |
| [0015](https://github.com/beam-bots/proposals/pull/18) | bb_lua | Draft | `bb_lua` |
| [0016](https://github.com/beam-bots/proposals/pull/19) | bb_python | Draft | `bb_python` |
| [0017](accepted/0017-bb-perception.md) | bb_perception | Accepted | `bb_perception` |
| [0018](implemented/0018-bb-estimator.md) | BB.Estimator | Implemented | [`bb`](https://hex.pm/packages/bb) / [`bb_estimator_ahrs`](https://hex.pm/packages/bb_estimator_ahrs) |
| [0019](rejected/0019-set-middle-position.md) | Set middle position | Rejected | `feetech` |
| [0020](rejected/0020-bb-rpc.md) | bb_rpc | Rejected | `bb_rpc` |
| [0021](implemented/0021-actuator-command-pipeline.md) | Actuator Command Pipeline | Implemented | [`bb`](https://hex.pm/packages/bb) |

Rows linking to a PR rather than a file are still open for discussion.
