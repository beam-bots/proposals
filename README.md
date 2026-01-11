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

## Numbering

Proposals are numbered sequentially: `0001`, `0002`, etc. Use the next available number when opening a PR.

## Template

See [template.md](template.md) for the proposal format.

## Proposals

| # | Name | Status | Package |
|---|------|--------|---------|
| [0001](https://github.com/beam-bots/proposals/pull/1) | bb_teleop | Proposed | `bb_teleop` |
| [0002](https://github.com/beam-bots/proposals/pull/2) | bb_policy | Proposed | `bb_policy` |
| [0003](https://github.com/beam-bots/proposals/pull/3) | bb_dataset | Proposed | `bb_dataset` |
| [0004](https://github.com/beam-bots/proposals/pull/4) | bb_mcp | Proposed | `bb_mcp` |
| [0005](https://github.com/beam-bots/proposals/pull/5) | bb_tui | Proposed | `bb_tui` |
