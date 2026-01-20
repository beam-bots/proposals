<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding agents when working with this repository.

## Project Overview

This repository contains feature proposals for the [Beam Bots](https://github.com/beam-bots/bb) robotics framework. Proposals follow a lightweight process: draft, discuss via PR, merge when ready to implement.

## Creating a New Proposal

### 1. Determine the Next Proposal Number

**IMPORTANT:** Before creating a proposal, you MUST check both merged and open PRs to find the next available number:

```bash
gh pr list --repo beam-bots/proposals --state all --json number,title,state
```

The next proposal number is one higher than the highest existing proposal number (including open PRs, not just merged ones).

### 2. Create the Proposal File

1. Copy `template.md` to `accepted/NNNN-feature-name.md`
2. Add the SPDX license header at the top:
   ```markdown
   <!--
   SPDX-FileCopyrightText: 2026 James Harton

   SPDX-License-Identifier: Apache-2.0
   -->
   ```
3. Fill in all sections of the template
4. Use the current date (check with `date +%Y-%m-%d`)

### 3. Proposal Structure

Proposals should include:

- **Summary** — One paragraph explaining what this is
- **Motivation** — Why are we doing this? What problem does it solve?
- **Design** — How does it work? Core abstractions, modules, code examples
- **Package Structure** — Directory layout and dependencies
- **User Experience** — How users will interact with it
- **Acceptance Criteria** — Must have / Should have / Won't have
- **Open Questions** — Things to figure out during implementation
- **References** — Links to prior art, documentation, related projects

### 4. Style Guidelines

- Follow the style of existing proposals (see `accepted/` directory)
- Include code examples showing typical usage
- Be specific about integration points with existing BB packages
- Clearly scope what's in vs out of scope
- Use tables for comparisons where appropriate

## Common Commands

```bash
# Check REUSE license compliance
pipx run reuse lint

# List all proposals (merged and open)
gh pr list --repo beam-bots/proposals --state all --json number,title,state

# Create a PR for a new proposal
gh pr create --title "Proposal NNNN: feature-name" --body "Description of the proposal"
```

## Directory Structure

```
proposals/
├── accepted/     # Proposals merged and approved for implementation
├── implemented/  # Proposals that have shipped
├── rejected/     # Proposals we decided against (with explanation)
├── template.md   # Proposal template
└── README.md     # Overview and proposal index
```

## REUSE Compliance

All files must have SPDX license headers. For markdown files, use HTML comments at the top of the file. For files that don't support comments (like `.json`), create a companion `.license` file.
