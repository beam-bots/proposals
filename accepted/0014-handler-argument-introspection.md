<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal: Handler-Level Argument Introspection

**Status:** Draft
**Author:** James Harton
**Created:** 2026-05-16
**Dependencies:** None (changes to `bb` core)

---

## Summary

Allow command handler modules to advertise their goal-map schema directly via
an optional `c:BB.Command.arguments/0` callback. The `BB.Robot.Runtime`
already coerces goal values to declared types when a command's DSL block
contains `argument` declarations (#TODO: link bb PR). This proposal removes
the requirement that every consumer re-declare the same arguments in their
own robot DSL whenever they reuse a shared command handler.

DSL-level `argument` declarations remain authoritative — they can override or
augment what the handler exposes — so existing robots are unaffected.

---

## Motivation

`BB.Command` handlers can be reused across robots. Examples already on hand:

- `BB.Examples.ArmCommands.{Home, MoveToPose, DemoCircle}` — three demo
  commands shipped via the `bb_examples` repo.
- `BB.Command.Arm` / `BB.Command.Disarm` — the built-in safety commands every
  generated robot picks up.
- Anything a downstream user writes once and uses across two or more arms.

In each case the handler's *goal map shape* is a property of the handler,
not the robot. Today the only way to surface that shape to the dashboard,
or to enable runtime coercion, is to repeat it inside each consumer's robot
DSL:

```elixir
# In MyArm.Robot
command :demo_circle do
  handler BB.Examples.ArmCommands.DemoCircle
  argument :ee_link, :atom, default: :ee_link
  argument :plane, {:in, [:xy, :xz, :yz]}, default: :xz
  argument :radius, :float, default: 0.03
  argument :points, :integer, default: 16
  argument :settle_tolerance_m, :float, default: 5.0e-3
  argument :settle_timeout_ms, :integer, default: 1500
end

# In MyOtherArm.Robot
command :demo_circle do
  handler BB.Examples.ArmCommands.DemoCircle
  argument :ee_link, :atom, default: :ee_link
  argument :plane, {:in, [:xy, :xz, :yz]}, default: :xz
  # ... same six argument declarations ...
end
```

Every consumer pays the same documentation tax. When the handler changes
(adds a flag, retypes a value, tightens a default), every consumer has to
update their DSL to match — or the dashboard form drifts out of sync with
what the handler actually accepts.

We want the handler to be a self-describing thing.

---

## Design

### The callback

Add an optional behaviour callback to `BB.Command`:

```elixir
@callback arguments() :: [BB.Dsl.Command.Argument.t()]
```

The return value is the same struct list that `BB.Dsl.Info.commands/1`
already produces — same shape, same fields, same downstream consumers. The
existing `BB.Dsl.Command.Argument` struct gains no new fields.

A handler that doesn't define `arguments/0` keeps its current behaviour:
arguments are entirely sourced from the DSL.

A handler that defines `arguments/0` advertises a default schema. Example:

```elixir
defmodule BB.Examples.ArmCommands.DemoCircle do
  use BB.Command

  alias BB.Dsl.Command.Argument

  @impl BB.Command
  def arguments do
    [
      %Argument{name: :ee_link, type: :atom, required: true,
        doc: "End-effector link name"},
      %Argument{name: :plane, type: {:in, [:xy, :xz, :yz]}, default: :xz,
        doc: "Plane the circle is traced in"},
      %Argument{name: :radius, type: :float, default: 0.03,
        doc: "Circle radius (metres)"},
      %Argument{name: :points, type: :integer, default: 16,
        doc: "Number of waypoints"},
      %Argument{name: :exclude_joints, type: {:list, :atom}, default: [],
        doc: "Joints to hold in place"},
      %Argument{name: :settle_tolerance_m, type: :float, default: 5.0e-3},
      %Argument{name: :settle_timeout_ms, type: :integer, default: 1500}
    ]
  end

  # ... handle_command, result, etc.
end
```

Consumers reduce to:

```elixir
command :demo_circle do
  handler BB.Examples.ArmCommands.DemoCircle
  allowed_states [:idle]
end
```

…and `BB.Dsl.Info.commands/1` reports a `:demo_circle` command whose
`arguments` list comes from the handler.

### Merging precedence

When a command has both handler-advertised arguments *and* DSL-declared
arguments, the DSL wins on a per-key basis. This lets a consumer:

- **Override a default**: handler declares `default: :ee_link`, consumer
  bumps it to `:wrist_link` for a robot with a different topology.
- **Tighten a constraint**: handler declares `type: :float`, consumer
  switches to a narrower enum like `{:in, [0.01, 0.02, 0.05]}`.
- **Hide an argument**: this is *not* covered by the merge — the
  expectation is that consumers add a wrapping handler if they want a
  different surface area.

Concrete merge rule: `BB.Dsl.Info.commands/1` returns the union of
`handler.arguments()` and the DSL-declared arguments, deduplicated by
`name`, with the DSL entry winning on collision.

### Updated runtime contract

`BB.Robot.Runtime.handle_execute_command/4` already calls
`coerce_goal(goal, command.arguments)` (added in the bb runtime-coercion
change). The change here is upstream of that — `command.arguments` becomes
the merged list, not just the DSL-declared one. The coercion code path is
unchanged.

### `BB.Dsl.Info.commands/1` API

`Info.commands/1` becomes the single source of truth that handlers
advertise + DSL declarations have been merged. Nothing that already
consumes `Info.commands/1` (the LiveView dashboard, `bb.to_urdf`,
introspection-driven CLIs) needs to change.

---

## Acceptance Criteria

### Must Have

- [ ] Optional `c:BB.Command.arguments/0` callback declared on the
      `BB.Command` behaviour.
- [ ] `BB.Dsl.Info.commands/1` merges handler-advertised and DSL-declared
      arguments, DSL wins on conflict.
- [ ] No existing consumer needs to change. Handlers without
      `arguments/0` keep their current behaviour.
- [ ] Tests cover: handler-only, DSL-only, both with no conflict, both
      with conflict (DSL precedence), invalid handler return value.

### Should Have

- [ ] `bb_examples`'s three arm commands (`Home`, `MoveToPose`,
      `DemoCircle`) implement `arguments/0`.
- [ ] Documentation update in the BB tutorials covering "writing a
      reusable command handler".

### Won't Have

- A way for the handler to *require* DSL overrides (e.g. "consumer MUST
  set `ee_link` in their DSL"). The merge is permissive; if defaults
  exist, they apply.
- Goal-map validation against the merged argument list. The runtime
  coerces types but doesn't reject unknown keys or missing required
  arguments. That's a separate proposal.
- Argument metadata beyond the existing `BB.Dsl.Command.Argument`
  struct. If we want widget hints, units, or grouping for the dashboard,
  that's a downstream change on the struct, not on this introspection
  mechanism.

---

## Open Questions

### Validation of `arguments/0` return value

Should `BB.Dsl.Info.commands/1` validate that each entry is actually a
`BB.Dsl.Command.Argument` struct, or accept bare maps with the right
keys? Strict validation gives better error messages; permissive
acceptance is friendlier to test mocks.

**Recommendation:** strict — raise `BB.Error.Invalid` with a clear message
pointing at the offending handler module.

### Caching

`handler.arguments/0` is called every time `Info.commands/1` runs. For a
robot with 20 commands, that's 20 module calls per dashboard render. Is
that fine, or do we cache?

**Recommendation:** don't cache yet. The data is small, the function is
pure, and Erlang's module dispatch is cheap. Profile first if it shows
up as a hotspot.

### `BB.Command.Arm` / `BB.Command.Disarm`

The two built-in safety commands don't currently take goal arguments. Do
we add an empty `arguments/0` to them as a documentation gesture, or
leave it absent?

**Recommendation:** leave absent. An empty list and a missing callback
are semantically identical; redundant noise.

---

## References

- `bb` PR: runtime coercion of goal values via declared argument types
  (precondition for this proposal — already merged / in flight)
- `bb_examples` — first set of cross-robot handlers that motivate this
  change
- `BB.Dsl.Command.Argument` struct: `bb/lib/bb/dsl/command/argument.ex`
- Dashboard command renderer:
  `bb_liveview/lib/bb/live_view/components/command.ex`
