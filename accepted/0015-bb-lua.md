<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal: bb_lua

**Status:** Draft
**Author:** James Harton
**Created:** 2026-05-22
**Dependencies:** `bb`, `{:lua, "~> 0.3"}` (which wraps `:luerl`)

---

## Summary

`bb_lua` projects a Beam Bots robot into a sandboxed Lua 5.3 interpreter so that operator scripts, mission files, and live-editing surfaces (Livebook, on-device REPLs) can drive robots from Lua without giving up BB's safety, command, and event semantics. It follows the design of [`ash_lua`](https://hex.pm/packages/ash_lua) — a thin Spark-driven layer over the `lua`/`:luerl` stack that lets the host stay the sole source of authority over identity, safety, and side effects.

---

## Motivation

### Why expose BB to a scripting language at all?

BB is a compile-time DSL framework. That is the right default for the safety-critical, hot-path code — kinematics, controllers, safety. But there are robotics workflows where compile-time is the wrong loop:

| Scenario | Problem with Elixir-only |
|----------|--------------------------|
| Mission files, demo sequences | Each tweak is a recompile or a release |
| Operator-authored macros | Operators aren't Elixir devs; recompile-on-save is a non-starter |
| Lab Livebooks driving real hardware | Mixing notebook-state and OTP-state gets messy fast |
| On-device "behaviour packs" | Bundling Beam releases per behaviour is overkill |
| Education / kit robots | Lua is a far gentler first language than Elixir |

These call for a **scripting surface** that is cheap to author, safe to run, and gives up nothing the framework already guarantees.

### Why Lua?

1. **Universally cheap to embed.** Lua 5.3 semantics are tiny; `:luerl` is pure-Erlang so no NIFs or ports are required, which matters for Nerves and other constrained targets.
2. **Sandbox-by-default.** `:luerl` has no `os`, `io`, or `debug` library exposed to scripts unless the host installs it. The host explicitly chooses every callable surface.
3. **First-class numbers and tables.** Lua's table-as-everything matches BB's argument shape (`%{joint: :shoulder, velocity: ~u(0.5 radian_per_second)}`) without ceremony.
4. **Mature precedent.** [`ash_lua`](https://hex.pm/packages/ash_lua) v0.1.1 (Zach Daniel, 2026-05-20) showed how to project a Spark-DSL framework into Lua cleanly: a per-resource extension, manifest-driven docs, host-supplied actor/tenant/context, Go-style `(result, err)` returns. We are not inventing this — we are mirroring it.
5. **AI-friendly target.** The same manifest that drives the bindings drives docs and `eval` actions, giving `bb_mcp`/`bb_jido` a richer scripting tool surface for free.

### Why a separate package?

- `bb` stays free of Lua dependencies for hardware-only deployments.
- The bindings can evolve at their own pace (luerl tracks Lua language updates independently).
- The integration boundary mirrors `bb_jido` and `bb_mcp` — BB stays the substrate; each adapter is a layered consumer.

### Where this sits

```
┌──────────────────────────────────────────────────────────┐
│ Lua script                                               │
│   robot.arm.move_to_pose({ position = {0.1,0,0.2} })     │
│   bb.on("safety.state", function(s) … end)               │
├──────────────────────────────────────────────────────────┤
│ bb_lua                                                   │
│   - Sandboxed Lua VM (per script or pooled)              │
│   - Command projection from BB.Dsl.Info                  │
│   - PubSub bridge → Lua callbacks                        │
│   - Safety-aware dispatch                                │
├──────────────────────────────────────────────────────────┤
│ BB                                                       │
│   commands · pubsub · safety · runtime                   │
└──────────────────────────────────────────────────────────┘
```

The Lua VM only ever calls Elixir; Elixir never calls Lua except by invoking a registered callback in response to a PubSub event.

---

## Design

### Sandbox

All Lua execution goes through the `lua` hex package (v0.3+), which wraps `:luerl`. The host:

1. Builds a fresh `%Lua{}` per script (or reuses a cached one for hot paths).
2. Installs only the BB surface via `Lua.set!/3`. No filesystem, network, OS, debug.
3. Propagates a host-supplied **safety/identity context** that is *not* readable by the script. The script cannot spoof robot identity, force-disarm, or bypass safety.
4. Runs the script synchronously on the caller. Long scripts can be wrapped in `Task.async`/`Task.yield` by the consumer if a timeout is required.

This is the same shape `ash_lua` uses, and the same shape `Lua`'s standard library exposes — we are not subclassing the sandbox, just choosing which callables to register.

### Public API

```elixir
defmodule BB.Lua do
  @doc """
  Evaluate a Lua script against the given robot, returning `{results, vm}`.
  """
  @spec eval!(String.t(), keyword()) :: {[term()], Lua.t()}
  def eval!(script, opts), do: …

  @doc """
  Same as `eval!/2` but returns `{:ok, results} | {:error, reason}`.
  """
  @spec eval(String.t(), keyword()) :: {:ok, [term()]} | {:error, term()}
  def eval(script, opts), do: …

  @doc """
  Build a reusable `%Lua{}` VM with BB bindings installed.
  Cache this if you evaluate many scripts against the same robot.
  """
  @spec new(keyword()) :: Lua.t()
  def new(opts), do: …
end
```

Typical options:

| Option | Type | Default | Notes |
|---|---|---|---|
| `:robot` | `module()` | — (required) | The `BB.Robot` to bind |
| `:safety` | `:require_armed \| :allow_disarmed \| :force` | `:require_armed` | Pre-dispatch safety check |
| `:topics` | `[BB.PubSub.path()]` | `[]` | PubSub topics to forward as Lua callbacks |
| `:bindings` | `(Lua.t() -> Lua.t())` | `& &1` | Host extension hook (mirrors `ash_lua`'s `:lua` option) |
| `:command_timeout` | `pos_integer()` | `30_000` | Default `BB.Command.await/2` timeout |

### Spark extension: `BB.Lua.Robot`

Following `ash_lua`'s pattern, opt-in is declarative on the robot:

```elixir
defmodule MyRobot do
  use BB.Robot,
    extensions: [BB.Lua.Robot]

  lua do
    name "arm"                            # Lua-side root name; default snake_cased module
    expose_commands [:home, :move_to_pose, :open_gripper, :close_gripper]
    expose_parameters :all
    expose_topics [[:state_machine], [:safety, :error]]
  end

  commands do
    command :home, handler: HomeHandler
    command :move_to_pose, handler: PoseHandler, arguments: [
      argument :position, type: :vec3, required: true,
      argument :orientation, type: :quaternion, required: false
    ]
    # …
  end
end
```

If `lua do … end` is omitted entirely, **nothing** is projected — explicit opt-in is required. This matches BB's general philosophy of declarative, compile-time-validated capability surfaces.

### Calling convention

Commands appear as nested Lua tables shaped `<root>.<group?>.<command>`. The `<group?>` segment is reserved for future categorisation (matches `BB.Dsl.Command`'s `:category` field); the v0.1 default is a flat namespace.

Calls take a single table of named arguments. Returns follow `ash_lua`'s **Go-style** convention:

```lua
local result, err = arm.move_to_pose({
  position    = { x = 0.10, y = 0.0, z = 0.20 },
  orientation = { w = 1.0, x = 0.0, y = 0.0, z = 0.0 }
})

if err then
  log("move failed: " .. err.code)
else
  log("ok at " .. tostring(result.completed_at))
end

-- or, for "raise on error":
local result = assert(arm.move_to_pose({ … }))
```

The error table has a stable shape:

```lua
{ code = "safety_disarmed", message = "...", details = {...} }
```

with codes drawn from a closed set: `"safety_disarmed"`, `"safety_not_armed"`, `"command_failed"`, `"timeout"`, `"cancelled"`, `"unknown_command"`, `"state_not_allowed"`, `"invalid_argument"`. These map 1:1 onto the existing taxonomies in `BB.Command.await/2`, `BB.Jido.Action.Command`, and `BB.Safety`.

### Value marshalling

Round-trip rules:

| BB / Elixir | Lua | Notes |
|---|---|---|
| `nil` | `nil` | |
| `true` / `false` | `true` / `false` | |
| integer | integer | Lua 5.3 has a real integer subtype |
| float | number | |
| binary (UTF-8) | string | |
| atom | string | Atoms always cross as strings; the manifest tells the encoder which positions expect atoms on the way in |
| list | sequence table | 1-indexed on the Lua side |
| keyword list / map | hash table | |
| `%BB.Math.Vec3{}` | `{ x=…, y=…, z=… }` | Constructible from either `{x,y,z}` or positional `{1,2,3}` |
| `%BB.Math.Quaternion{}` | `{ w=…, x=…, y=…, z=… }` | WXYZ; matches `BB.Math.Quaternion` |
| `%BB.Math.Transform{}` | `{ position = {…}, orientation = {…} }` | Canonical pose form — matches the `CLAUDE.md` rule |
| `%Localize.Unit{name, value}` | `{ value = …, unit = "meter" }` | Bridging the `~u` sigil |
| `%BB.Message{}` | `{ payload = …, frame_id = …, monotonic_time = …, wall_time = …, node = … }` | Used in PubSub callbacks |
| Anything else | opaque `BB.Lua.Ref` userdata | Round-trippable but not introspectable from Lua |

A `BB.Lua.Encoder` protocol mirrors `Pythonx.Encoder`/`AshLua.Encoder` for custom types.

### PubSub → Lua callbacks

`:luerl` is single-threaded per VM, so events cannot be delivered concurrently. The host pumps them between `Lua.eval!` slices using a dedicated process (analogous to `BB.Jido.PubSubBridge`):

```elixir
defmodule BB.Lua.PubSubBridge do
  use GenServer
  # subscribes to configured topics, queues incoming
  # {:bb, path, %BB.Message{}} for delivery into a Lua callback registered
  # via bb.on(<event_name>, function(msg) ... end).
end
```

From Lua:

```lua
bb.on("state_machine", function(msg)
  -- msg.payload.to, msg.payload.from
  log("state -> " .. msg.payload.to)
end)

bb.on("safety.error", function(msg)
  log("SAFETY ERROR: " .. msg.payload.reason)
end)
```

Event names are the dotted PubSub path (`["safety", "error"]` → `"safety.error"`). High-volume topics (joint state at 100 Hz) are subject to the same throttle/sampling controls `BB.Jido.PubSubBridge` already implements; the option is `throttle_ms: %{"sensor.joint_state" => 50}` at bridge start.

Lua callbacks run **synchronously** on the bridge process, holding the VM. Misbehaving handlers can therefore block event delivery; this is documented, and the bridge has a configurable timeout that aborts a callback and emits a `[:safety, :error]` event so the system can decide what to do.

### Safety integration

Every dispatched command is gated on `BB.Safety.state(robot)` before `apply/3` is called, in exactly the way `BB.Jido.Action.SafetyAware` does. When `:safety` is `:require_armed` (default) and the state is anything other than `:armed`, the call returns `nil, { code = "safety_not_armed", … }`. When a command exits with `{:error, :disarmed}` mid-flight (safety tripped during execution), the error code is `"safety_disarmed"`.

`bb.safety` is the only host-installed surface that can read safety state from Lua:

```lua
bb.safety.state()   -- "armed" | "disarmed" | "disarming" | "error"
bb.safety.armed()   -- boolean
```

Scripts **cannot** call `arm` / `disarm` / `force_disarm` from Lua. Operators arm hardware through the existing BB surfaces (LiveView dashboard, MCP, physical e-stop, etc.). This is a deliberate restriction — letting a script disarm itself defeats the safety story.

### Auto-generated docs surface

`BB.Lua.Docs.generate(MyRobot)` produces a markdown document enumerating every projected command (signature, argument table, allowed states, default timeout) and every subscribable topic, suitable for inclusion in `bb_mcp` and `bb_kino` consoles. This is the direct analogue of `AshLua.Docs` and shares the same downstream consumers: a `list_callables` / `get_docs` pair for AI agents, and inline help in interactive surfaces. Source for the docs is `BB.Dsl.Info.commands/1` — no duplicate manifest.

### Example: a small mission

```lua
local arm = require("arm")  -- the projected robot root

bb.on("safety.error", function(msg)
  print("aborting: " .. msg.payload.reason)
  bb.exit("safety_aborted")  -- terminates this script run
end)

assert(arm.home({}))

for _, pose in ipairs({
  { x = 0.20, y = 0.00, z = 0.20 },
  { x = 0.20, y = 0.10, z = 0.20 },
  { x = 0.20, y = 0.10, z = 0.10 },
  { x = 0.20, y = 0.00, z = 0.10 },
}) do
  local _, err = arm.move_to_pose({ position = pose })
  if err then
    print("step failed: " .. err.code)
    break
  end
end

assert(arm.home({}))
```

### Integration with `bb_jido` and `bb_mcp`

- **bb_jido:** a `BB.Jido.Action.LuaScript` action can run a Lua script as an atomic agent step, with the script seen as a planning primitive. This is symmetric with `BB.Jido.Action.Reactor`.
- **bb_mcp:** an `eval_lua` MCP tool can execute Lua scripts in the same sandbox, giving AI clients a higher-level macro surface than per-command tools. The `BB.Lua.Docs` output feeds the tool's description.

Neither integration is a v0.1 requirement; both are obvious follow-ups and shape the v0.1 API so they remain natural.

---

## Package Structure

```
bb_lua/
├── lib/
│   └── bb/
│       └── lua/
│           ├── docs.ex              # Markdown generation
│           ├── encoder.ex           # Protocol + default impls
│           ├── error.ex             # Closed-set error mapping
│           ├── pubsub_bridge.ex     # PubSub → Lua callback bridge
│           ├── robot.ex             # Spark extension on BB.Robot
│           ├── runtime.ex           # VM construction + binding install
│           └── safety.ex            # Safety-aware dispatch helpers
├── test/
├── mix.exs
├── README.md
└── CHANGELOG.md
```

### Dependencies

```elixir
defp deps do
  [
    {:bb, bb_dep("~> 0.13")},
    {:lua, "~> 0.3"},
    {:spark, "~> 2.2"}
  ]
end
```

`:lua` already depends on `:luerl`; we do not depend on `:luerl` directly so we inherit its version pinning.

---

## User Experience

### Minimal robot exposure

```elixir
defmodule WX200 do
  use BB.Robot, extensions: [BB.Lua.Robot]

  lua do
    expose_commands [:home, :move_to_pose, :open_gripper, :close_gripper]
    expose_topics [[:state_machine]]
  end

  # ...
end
```

### Running a script

```elixir
{:ok, results} =
  BB.Lua.eval(File.read!("missions/pick_demo.lua"),
    robot: WX200,
    topics: [[:state_machine], [:safety, :error]]
  )
```

### Reusing a VM in a control loop

```elixir
vm = BB.Lua.new(robot: WX200)

Enum.reduce(steps, vm, fn step, vm ->
  {_results, vm} = Lua.eval!(vm, step.script)
  vm
end)
```

### Driving from Livebook

`BB.Lua.eval/2` is the only API needed in a Livebook cell; combined with `bb_kino` widgets for safety/joint state, this gives operators a notebook-driven scripting surface without exposing the BEAM shell.

---

## Acceptance Criteria

### Must Have

- [ ] `BB.Lua.Robot` Spark extension with `lua do … end` block (`name`, `expose_commands`, `expose_topics`, `expose_parameters`)
- [ ] `BB.Lua` entry API (`eval/2`, `eval!/2`, `new/1`)
- [ ] Command projection from `BB.Dsl.Info.commands/1` with argument validation
- [ ] Safety gating using `BB.Safety.state/1` before every dispatch (`:require_armed` default)
- [ ] Closed-set error taxonomy aligned with `BB.Command.await/2`
- [ ] `BB.Lua.PubSubBridge` for `bb.on(name, fn)` subscriptions, with topic allow-listing and per-topic throttling
- [ ] Value marshalling for primitives + `Vec3` / `Quaternion` / `Transform` / `Localize.Unit`
- [ ] Tests covering dispatch, safety gating, error mapping, marshalling, and event delivery
- [ ] README + at least one runnable mission example (`examples/missions/`)

### Should Have

- [ ] `BB.Lua.Docs.generate/1` producing markdown for projected callables
- [ ] `BB.Lua.Encoder` protocol for custom Elixir types
- [ ] Parameter projection (`bb.params.get/set`) honouring `BB.Parameter` read/write semantics
- [ ] Telemetry events on script eval start/stop and command dispatch
- [ ] `bb_kino` widget for running Lua scripts against a robot

### Won't Have

- [ ] In-Lua arming / disarming / force-disarming
- [ ] Direct ETS / `BB.Robot.Runtime` access from Lua
- [ ] Coroutine-based concurrency across multiple Lua VMs (single VM per script run)
- [ ] Hot-reloading running scripts (run-to-completion only)
- [ ] LuaJIT or native Lua via NIFs (sandbox boundary would change)

---

## Open Questions

1. **VM pooling.** Most scripts are short, so a fresh VM per run is fine. For hot paths (e.g. an operator macro pad firing many short scripts per second), do we expose an explicit pool (`BB.Lua.Pool`) or document `BB.Lua.new/1` caching?
2. **Argument-type fidelity.** `BB.Dsl.Command.Argument` types include `:vec3`, `:quaternion`, etc. but also custom-type atoms. Do we require types to declare a `BB.Lua.Encoder` impl, or fall back to opaque `BB.Lua.Ref`?
3. **Long-running commands.** `BB.Command.await/2` blocks. Do we expose an explicit `arm.move_to_pose_async({…})` that returns a handle Lua can poll, or only the blocking form in v0.1?
4. **Script-side telemetry.** Should scripts have a `bb.log(...)` and `bb.telemetry(...)` surface? Probably yes — but consciously kept minimal.
5. **Cancellation.** Scripts run inside the caller; cancelling means killing the calling process. Do we need a cooperative `bb.exit(reason)` (sketched in the example) plus a hard timeout option?
6. **Hot config reload.** When the robot's `lua do … end` block changes, do we recompile any cached VMs? v0.1: no — caching is opt-in and the consumer manages it.
7. **`expose_commands :all`** as shorthand. Convenient but undermines the "explicit surface" goal; lean toward keeping opt-in explicit.

---

## References

- [`ash_lua` on Hex](https://hex.pm/packages/ash_lua) — Direct precedent for the Spark-DSL → Lua projection
- [`ash_lua` repo](https://github.com/ash-project/ash_lua) — `lib/runtime.ex`, `lib/encoder.ex`, `lib/docs.ex`
- [`lua` on Hex](https://hex.pm/packages/lua) — High-level Elixir wrapper used by `ash_lua`
- [`:luerl`](https://github.com/rvirding/luerl) — Pure-Erlang Lua 5.3 implementation
- [Beam Bots `bb_mcp`](https://github.com/beam-bots/bb_mcp) — Existing closest analogue (manifest-driven external surface)
- [Beam Bots `bb_jido` proposal](./0009-bb-jido.md) — Source of the PubSubBridge, SafetyAware, and error-taxonomy patterns
- `bb/lib/bb/command.ex`, `bb/lib/bb/safety.ex`, `bb/lib/bb/pub_sub.ex` — Primary BB surfaces this proposal projects
