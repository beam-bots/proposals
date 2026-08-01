<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal 0021: Actuator Command Pipeline

**Status:** Implemented
**Author:** James Harton
**Created:** 2026-07-31

Shipped across bb 0.23.0, 0.24.0 and 0.25.0, plus migrations in six packages.

**Ownership and arbitration are deliberately deferred.** Everything else in this
proposal is implemented; the arbiter is not, and won't be until there's a
concrete case for it. The reasoning is in "Ownership: deferred" below — in
short, the robot state machine already serialises commands, so the race is
narrower than it first appeared, and the design questions it raises are better
answered by a real conflict than by speculation. [beam-bots/bb#148] stays open
to track it.

[beam-bots/bb#148]: https://github.com/beam-bots/bb/issues/148

---

## Summary

Give `BB.Actuator` a single, framework-owned inbound command pipeline. Today an actuator driver implements the same command logic three times — once per transport — and the pubsub transport doesn't work at all because nothing subscribes to `[:actuator | path]`. This proposal replaces the three transport callbacks with one `handle_command/2` callback, moves the subscription into `BB.Actuator.Server`, and routes all three public delivery methods (pubsub, direct cast, synchronous call) through one gated path.

Owning that path lets the framework enforce two invariants that are currently either missing or left to driver authors: commands are refused when the robot is disarmed, and commands are refused when another process holds the actuator. Ownership is modelled as an opt-in lease held in a per-robot `BB.Actuator.Arbiter`, monitored so it dies with its owner, and pre-emptible by higher priority. An unleased actuator accepts commands from anyone, so existing single-commander robots need no ceremony.

This is a breaking change to the `BB.Actuator` behaviour affecting five actuator implementations.

---

## Motivation

### The pubsub transport has never worked

`BB.Actuator.set_position/4` and its siblings publish to `[:actuator | path]` (`bb/lib/bb/actuator.ex:368`). `BB.Actuator.Server.init/1` subscribes to parameter-change topics and resolves the transmission, but never subscribes to its own command topic. The receiving half is already built — `server.ex:191` applies `BB.Transmission.apply_to_command/2` and forwards to the driver — so the only missing piece is the subscription.

Nothing else fills the gap. `BB.PubSub.publish/3` dispatches to the published path and its ancestors (`pub_sub.ex:120`), so `bb_servo_feetech` and `bb_servo_robotis`, which subscribe themselves at `[:actuator, joint_name, name]` (`actuator.ex:112` and `:88` respectively), register a path that is neither the published path nor an ancestor of it — a component's real path is `[link, joint, actuator]`. Their subscriptions never match.

The reason this survived is that the only test covering pubsub delivery uses a support module that does the framework's job itself, with the correct path: `bb/test/support/recording_actuator.ex:26`. The test passes while testing the double.

### The blast radius is larger than one function

Four independent consumers publish onto a channel with no subscribers:

| Consumer | Call site | Effect today |
|---|---|---|
| `BB.Motion` | `motion.ex:127`, `:260`, `:354` — `delivery: :pubsub` is the **default** | `move_to/4`, `move_to_multi/3`, `send_positions/3` silently do nothing |
| `BB.Actuator.set_position/4` etc. | `actuator.ex:368` | documented public API, no-op |
| `bb_pid_controller` | `controller.ex:192` publishing to its `output_topic` | PID output never reaches the servo |
| `bb_policy` | `actuator_command.ex:101` | the policy runner's entire actuator effect is a no-op |

The inverse-kinematics API — the subject of tutorial 09 — is a no-op on its default transport.

### Drivers implement commands three times

Every actuator carries the same clause once per transport, all three delegating to one private function:

| Driver | `handle_info` | `handle_cast` | `handle_call` |
|---|---|---|---|
| `BB.Sim.Actuator` | `:77` | `:93` | `:102` |
| `bb_servo_feetech` | `:208`, `:216` | `:229`, `:237` | `:246`, `:255` |
| `bb_servo_robotis` | `:184` | `:193` | `:202` |
| `bb_servo_pca9685` | `:194` | `:200` | `:205` |
| `bb_servo_pigpio` | `:183` | `:189` | `:194` |

Roughly eighteen clauses expressing "a command arrived", differing only in how the framework chose to deliver it. Which transport a caller picked is not information a driver should have to act on.

### Commands are not gated on arm state

`BB.Safety.armed?/1` (`bb/lib/bb/safety.ex:78`) is checked by feetech and robotis. It is **not** checked by `BB.Sim.Actuator`, `bb_servo_pca9685`, or `bb_servo_pigpio`, and nothing in the transport gates it — `BB.Process.cast/3` is a bare `GenServer.cast` (`process.ex:158`). So `BB.Actuator.set_position!(Robot, :pan_servo, 1.0)` on a disarmed robot writes PWM to a real PCA9685 or pigpio servo.

Tutorial 12 shows the check inside the driver's `handle_cast`, which makes a safety invariant contingent on five driver authors remembering it. Three of the five didn't.

### There is no ownership or arbitration

Any process may publish to `[:actuator | path]`. There is no notion of who owns a joint, no priority, and no way to ask who is currently commanding. Today this is masked by the pubsub transport being dead; fixing it makes the race real and reachable through the default path of the primary motion API. `bb_pid_controller`'s documented wiring publishes onto a servo's command topic (`controller.ex:50`) — the same channel `move_to/4` and any command handler use.

### What this proposal does NOT cover

- **DSL-declared command inputs.** `BB.Estimator` declares its inputs in the DSL because it genuinely fuses several heterogeneous sources. An actuator has exactly one command topic, and the thing declared inputs would buy — knowing who may command — is answered better at runtime by ownership. Addable later without breaking anything.
- **Outbound reply routing.** Estimators emit by returning `{:reply, outputs, state}`. Actuators publish outside the callback return — mid-motion, and for read-back hardware from a separate controller process. `BB.Actuator.publish_begin_motion/3` already handles the transmission correctly on the way out and is left alone.
- **Command watchdog / staleness.** "No command for N ms → hold or stop" is a real gap and a natural fit for this pipeline, but it is a separate feature.
- **Cross-node leases.** Local-only, matching the rest of the framework.
- **Changes to the three public delivery methods.** Pubsub, direct and synchronous delivery all remain. Only the callback side unifies.

---

## Design

### The inbound pipeline

All three transports converge on one gated path inside `BB.Actuator.Server`:

```
BB.Actuator.set_position(robot, path, x)      ─┐  publish to [:actuator | path]
BB.Actuator.set_position!(robot, name, x)     ─┼─►  BB.Actuator.Server
BB.Actuator.set_position_sync(robot, name, x) ─┘         │
                                                         ├─ 1. ownership check (lease)
                                                         ├─ 2. arm check (BB.Safety.armed?/1)
                                                         ├─ 3. BB.Transmission.apply_to_command/2
                                                         ▼
                                              MyDriver.handle_command(motor_space_msg, state)
                                                         │
                                                         └─ 4. reply routed per transport
```

Choosing a transport cannot dodge a gate. The gates run in the order above so the more specific rejection wins: a non-owner learns it doesn't own the actuator rather than being told the robot is disarmed.

Nothing bypasses the gates — not even `Command.Stop`.

An earlier draft exempted `Stop`, on the grounds that stopping is the fail-safe direction and a safety supervisor must be able to stop a joint it neither owns nor armed. That was a misreading, and it shipped in 0.23 before being reversed in 0.25. `Stop` documents itself as ceasing an actuator's *motion*, after which it "becomes passive and will not actively resist external forces" — the counterpart to `Command.Hold`, which maintains position. Its `:decelerate` mode settles the question: nothing that slows down smoothly is an emergency stop.

Making hardware safe is `disarm/1`, which is robot-wide and leaves the robot unable to move until re-armed. `BB.Error.Safety.EmergencyStop` is the separate safety concept. Conflating a motion command with a safety mechanism bought two special cases and no safety, and the second was actively harmful: admitting `Stop` regardless of `command_payloads/1` could hand a driver a payload it never declared and has no clause for.

### `handle_command/2`

```elixir
@callback handle_command(command :: BB.Message.t(), state :: term()) ::
            {:reply, reply :: term(), new_state :: term()}
            | {:reply, reply :: term(), new_state :: term(),
               timeout() | :hibernate | {:continue, term()}}
            | {:noreply, new_state :: term()}
            | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
            | {:stop, reason :: term(), new_state :: term()}
```

Required, replacing the transport triple. The message arrives in motor-space, exactly as it does today via `server.ex:191`.

Reply routing by transport:

| Transport | `{:reply, reply, state}` | `{:noreply, state}` |
|---|---|---|
| `*_sync/5` (call) | `reply` sent to the caller | server replies `{:ok, :accepted}` |
| `*!/4` (cast) | discarded | — |
| pubsub | discarded | — |

Drivers keep `handle_info/2`, `handle_cast/2`, `handle_call/3`, `handle_continue/2` and `terminate/2` for their own traffic — bus responses, timers, `bb_servo_feetech`'s `:trajectory_next` — they just stop using them for commands. `init/1`, `disarm/1`, `handle_options/2` and `options_schema/0` are unchanged.

### Framework-owned subscription

`BB.Actuator.Server.init/1` gains one subscription, filtered by payload type:

```elixir
@command_payloads [
  Command.Position, Command.Velocity, Command.Effort,
  Command.Trajectory, Command.Stop, Command.Hold
]

BB.PubSub.subscribe(bb.robot, [:actuator | bb.path], message_types: @command_payloads)
```

The type filter is load-bearing, not tidiness. `[:actuator | path]` is bidirectional: `publish_begin_motion/3` (`actuator.ex:332`) publishes `BeginMotion` to the same topic, so an unfiltered subscription would deliver every actuator its own outbound traffic. `BB.PubSub.subscribe/3` already supports the filter (`pub_sub.ex:83`), and filtering by payload separates inbound commands from outbound events without touching the topic taxonomy.

The catch-all at `server.ex:190` is narrowed to the actuator's own command topic. Today it matches `{:bb, _any_topic, %Message{}}` and applies *this actuator's* transmission to it, so a driver that subscribes to another actuator's command topic — a follower tracking a leader arm — has the command silently rescaled by the wrong transmission. After this change, anything arriving on a topic the driver subscribed to itself is delegated to its `handle_info/2` untouched.

### Declaring the command vocabulary — `command_payloads/1`

A fixed list of six is the right default and the wrong hard limit. `lostbean/bb_mcuhub` — a third-party package in bb's external CI matrix — resolves each port's accepted command struct at runtime from its own port index, via a `command_message/0` seam on its value-types. Every value-type it ships today names a BB built-in, so the fixed list happens to cover it; a value-type naming a struct of its own would not be covered.

The failure mode is what makes this worth fixing rather than documenting. A payload outside the list isn't rejected — it simply isn't delivered, so the driver's recourse is to subscribe to its own command topic. That message then fails the narrowed `handle_info` guard, is delegated straight to the driver, and arrives having skipped the arm gate and (per Phase 2) the ownership check. The extension point exists so that drivers with their own command vocabulary route *through* the pipeline rather than around it, keeping the safety properties this proposal establishes.

```elixir
@callback command_payloads(opts :: keyword()) :: [module()]
```

Optional, defaulted by `use BB.Actuator` to the six built-ins. It takes the resolved options rather than being zero-arity because the set isn't always known at compile time — `bb_mcuhub` derives it from a hub and port named in its opts.

The server calls it after option resolution and uses the result in **both** places: the `message_types` subscription filter and the dispatch guard. Both are required. `message_types` only governs pubsub, so a callback feeding only the subscription would leave a driver that narrows its vocabulary still receiving everything through `set_position!/4`.

One callback therefore serves both directions:

- **Widening** — a driver with a bespoke command struct gets it through the gate instead of around it.
- **Narrowing** — a port that speaks only `Effort` declares `[Command.Effort]`, and the framework enforces it. This subsumes a filter such packages otherwise hand-roll inside `handle_command/2`.

Two alternatives were considered and rejected. A marker behaviour on payload modules (`BB.Message.Command`) is more open — any module declaring itself a command is one — but `BB.PubSub`'s `message_types` is a concrete list matched in `dispatch_to_subscribers/4`, so a predicate means subscribing unfiltered and reintroducing the `BeginMotion` echo the filter exists to prevent. Supporting it properly means extending PubSub with kind-based filtering: a large change to the message system for something one callback solves. A `kind` field on payloads has the same PubSub problem and touches every payload module besides.

This lands with Phase 2 rather than Phase 1. It is the same surface as arbitration — `command_payloads/1` decides what enters the pipeline, ownership decides what survives it — so landing them together means one behaviour change for driver authors instead of two, and lets the lease design assume an open payload set rather than a fixed one.

### Arm gating

The server calls `BB.Safety.armed?/1` before delegating. When disarmed:

- **call** → `{:error, %BB.Error.State.NotArmed{}}`
- **cast / pubsub** → dropped, with telemetry

Drivers delete their own `armed?` checks; the three that never had one are fixed by the framework acquiring the responsibility.

`NotArmed` lives in the `:state` class rather than `:safety` deliberately. Safety-class errors are always `:critical` severity, and a command correctly refused because the robot is disarmed is the system working, not a safety violation. It sits alongside `BB.Error.State.NotAllowed` with severity `:error`.

### Ownership: `BB.Actuator.Arbiter`

One arbiter process per robot, started at the root of the supervision tree alongside the registries (`supervisor.ex:76`), owning a public-read ETS table keyed by actuator name:

```
{actuator_name, owner_pid, priority, monitor_ref, acquired_at}
```

Reads are hot — every inbound command consults the lease, at up to 100 Hz per actuator — so actuator servers read the table directly with `:ets.lookup/2` and never message the arbiter on the command path. Writes are rare and go through the GenServer, which gives atomicity and a place to hang monitors.

This is a single table per robot, not the per-actuator mux process considered and rejected: the actuator server is already the choke point, and a table beats a process hop where a lookup will do. Centralising it also makes joint-level acquisition atomic and makes "who owns what" a single query.

```elixir
@spec acquire(robot :: module(), target :: atom() | [atom()], opts :: keyword()) ::
        :ok | {:error, BB.Error.t()}
@spec release(robot :: module(), target :: atom() | [atom()]) :: :ok
@spec owner(robot :: module(), actuator :: atom()) ::
        {:ok, %{pid: pid(), priority: integer(), since: integer()}} | :none
```

`target` is an actuator name, a joint name, or a list of either. A joint resolves to every actuator beneath it and is acquired all-or-nothing — a joint can carry several actuators (`motion.ex:408` iterates them), and a half-acquired joint is worse than a refused one.

Semantics:

- **No lease** — the actuator accepts commands from anyone. This is today's behaviour, so `move_to/4` from IEx keeps working with no ceremony.
- **Leased** — only the owner's commands are accepted. Others are dropped (`{:error, %BB.Error.State.ActuatorBusy{}}` for the sync transport). Enforcement is against everyone, so a policy runner holding a lease is protected from a stray `move_to/4` — that is the point.
- **Pre-emption** — `acquire/3` with a strictly higher priority succeeds and revokes the incumbent. Equal or lower is refused.
- **Revocation** — the displaced owner is sent `{:bb_lease_revoked, %{robot: module, actuator: atom, by_priority: integer}}`. Silent theft would leave a PID loop integrating against a joint it no longer drives.
- **Owner death** — the monitor fires and the arbiter releases the lease.
- **Disarm** — leases are untouched. Safety and ownership are separate concerns, and the arm gate independently refuses commands; clearing leases on a momentary disarm would silently strand a commander after re-arming.

Priority is a property of the lease, not of the message. A priority field on the payload would be unenforceable — anyone can claim `:critical` — whereas a lease is held by a pid the arbiter monitors.

### Addressing

`BB.Motion` already resolves an actuator's full path correctly in a private helper (`motion.ex:430`), using `BB.Robot.path_to/2` on the actuator's joint. It's the one place in the ecosystem that builds the topic right, and it should be public rather than duplicated:

```elixir
@spec actuator_path(BB.Robot.t(), atom()) :: [atom()] | nil
def actuator_path(%BB.Robot{} = robot, actuator_name)
```

Actuators only ever attach to joints — `BB.LinkSupervisor` builds no actuator children, and `actuator_info` carries a `:joint` — so resolution is total. The pubsub API then accepts either form:

```elixir
BB.Actuator.set_position(Robot, :pan_servo, 0.5)                  # resolved
BB.Actuator.set_position(Robot, [:base, :pan, :pan_servo], 0.5)   # explicit
```

Three independent parties got the path wrong — feetech, robotis, and `usage-rules/actuators.md:15`, which documents `[:pan_joint, :servo]`. When the docs and two of five drivers make the same mistake, the API is the problem.

### Telemetry

Following the existing `[:bb, :<subsystem>, :<event>]` convention (`bb/lib/bb/telemetry.ex`):

| Event | Measurements | Metadata |
|---|---|---|
| `[:bb, :actuator, :command]` | `%{count: 1}` | `%{robot, actuator, transport, payload_module}` |
| `[:bb, :actuator, :rejected]` | `%{count: 1}` | `%{robot, actuator, transport, payload_module, reason: :disarmed \| :not_owner}` |
| `[:bb, :actuator, :lease]` | `%{count: 1}` | `%{robot, actuator, action: :acquired \| :released \| :revoked, owner, priority}` |

A dropped command must be observable. Silent refusal is how this class of bug hid for so long in the first place.

### Error types

Both in the `:state` class, both `severity: :error`:

- `BB.Error.State.NotArmed` — fields `:robot`, `:actuator`, `:command`
- `BB.Error.State.ActuatorBusy` — fields `:robot`, `:actuator`, `:owner`, `:owner_priority`, `:requested_priority`

### What a driver looks like afterwards

`bb_servo_pca9685` today (`actuator.ex:194-209`):

```elixir
@impl BB.Actuator
def handle_info({:bb, _path, %Message{payload: %Command.Position{} = cmd}}, state) do
  {:noreply, do_set_position(cmd, state)}
end

@impl BB.Actuator
def handle_cast({:command, %Message{payload: %Command.Position{} = cmd}}, state) do
  {:noreply, do_set_position(cmd, state)}
end

@impl BB.Actuator
def handle_call({:command, %Message{payload: %Command.Position{} = cmd}}, _from, state) do
  {:reply, {:ok, :accepted}, do_set_position(cmd, state)}
end
```

afterwards:

```elixir
@impl BB.Actuator
def handle_command(%Message{payload: %Command.Position{} = cmd}, state) do
  {:noreply, do_set_position(cmd, state)}
end
```

`bb_servo_feetech`, which handles two payload types across three transports, goes from six clauses to two. Drivers with an `armed?` check drop it as well.

---

## Affected packages and migration

`bb` is at 0.22.2; this lands as 0.23.0.

| Package | Current pin | Change |
|---|---|---|
| `bb` | — | behaviour, server, arbiter, errors, telemetry, addressing, `BB.Sim.Actuator`, tutorial 12, `usage-rules/actuators.md`, `test/support/recording_actuator.ex` |
| `bb_servo_feetech` | `~> 0.21` | six command clauses → two; drop self-subscription (`actuator.ex:112`) and `armed?` checks |
| `bb_servo_robotis` | `~> 0.21` | three → one; drop self-subscription (`actuator.ex:88`) and `armed?` checks |
| `bb_servo_pca9685` | `~> 0.20` | three → one; gains arm gating it never had |
| `bb_servo_pigpio` | `~> 0.18` | three → one; gains arm gating it never had |
| `bb_pid_controller` | `~> 0.16` | none required; output starts arriving. Should acquire a lease |
| `bb_policy` | `~> 0.22` | none required; actuator effects start working. Runner should acquire a lease |

`bb_servo_pigpio` and `bb_pid_controller` are pinned several minors behind and need bumping regardless.

Removing `BB.subscribe/2` from `bb/test/support/recording_actuator.ex:26` is part of the change, not a detail — otherwise the pubsub test keeps exercising the double instead of the framework.

---

## User experience

### Single commander — unchanged

```elixir
MyRobot.start_link(simulation: :kinematic)
BB.Command.arm(MyRobot)

BB.Motion.move_to(MyRobot, :gripper, target)   # default :pubsub — now actually moves
BB.Actuator.set_position(MyRobot, :pan_servo, 0.5)
```

No lease, no ceremony. The only visible difference from today is that it works.

### Disarmed

```elixir
BB.Actuator.set_position!(MyRobot, :pan_servo, 1.0)
# dropped, [:bb, :actuator, :rejected] with reason: :disarmed

BB.Actuator.set_position_sync(MyRobot, :pan_servo, 1.0)
# {:error, %BB.Error.State.NotArmed{actuator: :pan_servo, ...}}

BB.Actuator.stop(MyRobot, :pan_servo)
# always accepted
```

### A policy runner claiming a joint

```elixir
:ok = BB.Actuator.acquire(MyRobot, :shoulder, priority: 10)

BB.Motion.move_to(MyRobot, :gripper, target)     # from IEx — dropped, :not_owner
BB.Actuator.owner(MyRobot, :shoulder_servo)
#=> {:ok, %{pid: #PID<0.412.0>, priority: 10, since: 1234567890}}
```

### Teleop pre-empting an autonomous policy

```elixir
:ok = BB.Actuator.acquire(MyRobot, :shoulder, priority: 100)

# the policy runner receives:
# {:bb_lease_revoked, %{robot: MyRobot, actuator: :shoulder_servo, by_priority: 100}}
```

---

## Acceptance Criteria

### Must Have

#### Phase 1 — inbound pipeline (shipped in bb 0.23.0)

- [x] `BB.Actuator.handle_command/2` defined, required, documented
- [x] `BB.Actuator.Server` subscribes to `[:actuator | bb.path]` filtered to the six `Command.*` payloads
- [x] All three transports route through one gated pipeline and converge on `handle_command/2`
- [x] Reply routed per transport; `{:noreply, state}` yields `{:ok, :accepted}` for the sync transport
- [x] `server.ex:190` narrowed so a driver's own subscriptions reach `handle_info/2` untransformed
- [x] Arm gate enforced centrally; `Command.Stop` exempt, `Command.Hold` not
- [x] `BB.Error.State.NotArmed` with a `BB.Error.Severity` impl
- [x] `BB.Sim.Actuator` migrated; `test/support/recording_actuator.ex` no longer self-subscribes
- [x] Test proving a pubsub command reaches a driver *without* the driver subscribing

#### Phase 2 — the command vocabulary (shipped in bb 0.25.0)

- [x] `command_payloads/1` optional callback, defaulted to the six built-ins
- [x] The callback drives both the `message_types` subscription filter and the dispatch guard, so narrowing holds across all three transports
- [x] Test proving a bespoke command payload reaches `handle_command/2` *and* is refused while disarmed — i.e. enters the pipeline rather than bypassing it
- [x] `BB.Error.State.UnsupportedCommand`, with `:unsupported_command` telemetry
- [x] No exemptions: a command is delivered only if the actuator declared the payload and the robot is armed

#### Ownership — deferred

Not implemented, and not scheduled. [beam-bots/bb#148](https://github.com/beam-bots/bb/issues/148) tracks it.

The design below stands as written and is worth keeping, but two things argued for waiting.

The robot state machine already serialises commands globally — `:idle → :executing → :idle`, one at a time — so two *commands* cannot race. The gap is narrower than the issue suggests: it's between the command system and commanders outside it, which today means a `BB.Controller` running its own loop, a policy runner, or an ad-hoc `set_position!/4`. Real, but narrower, and it changes what the right model is.

That in turn raises a question the design doesn't answer: should an executing command implicitly own what it touches, should leases only bind non-command commanders, or are they simply orthogonal? Each gives different behaviour when a command meets a PID loop, and picking between them from first principles risks building the wrong one. A concrete conflict will answer it in a sentence.

- [ ] `BB.Actuator.Arbiter` per robot, in the root supervision tree, owning a public-read ETS table
- [ ] `acquire/3`, `release/2`, `owner/2`; actuator, joint and list targets; joint acquisition all-or-nothing
- [ ] Unleased actuators accept all commands
- [ ] Higher priority pre-empts; equal or lower refused with `BB.Error.State.ActuatorBusy`
- [ ] Displaced owner receives `{:bb_lease_revoked, …}`
- [ ] Owner death releases the lease via monitor
- [ ] Leases survive disarm and re-arm

#### Phase 3 — addressing and docs (shipped in bb 0.24.0)

- [x] `BB.Robot.actuator_path/2` public; `BB.Motion`'s private copy removed
- [x] Pubsub API accepts an actuator name or a full path
- [x] Tutorial 12 rewritten around `handle_command/2`; inbound diagram corrected
- [x] `usage-rules/actuators.md` corrected — path example and the driver contract
- [x] `documentation/how-to/integrate-servo-driver.md` corrected — it carried the fullest version of the old contract
- [ ] ~~Ownership documented, including the unleased default~~ — deferred with the arbiter
- [x] Every robot definition in bb's own docs fixed, and `mix bb.verify_docs` added to `mix check` — none had ever been compiled, and six didn't

#### Phase 4 — ecosystem

- [x] `bb` 0.23.0 released
- [x] feetech, robotis, pca9685, pigpio migrated, self-subscriptions and `armed?` checks removed
- [x] `bb_ik_fabrik`'s test mock actuator migrated
- [x] `bb_pid_controller` and `bb_policy` verified end to end — neither needed a source change
- [x] `bb` 0.24.0 and 0.25.0 released; all four servo drivers released against them
- [x] pca9685 and pigpio now act on `Command.Stop` rather than swallowing it; [feetech#86](https://github.com/beam-bots/bb_servo_feetech/issues/86) and [robotis#85](https://github.com/beam-bots/bb_servo_robotis/issues/85) track the same for the bus drivers, alongside velocity and effort support
- [ ] Third-party packages notified: `lostbean/bb_mcuhub` ([PR open](https://github.com/lostbean/bb_mcuhub/pull/12)), `mcass19/bb_tui` (unaffected)

### Should Have

- [x] Telemetry for rejections, with a reason
- [ ] The four servo drivers narrow `command_payloads/1` to what they implement
- [ ] ~~`bb_pid_controller` acquires a lease for its output actuator~~ — deferred with the arbiter
- [ ] ~~`bb_policy`'s runner acquires leases for the joints it drives~~ — deferred with the arbiter
- [ ] ~~A named priority scheme over bare integers~~ — deferred with the arbiter

### Won't Have

- [ ] DSL-declared command inputs
- [ ] Outbound `{:reply, outputs, state}` routing from `handle_command/2`
- [ ] Command watchdog / staleness detection
- [ ] Cross-node leases
- [ ] A per-actuator "requires a lease" DSL flag
- [ ] Changes to the three public delivery methods

---

## Open Questions

1. ~~**`handle_command/2` collides in name with `BB.Command.handle_command/3`.**~~ Resolved: kept. The arity and `@impl BB.Actuator` disambiguated cleanly through the whole Phase 1 migration, across six packages, and the ambiguity never bit.

2. **Does `command_payloads/1` need re-evaluating at runtime?** It is called once, after option resolution at `init`. A driver whose accepted payloads change while running — a port reconfigured through the parameter system, say — would need its subscription updated, which means re-subscribing with new `message_types`. No current package needs this; `handle_options/2` is the obvious hook if one does. Worth confirming that once-at-init is enough before implementing.

3. **Arbiter crash loses every lease.** The ETS table dies with its owner, so a crashed arbiter fails open to free-for-all. That's today's behaviour and the arm gate is independent, so it's not a safety regression — but an ETS `heir` or a restart-time rebuild from monitors would be more predictable. Worth deciding whether fail-open is acceptable for ownership state.

4. **Leases surviving disarm.** Decided yes, on the grounds that safety and ownership are orthogonal and clearing them would strand a commander across a momentary disarm. The counter-argument is that a full disarm is a natural "everybody let go" boundary. Confirm before implementing.

5. **Bare integer priorities vs named levels.** Integers are simple and totally ordered; named levels are self-documenting and prevent a priority arms race. Named levels can wrap integers later without breaking callers.

6. **Discarded replies on the cast and pubsub transports.** A driver returning `{:reply, {:error, reason}, state}` to a cast has that reply silently dropped. Emit telemetry, or accept it as the nature of fire-and-forget?

7. **Should `Command.Hold` bypass the arm gate?** Decided no — holding energises hardware. But "hold current position" is arguably closer to fail-safe than to motion, and a driver may implement hold as a passive brake. Confirm.

8. **Whether `bb_pid_controller` and `bb_policy` should acquire leases automatically or leave it to the user.** Automatic is safer and matches intent; explicit is less surprising and avoids a controller monopolising a joint the user wanted to share.

9. **Interaction with `BB.Motion`'s multi-actuator sends.** `send_positions/3` fans out across every actuator of every joint (`motion.ex:394-411`). If one actuator refuses on ownership, the others still move — a partially-applied pose. Should `send_positions/3` pre-check ownership and refuse as a unit?

10. **Does narrowing via `command_payloads/1` warrant an error rather than a drop?** A driver that declares `[Command.Effort]` and is then sent a `Command.Position` currently sees nothing: the subscription filters it and the dispatch guard drops it. That's right for pubsub, where the sender may be addressing a whole subtree, but a `set_position_sync/5` caller arguably deserves `{:error, …}` rather than `{:ok, :accepted}` for a command the actuator structurally cannot accept.

---

## References

- [beam-bots/bb#204](https://github.com/beam-bots/bb/pull/204) — the Phase 1 and 3 implementation, released as bb 0.23.0.
- [lostbean/bb_mcuhub#12](https://github.com/lostbean/bb_mcuhub/pull/12) — the third-party migration that surfaced the `command_payloads/1` gap. Its actuator was the only one in the ecosystem to get the subscription right, and the only one to derive its accepted payload rather than hard-code it.
- [beam-bots/bb#201](https://github.com/beam-bots/bb/issues/201) — `BB.Actuator.Server` never subscribes to `[:actuator | path]`; source issue for the pipeline half of this proposal.
- [beam-bots/bb#148](https://github.com/beam-bots/bb/issues/148) — no actuator ownership or arbitration; source issue for the lease half.
- [Proposal 0018: BB.Estimator](0018-bb-estimator.md) — precedent for a wrapper GenServer owning subscriptions on behalf of a callback module (`bb/lib/bb/estimator/server.ex:168`) and for a single named inbound callback. This proposal borrows the callback unification and deliberately does not borrow the declared-input DSL.
- [BB.Actuator](../../bb/lib/bb/actuator.ex) — behaviour and public API being changed.
- [BB.Actuator.Server](../../bb/lib/bb/actuator/server.ex) — wrapper gaining the pipeline.
- [BB.PubSub](../../bb/lib/bb/pub_sub.ex) — hierarchical dispatch and the `message_types` filter this relies on.
- [BB.Safety](../../bb/lib/bb/safety.ex) — `armed?/1`, moving from driver responsibility to framework responsibility.
- [Tutorial 12: Writing an Actuator](../../bb/documentation/tutorials/12-writing-an-actuator.md) — documents the inbound pipeline this proposal makes real.
- [ROS 2 controller_manager](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html) — prior art for claiming command interfaces, and for exclusive access as a precondition rather than a convention.
