<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal 0022: Multi-DoF Joints

**Status:** Draft
**Author:** James Harton
**Created:** 2026-08-04
**Dependencies:** `bb`

---

## Summary

`:floating` and `:planar` are already in `bb`'s joint-type enum, and both silently
return `Transform.identity()` from forward kinematics. This proposal implements
them properly, which means teaching the kinematics and state layers that a joint
can carry more than one degree of freedom. A `:floating` joint's configuration
becomes a `BB.Math.Transform`, a `:planar` joint's becomes `{x, y, θ}` in the
plane defined by its `axis`, and both contribute their full complement of columns
to the Jacobian. The scalar `positions` API that every existing single-DoF joint
uses is left intact and additive functions are introduced alongside it.

This is deliberately scoped to *finishing two joint types core already declares*.
It says nothing about world frames, earth frames, odometry sources, or geodesy.

---

## Motivation

### It is currently a silent wrong answer

The joint-type enum in `BB.Dsl` accepts `:floating` and `:planar`
(`lib/bb/dsl.ex:524`). `BB.Robot.Kinematics.compute_joint_transform/3` handles
them like this (`lib/bb/robot/kinematics.ex:231-238`):

```elixir
:fixed ->
  Transform.identity()

:floating ->
  Transform.identity()

:planar ->
  Transform.identity()
```

So a user can write `joint :base, type: :planar do ... end` today, get no
warning at compile time or runtime, and receive kinematics that treat the joint
as welded. Every downstream pose is wrong, and nothing says so. That is worse
than not supporting the types at all — an unsupported type would at least fail
validation.

This alone justifies the change. Either implement the types or remove them from
the enum, and removing them forecloses mobile robots.

### Beam Bots has only ever been used on fixed-base robots

Every robot built with `bb` so far — the SO-101 and WX200 arms, the examples — is
bolted to a table. The kinematic root is a fixed link, and the whole tree hangs
off it. That assumption is now baked into the state and kinematics layers, not
because anyone chose it but because nothing has pushed on it.

Extending to rovers, boats, drones, and anything else that moves through the
world means the root of the tree stops being fixed. That is exactly what a
floating or planar joint expresses.

### The justification is mobile bases, not GNSS

This proposal came out of a discussion about GNSS support, but it does not depend
on it and should not be judged on it:

- A differential-drive rover with **wheel odometry and no GPS at all** needs
  `:planar`. Its base moves in a plane; three degrees of freedom.
- A drone or an underwater vehicle needs `:floating`. Six degrees of freedom.
- A GNSS fix, wheel odometry, visual odometry, and a fused EKF estimate are all
  merely *ways to drive* such a joint. The joint's existence is upstream of all
  of them.

Whatever ends up populating the joint — see Proposal 0023 for the geodetic half —
needs somewhere to put the answer first.

### Why in core

There is no alternative. `BB.Robot.Kinematics`, `BB.Robot.State`,
`BB.Robot.Kinematics.Defn`, and the joint-type enum are all in `bb`. No satellite
package can make forward kinematics respect a non-fixed root, because forward
kinematics is core. This is not a capability to add alongside core; it is an
unfinished primitive inside it.

The cost to a fixed-base arm is a `case` branch it never reaches. The state
representation change is internal and additive.

---

## Design

### What exists today

Three facts constrain the design, and all three need to move together.

**1. Joint configuration is one float.** `BB.Robot.State` is ETS-backed with a
scalar per joint. `get_all_positions/1` is spec'd `%{atom() => float()}`,
`set_positions/2` takes the same shape, and `compute_joint_transform/3` does
`Map.get(positions, joint_name, 0.0)`.

**2. The Nx kernel is vectorised over scalars with type masks.** Every function
in `BB.Robot.Kinematics.Defn` has the shape:

```elixir
defn fk_chain(positions, origin_rpy, origin_xyz, axes, is_revolute, is_prismatic)
defn position_jacobian(positions, origin_rpy, origin_xyz, axes, is_revolute, is_prismatic)
defn orientation_jacobian(positions, origin_rpy, origin_xyz, axes, is_revolute, is_prismatic)
```

One entry in `positions` per joint, with `is_revolute`/`is_prismatic` boolean
masks selecting behaviour. `chain_tensors/3` in `BB.Robot.Kinematics` is the seam
that builds those tensors from `robot.joints`.

**3. The Jacobian is one column per joint.** `chain_jacobian/4` produces a
`{3, length(joint_names)}` tensor. A floating joint contributes six columns and a
planar joint three, so Jacobian width becomes *sum of DoF along the chain*
rather than *number of joints in the chain*. This is very likely why the types
were stubbed in the first place — it is an IK-facing change, not just an FK one.

### Degrees of freedom per joint type

| Type | DoF | Configuration | Uses `axis` |
|---|---|---|---|
| `:fixed` | 0 | — | no |
| `:revolute` | 1 | angle | yes (rotation axis) |
| `:continuous` | 1 | angle | yes (rotation axis) |
| `:prismatic` | 1 | displacement | yes (translation axis) |
| `:planar` | 3 | `{x, y, θ}` | yes (**surface normal**) |
| `:floating` | 6 | `BB.Math.Transform` | no |

`:planar` already uses `axis` as its surface normal per the existing DSL
documentation (`lib/bb/dsl.ex:506`), so the plane is already expressible and no
DSL change is needed for it.

#### When `:planar` is the right model

A planar joint is **three** degrees of freedom: two translations in the plane and
one rotation about its normal. Not two rotations — a body constrained to a plane
has only one rotational freedom, yaw.

Whether a given base is planar turns on the *surface*, not the drive:

- **Mecanum or omni base on a flat floor** — exactly `:planar`, and the case where
  it fits best. Such a base is genuinely **holonomic**: it can translate in both
  in-plane axes and rotate about the normal, all independently. The joint's three
  DoF map one-to-one onto motions the robot can actually achieve, so nothing is
  over-promised.
- **Differential-drive or Ackermann base on a flat floor** — still kinematically
  `:planar`, but the joint over-promises. It admits sideways translation the robot
  cannot perform. See below.
- **Any wheeled base on uneven terrain** — **not planar.** A rover crossing rough
  ground pitches and rolls, and its body height changes. That is six degrees of
  freedom, so the joint is `:floating` even though the robot is nominally
  "driving around on the ground". Modelling it as planar discards real motion and
  will put a mast-mounted sensor in the wrong place.

That last case is worth stating plainly because "it's a ground vehicle, so it's
planar" is the intuitive and often wrong choice.

#### Non-holonomic constraints

For a differential-drive or Ackermann base, `:planar` describes the
*configuration space* correctly while admitting *velocities* the robot cannot
realise. Nothing in this proposal prevents a solver commanding a sideways
translation such a base can't achieve.

That is left out on purpose. The constraint is a property of the drive mechanism,
not of the joint — the same `:planar` joint is unconstrained under mecanum wheels
and constrained under differential drive. Expressing it belongs with whatever
describes the drive, alongside wheel radius and track width, and is entangled with
motion planning (Proposal 0006). A holonomic joint plus a separate constraint
description is a cleaner factoring than a joint type per drive geometry.

### Representing a multi-DoF configuration

Three options were considered.

**Option A — bare list of floats.** `%{atom() => float() | [float()]}`.
Minimal change to the map shape, but every consumer of `get_all_positions/1`
has to branch on arity, and nothing stops a caller putting four floats in a
planar joint.

**Option B — pseudo-joint decomposition.** Expand `:floating` at build time into
six chained single-DoF joints (three prismatic, three revolute) and `:planar`
into three. The state layer, the Jacobian machinery, and the `defn` kernel all
work unchanged.

Rejected, for two reasons. First, `robot.joints` gains synthetic names the user
never wrote, which then surface in topology paths and `JointState` messages —
leaky in a way that is hard to un-leak later. Second and decisively,
representing a 6-DoF *configuration* as three chained revolute joints is an
Euler parameterisation, which has gimbal-lock singularities. `BB.Math.Transform`
and `BB.Math.Quaternion` exist precisely to avoid that; reintroducing Euler
angles as the storage representation would be a step backwards.

**Option C — typed configuration, expanded only for the kernel. Recommended.**

Store each joint's configuration in the shape appropriate to its type — a float
for single-DoF joints, `{x, y, θ}` for planar, a `Transform` for floating — and
expand to the vectorised scalar form *only* inside `chain_tensors/3`, where the
tensors handed to `defn` are built.

This takes the best of both. The user sees one joint with one name. Storage is
singularity-free, because a `Transform` carries a quaternion. And the `defn`
kernel keeps its vectorised shape, because by the time it sees anything the
floating joint has become six entries.

Crucially, the gimbal-lock objection to Option B does **not** apply here. A
floating base's six Jacobian columns are three unit translations and three unit
rotations expressed in a chosen frame — exact and singularity-free as a
*derivative*. Gimbal lock is a property of storing and integrating an Euler
*configuration*, which Option C never does.

### State API

Additive, so no existing arm code changes:

```elixir
# Unchanged. Still %{atom() => float()}; multi-DoF joints are absent from it.
BB.Robot.State.get_all_positions(state)
BB.Robot.State.set_positions(state, %{shoulder: 0.5})

# New. Returns every joint, in its type-appropriate shape.
BB.Robot.State.get_all_configurations(state)
# => %{shoulder: 0.5, base: %BB.Math.Transform{...}}

BB.Robot.State.get_configuration(state, :base)
BB.Robot.State.set_configuration(state, :base, transform)
```

Keeping `get_all_positions/1` scalar-only is a deliberate choice: it means
existing callers cannot silently receive a shape they were never written to
handle. Anything that wants the full picture opts in by name.

`set_configuration/3` validates arity and type against the joint, so a
three-tuple aimed at a revolute joint is an error rather than a wrong pose.

**Writes are whole-configuration and atomic.** There is no API for setting part
of a floating joint — no "just the yaw". A partial-update API invites
inconsistent intermediate states, and the natural producer of a floating joint's
configuration is an estimator emitting a complete pose, which has no use for one.

#### Storage must be lossless

`BB.Math.Transform` is a 4×4 `:f64` homogeneous matrix (`Nx.eye(4, type: :f64)`,
`Nx.as_type(tensor, :f64)`). The obvious space optimisation — decompose to a
quaternion and translation, store seven floats, recompose on read — is
**wrong** and must not be done:

- Matrix-to-quaternion is a `sqrt` with a branch on the trace, and the inverse is
  more arithmetic. Both are lossy in the low bits.
- 16 numbers → 7 → 16 is not a bijection. A matrix that has drifted slightly from
  orthonormal cannot round-trip, and recomposition silently renormalises it to a
  different matrix than the one stored.
- The round-trip happens on **every read**, so error accumulates rather than
  being a one-off.

The whole point of holding state in ETS is that it is fast *and* correct. Store
the tensor's bytes verbatim — `Nx.to_binary/1` gives 128 bytes for a 4×4 f64
matrix — and recover with `Nx.from_binary/2` plus a reshape. That is bit-exact
and involves no arithmetic at all.

Storing the raw binary rather than the `%Nx.Tensor{}` struct is also
backend-safe. A tensor struct carries backend state: fine for
`Nx.BinaryBackend`, but under EXLA it is a reference to accelerator memory, which
is not meaningfully shareable through ETS and may be invalidated out from under
a reader. `bb` uses `Nx.BinaryBackend` today, but the default backend is
user-configurable, and state storage should not silently break when someone sets
`EXLA` to speed up kinematics.

A `:planar` configuration is three floats and has no such problem; store it as a
tuple.

### Jacobian and IK solver impact

Jacobian width becomes the sum of DoF along the chain. Existing solvers are
affected differently, and the difference is worth stating plainly:

- **`bb_ik_dls`** — damped least squares is a pseudo-inverse over whatever
  Jacobian it is handed. Wider input, wider delta; dimension-agnostic. Expected
  to work with no change beyond applying the resulting delta through the new
  configuration API rather than the scalar one.
- **`bb_ik_fabrik`** — FABRIK is not Jacobian-based. It is a heuristic that
  iteratively repositions joints along a chain, and a 6-DoF floating base has no
  meaningful interpretation in that scheme. FABRIK must not silently mis-solve.

This asymmetry is a genuine capability difference between the two solvers and
should be surfaced rather than papered over.

#### Declaring solver capability

The mechanism matters here, because the obvious one doesn't reach. A solver
cannot inspect a robot's DSL state at compile time: `BB.IK.Solver`
implementations are **not declared in the DSL at all** — there is no mention of
solvers anywhere in `BB.Dsl`. A solver is chosen per call, as
`BB.Motion.move_to(Robot, link, target, solver: BB.IK.FABRIK)`. So an
`@after_verify` hook in `bb_ik_fabrik` has no robot to look at, and one on the
robot module has no idea which solver will be used.

The minimum that makes any check possible is a capability declaration on the
behaviour itself:

```elixir
defmodule BB.IK.Solver do
  @doc """
  The joint types this solver can handle.

  Optional. A solver that doesn't implement it is assumed to support the
  single-DoF types only, so an existing or third-party solver that never
  contemplated multi-DoF joints reports the truth without being edited.
  """
  @callback supported_joint_types() :: [atom()]

  @optional_callbacks supported_joint_types: 0
end
```

```elixir
# bb_ik_fabrik
def supported_joint_types, do: [:fixed, :revolute, :continuous, :prismatic]

# bb_ik_dls
def supported_joint_types,
  do: [:fixed, :revolute, :continuous, :prismatic, :planar, :floating]
```

The permissive default matters: a solver that predates this proposal reports the
truth without its author touching it.

#### Compatibility is a property of the chain, not the robot

This proposal deliberately stops at the callback and does **not** enforce it,
because enforcement is harder than it first appears and belongs elsewhere.

The naive check — "this robot has a floating joint, so refuse FABRIK" — is wrong.
Consider a legged robot: the body floats, but each leg is solved as its own chain
from body to foot, and those chains contain nothing but revolute joints. FABRIK
is a perfectly good choice there. A robot-level check would false-positive on
exactly the case where the solver is being used correctly.

So the question is never "does this solver support this robot" but "does this
solver support **this chain**" — which depends on which chain is being solved, and
therefore on how the caller has scoped the problem.

That has two consequences:

1. **A runtime check is possible but is the wrong shape.** `BB.Motion` could
   compare the chain's joint types against `supported_joint_types/0` on every
   call. It would be correct, but a wrong solver choice is a *configuration*
   error, and configuration errors should surface when the configuration is
   written, not when the robot is running.

2. **A compile-time check needs the chains to be declarable.** For a Spark
   verifier to check anything, the robot has to know which solver solves which
   chain. That means expressing solver configuration in the DSL — something
   closer to

   ```elixir
   # Illustrative only; the real shape needs design work.
   settings do
     ik_solver :legs, {BB.IK.FABRIK, max_iterations: 50},
       chains: [{:body, :front_left_foot}, {:body, :front_right_foot}]

     ik_solver :arm, {BB.IK.DLS, max_iterations: 100},
       chains: [{:torso, :gripper}]
   end
   ```

   at which point `@after_verify` — already an established pattern in `bb`, via
   `BB.Error`'s `@after_verify {BB.Error, :__verify_severity_impl__}` — can
   compare each declared chain's joint types against its solver's declared
   support and fail the build.

That is a substantial design problem in its own right: it has to express chains,
per-chain solver options, and probably per-chain solver *selection* at runtime,
and it interacts with what the root link means once the base floats. It is not
something to smuggle into a proposal about joint types.

It is also worth doing independently of multi-DoF joints. Solver options are
currently an untyped keyword list with no schema or validation, and `bb_ik_dls`
and `bb_ik_fabrik` already disagree on their `max_iterations` defaults — 100
against 50 — with nothing to reconcile them. Declaring solvers in the DSL would
bring them under the same option validation every other component gets.

**Consequence for sequencing.** Until that proposal lands there is no check, and
FABRIK handed a chain containing a multi-DoF joint will produce a wrong answer
rather than a refusal. That is a real gap and it is called out in Open Questions
rather than papered over with a runtime error that the eventual compile-time check
would replace.

### Message payloads

`BB.Message.Sensor.JointState` carries parallel `names`/`positions`/
`velocities`/`efforts` lists of floats. A floating joint's pose does not fit.

Rather than overload `JointState`, multi-DoF joints publish their state as
`BB.Message.Estimator.Pose`, which already exists and is exactly the right
shape. `JointState` continues to describe single-DoF joints only, and its
documentation is updated to say so.

Velocity for a floating joint is a 6-vector twist rather than a scalar. Whether
that needs a new payload or fits in `BB.Message.Estimator.Odometry` is left as an
open question, since nothing in this proposal produces one yet.

### Explicitly out of scope

- **World, earth, `map`, or `odom` frame conventions.** This proposal provides
  the joint. What the root link is called and what it means is a separate
  question.
- **Geodesy and GNSS.** See Proposal 0023.
- **Odometry sources.** Wheel, visual, or fused — all consumers of this, none
  part of it.
- **Motion planning for mobile bases.** See Proposal 0006.
- **Driving a floating joint from an estimator.** The mechanism to populate the
  joint is deferred; this proposal only makes the joint real.

---

## Package Structure

Entirely within `bb`. Files touched:

```
bb/lib/bb/
├── dsl.ex                        # joint entity docs; possibly DoF validation
├── robot/
│   ├── state.ex                  # multi-DoF configuration storage + API
│   ├── kinematics.ex             # compute_joint_transform/3, chain_tensors/3
│   └── kinematics/defn.ex        # tensor plumbing for expanded DoF
└── message/
    └── sensor/joint_state.ex     # docs: single-DoF only
```

No new dependencies. Nx is already a core dependency.

Downstream packages needing a change:

| Package | Change |
|---|---|
| `bb_ik_dls` | Apply deltas via the configuration API |
| `bb_ik_fabrik` | Error clearly on a multi-DoF joint in the chain |

---

## User Experience

A differential-drive rover. The base moves in the ground plane, so the joint
connecting the root to the chassis is planar with a vertical surface normal:

```elixir
defmodule Rover do
  use BB

  topology do
    link :odom do
      joint :base, type: :planar, axis: {0.0, 0.0, 1.0} do
        link :chassis do
          joint :mast, type: :revolute, axis: {0.0, 0.0, 1.0} do
            link :sensor_head do
              sensor :lidar, {SomeLidar, bus: "spi-0"}
            end
          end
        end
      end
    end
  end
end
```

Forward kinematics through `:sensor_head` now correctly composes the base's
planar motion with the mast rotation, so a LIDAR scan can be expressed in the
`:odom` frame.

Something — an odometry estimator, out of scope here — drives the joint:

```elixir
BB.Robot.State.set_configuration(state, :base, {12.4, -3.1, 1.57})
```

A drone, where the base is fully free:

```elixir
topology do
  link :world do
    joint :base, type: :floating do
      link :airframe do
        sensor :imu, {BB.Sensor.BMI323, bus: "i2c-1"}
      end
    end
  end
end
```

```elixir
BB.Robot.State.set_configuration(state, :base, pose_transform)
```

Nothing about an existing arm changes:

```elixir
# Still exactly as it was.
BB.Robot.State.set_positions(state, %{shoulder: 0.5, elbow: -0.2})
positions = BB.Robot.State.get_all_positions(state)
```

---

## Acceptance Criteria

### Must Have

- [ ] `compute_joint_transform/3` returns the correct transform for `:planar`,
      composing `{x, y, θ}` in the plane defined by `axis`
- [ ] `compute_joint_transform/3` returns the correct transform for `:floating`
      from a stored `BB.Math.Transform`
- [ ] `BB.Robot.State` stores multi-DoF configurations, with
      `get_configuration/2`, `set_configuration/3`, `get_all_configurations/1`
- [ ] `get_all_positions/1` and `set_positions/2` keep their existing scalar
      contract and behaviour
- [ ] `set_configuration/3` rejects a value whose shape doesn't match the joint
      type
- [ ] `chain_tensors/3` expands multi-DoF joints into the vectorised form the
      `defn` kernel expects
- [ ] Jacobian width is the sum of DoF along the chain; a floating joint
      contributes three translation and three rotation columns
- [ ] Multi-DoF configurations are stored losslessly — bit-exact round-trip
      through ETS, with no quaternion decomposition and no tensor structs held in
      the table
- [ ] `set_configuration/3` writes a whole configuration; there is no partial
      update API
- [ ] `BB.IK.Solver` gains an optional `supported_joint_types/0` callback,
      defaulting to the single-DoF types when unimplemented
- [ ] `bb_ik_fabrik` declares single-DoF support only
- [ ] `bb_ik_dls` declares multi-DoF support and solves such chains
- [ ] `BB.Message.Sensor.JointState` documents that it describes single-DoF
      joints only
- [ ] Round-trip tests: a configuration set, then read back through FK, produces
      the expected pose for both new types
- [ ] Regression tests proving fixed-base arm kinematics are unchanged

### Should Have

- [ ] A compile-time verifier rejecting `axis` on a `:floating` joint, and
      requiring it on `:planar`
- [ ] Documentation in both solver packages stating which joint types they handle,
      and that `bb_ik_fabrik` on a chain containing a multi-DoF joint is wrong
      until solver declaration lands
- [ ] Velocity/twist representation for multi-DoF joints
- [ ] `BB.Robot.Topology` path helpers aware of per-joint DoF

### Won't Have

- [ ] World/earth/`map`/`odom` frame conventions or semantics
- [ ] Geodetic conversions (Proposal 0023)
- [ ] Odometry estimators of any kind
- [ ] Mobile-base motion planning (Proposal 0006)
- [ ] Non-holonomic constraints — a `:planar` joint is fully holonomic here, so
      declaring one does not stop a solver commanding sideways motion a
      differential-drive base can't achieve. The constraint belongs with the drive
      description, not the joint
- [ ] Declaring IK solvers and their chains in the DSL, and the compile-time
      verification that depends on it. Needs its own proposal
- [ ] Any enforcement of solver/joint-type compatibility. This proposal ships the
      callback as metadata only

---

## Open Questions

1. **Solver and chain declaration in the DSL.** The largest thing this proposal
   defers, and a prerequisite for compile-time verification of solver/joint-type
   compatibility. It has to express chains, per-chain solver selection and
   options, and it interacts with question 2. Needs its own proposal; see
   "Compatibility is a property of the chain, not the robot".

2. **Root link semantics.** With a floating joint, is the root link still the
   kinematic root, or a reference frame the robot moves *within*? A legged robot
   solved as body-to-foot chains implies the body — not the root — is the natural
   base for those solves, which suggests "the root" and "the base a solver works
   from" are different concepts that currently share a name. Entangled with
   question 1 and with the follow-up frame proposal.

3. **How much of `defn.ex` has to change.** As much as is necessary. The expansion
   may fit entirely in `chain_tensors/3`, or the kernel may need additional masks.
   This affects effort estimation, not the design, so it wants a spike rather than
   a decision.

4. **Which frame the floating Jacobian columns are expressed in.** Body frame and
   world frame are both defensible, and the choice changes what an IK solver's
   delta means. Should follow whatever makes `bb_ik_dls` correct with the least
   special-casing, but interacts with question 2 — if the solver's base is the body
   rather than the root, that likely settles it.

5. **The gap before solver declaration lands.** Between this proposal and the
   solver proposal, `bb_ik_fabrik` handed a chain containing a multi-DoF joint
   produces a wrong answer with nothing to catch it. Options: ship the two
   proposals together; ship this one and accept the window, documented in both
   solver packages; or add a temporary runtime refusal that the compile-time check
   later replaces. The third contradicts the preference for compile-time
   diagnostics over runtime ones, so it is listed for completeness rather than
   recommended.

6. **Velocity for a multi-DoF joint.** A floating joint's velocity is a 6-vector
   twist rather than a scalar. Whether that needs a new payload or fits
   `BB.Message.Estimator.Odometry` is unresolved, but nothing in this proposal
   produces one yet.

---

## References

- [URDF joint specification](http://wiki.ros.org/urdf/XML/joint) — the
  `floating` and `planar` types this enum mirrors
- Proposal 0018 (`BB.Estimator`) — the abstraction that would drive these joints
- Proposal 0006 (`bb_motion_planning`) — mobile-base planning, downstream of this
- Proposal 0023 (`bb_geo`) — the geodetic half of mobile-robot support
- `bb/lib/bb/robot/kinematics.ex` — the stubs this proposal replaces
