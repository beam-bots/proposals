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
to the Jacobian. The state API and `BB.Message.Sensor.JointState` are changed
rather than supplemented — `bb` is pre-1.0, and a parallel scalar-only API kept
for compatibility would be a permanent translation cost paid to avoid a one-off
migration.

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

The cost to a fixed-base arm is a `case` branch it never reaches, plus a
mechanical migration where the state API is renamed. See the breaking-change
surface below.

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

There is **one** state API, and it changes. `bb` is pre-1.0, and the alternative —
keeping a scalar-only `get_all_positions/1` alongside a new multi-DoF accessor —
means two ways to read the same ETS table, one of which silently omits joints. A
function called "get all positions" that returns some of them is a lying API, and
maintaining the translation between the two is a permanent cost paid to avoid a
one-off migration across 21 call sites.

So the existing functions are **renamed and retyped**, and the old names are
deleted:

| Before | After |
|---|---|
| `get_all_positions(state)` | `get_all_configurations(state)` |
| `set_positions(state, map)` | `set_configurations(state, map)` |
| `get_chain_positions(state, link)` | `get_chain_configurations(state, link)` |
| `compute_joint_transform(robot, positions, name)` | `compute_joint_transform(robot, configurations, name)` |

```elixir
BB.Robot.State.get_all_configurations(state)
# => %{shoulder: 0.5, elbow: -0.2, base: %BB.Math.Transform{...}}

BB.Robot.State.set_configurations(state, %{shoulder: 0.5, base: transform})
BB.Robot.State.get_configuration(state, :base)
BB.Robot.State.set_configuration(state, :base, transform)
```

**The rename is the point, not incidental.** "Position" is the wrong word for a
4×4 homogeneous transform; "configuration" is the standard term for a point in a
robot's configuration space and is correct for every joint type. More usefully,
renaming makes the break *visible*: had the names been kept and only the types
widened, every existing call site would still compile and then fail at runtime on
a shape it wasn't written for. Renaming turns all 21 into compile errors that get
looked at.

`set_configuration/3` validates the value's shape against the joint type, so a
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
  meaningful interpretation in that scheme.

The asymmetry is real, but it is a property of the **chain** rather than the
robot. A legged robot whose body floats is a perfectly good FABRIK problem when
each leg is solved from body to foot, because those chains contain nothing but
revolute joints. So the answer is not to refuse solvers on robots — it is to let
the caller say which chain they mean, which today they cannot.

Both packages should document which joint types they handle, and that a chain
containing a multi-DoF joint is outside `bb_ik_fabrik`'s competence.

#### Scoping a solve with a source link

Enforcing solver/chain compatibility is out of scope — a future motion-planning
DSL extension is the likely home for it, and until then solvers are called
programmatically and the developer is responsible for asking only for chains that
make sense. But the developer currently has **no way to express the chain they
mean**, and that gap has to close for multi-DoF joints to be usable at all.

The `BB.IK.Solver` callback is:

```elixir
@callback solve(
            robot :: Robot.t(),
            state_or_positions :: Robot.State.t() | positions(),
            target_link :: atom(),
            target :: target(),
            opts :: opts()
          ) :: solve_result()
```

There is no source. Both solvers derive their chain with
`Robot.path_to(robot, target_link)`, and `BB.Robot.Topology.path_to/2` returns the
path **from the root**. So every solve implicitly spans root-to-target.

For a fixed-base arm that is exactly right. For a robot whose base floats it is
usually wrong: a legged robot solving for a foot wants the chain from the *body*,
not from a root that includes the floating base joint. Under the current callback
that chain cannot be asked for, which means the only available solve drags a
6-DoF joint into a problem that has no business containing one.

So this proposal changes the callback to take both ends:

```elixir
@callback solve(
            robot :: Robot.t(),
            state_or_positions :: Robot.State.t() | positions(),
            source_link :: atom(),
            target_link :: atom(),
            target :: target(),
            opts :: opts()
          ) :: solve_result()
```

`solve/5` is **removed**, not retained as a defaulting clause, and `source_link`
has **no default** anywhere — including in `BB.Motion`, where `:source_link`
becomes a required option alongside the already-required `:solver`.

Defaulting it to the root was tempting and is wrong. The root is the correct
source for a fixed-base arm and precisely the *wrong* one for a robot whose base
floats, where it silently yields the contaminated chain. A default that is right
for one class of robot and quietly wrong for another is the footgun this change
exists to remove — and it would hide the breakage rather than surface it, which
defeats the point of breaking the callback at all.

Every solve therefore states its own scope:

```elixir
# The chain contains only revolute joints, so FABRIK is a fine choice —
# the floating base is simply not part of the problem.
BB.Motion.move_to(robot, :front_left_foot, target,
  source_link: :body,
  solver: BB.IK.FABRIK
)
```

##### `BB.Robot.root_link/1`

Requiring the source shifts a small burden onto callers that genuinely do want
root-to-target, which is most arm commands. `bb` already exposes `root_link` as a
field on the `BB.Robot` struct but has no accessor for it, while every neighbouring
piece of topology introspection does — `get_link/2`, `get_joint/2`,
`parent_joint/2`, `child_joints/2`, `path_to/2`, `links_in_order/1`,
`joints_in_order/1`.

So add one, alongside the others:

```elixir
@spec root_link(t()) :: atom()
def root_link(%__MODULE__{root_link: root_link}), do: root_link
```

A command that wants the whole tree then says so explicitly:

```elixir
BB.Motion.move_to(robot, :gripper, target,
  source_link: BB.Robot.root_link(robot),
  solver: BB.IK.DLS
)
```

Marginally more verbose than a default, and the verbosity is the feature: the
call now records which chain was intended, so a reader can tell whether
root-to-target was a decision or an accident.

##### A new topology operation

`BB.Robot.Topology` currently offers only `path_to/2`, root-relative and served
from a precomputed `paths` map. A source link needs a path *between* two links:

```elixir
BB.Robot.Topology.path_between(topology, source_link, target_link)

# Delegated from BB.Robot, as path_to/2 already is.
BB.Robot.path_between(robot, source_link, target_link)
```

Restricted to the case where **`source_link` is an ancestor of `target_link`**,
which is a prefix drop on the existing precomputed paths and therefore cheap. It
covers the motivating cases — body-to-foot, torso-to-gripper — and anything else
returns an error.

The general case, where the two links share only a common ancestor, requires
traversing *up* the tree from the source before descending to the target, which
means composing inverted joint transforms and reasoning about what a joint's
position means when traversed backwards. That is a real piece of work with no
current use case, and it is deferred.

##### Why this belongs here rather than with the deferred DSL work

It is the minimum that makes multi-DoF joints usable with the solvers that exist
today. Without it the only expressible chain starts at the root, so a floating
base contaminates every solve on the robot and the joint types this proposal
implements would be unusable in practice with `bb_ik_fabrik` and awkward with
`bb_ik_dls`. It is also small, mechanical, and backwards-compatible, which the
DSL work is not.

### Message payloads

`BB.Message.Sensor.JointState` carries parallel `names`/`positions`/`velocities`/
`efforts` lists of floats. A floating joint fits none of the three value lists.

The tempting workaround is to leave `JointState` alone, declare it single-DoF
only, and publish multi-DoF state as `BB.Message.Estimator.Pose` instead. **That
is the wrong call.** It would mean a robot's joint state is split across two
message types on two topics, so any consumer wanting the whole configuration —
`bb_liveview`, `bb_kino`, a logger, a policy — has to subscribe to both and
correlate them by timestamp to reconstruct one instant. That is a permanent tax on
every consumer, levied to avoid changing one message.

So `JointState` changes to carry type-appropriate values, matching the state API:

| Joint type | `positions` | `velocities` | `efforts` |
|---|---|---|---|
| single-DoF | `float` | `float` | `float` |
| `:planar` | `{x, y, θ}` | `{vx, vy, ω}` | `{fx, fy, τ}` |
| `:floating` | `Transform` | `Twist` | `Wrench` |

`BB.Message.Geometry.Twist` (linear and angular `Vec3`) and
`BB.Message.Geometry.Wrench` (force and torque `Vec3`) **already exist in core** and
are exactly the right shapes, so this needs no new types.

**All three lists change at once, deliberately.** `velocities` and `efforts` have
precisely the same problem as `positions`, and breaking a widely-consumed message
twice is worse than breaking it once. Doing only what this proposal strictly needs
would mean revisiting `JointState` again the moment anything reports a floating
joint's twist.

#### Breaking-change surface

Stated plainly, since it's the cost of not papering over this:

| Change | Call sites | Packages affected |
|---|---|---|
| State API rename and retype | 21 | `bb`, `bb_ik_dls`, `bb_ik_fabrik`, `bb_policy`, `bb_example_so101` |
| `BB.IK.Solver.solve/5` → `solve/6` | 3 in `BB.Motion`, plus both solvers | `bb`, `bb_ik_dls`, `bb_ik_fabrik` |
| `:source_link` required by `BB.Motion` | ~47, many in doc examples | `bb`, `bb_ik_dls`, `bb_ik_fabrik`, `bb_example_so101`, `bb_example_wx200` |
| `JointState` value types | 24 files | `bb`, `bb_kino`, `bb_liveview`, `bb_pid_controller`, `bb_jido`, `bb_mcp`, all four `bb_servo_*` |

The `JointState` change is the widest, and most of those consumers only ever see
single-DoF joints, so in practice their handling is unchanged — but their type
specs and any exhaustive pattern matches need review. The `:source_link`
requirement is the most numerous but the most mechanical: each call site gains
either an explicit link name or `source_link: BB.Robot.root_link(robot)`. `bb` being pre-1.0 is what
makes this affordable; it will not be later, which is an argument for doing it now
rather than after more consumers exist.

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

Solving for the sensor head means choosing whether the base is part of the
problem, and the call now has to say which:

```elixir
# Aim the head by rotating the mast only. The base is not part of this.
BB.Motion.move_to(robot, :sensor_head, target,
  source_link: :chassis,
  solver: BB.IK.FABRIK
)

# Reach the target by driving *and* rotating the mast, in one solve.
BB.Motion.move_to(robot, :sensor_head, target,
  source_link: BB.Robot.root_link(robot),
  solver: BB.IK.DLS
)
```

Those are genuinely different problems, and before this change only the second was
expressible — while being the one `bb_ik_fabrik` cannot solve.

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

An existing arm's *behaviour* is unchanged — the same joints, the same values, the
same kinematics — but the calls are renamed:

```elixir
# Before
BB.Robot.State.set_positions(state, %{shoulder: 0.5, elbow: -0.2})
positions = BB.Robot.State.get_all_positions(state)

# After
BB.Robot.State.set_configurations(state, %{shoulder: 0.5, elbow: -0.2})
configurations = BB.Robot.State.get_all_configurations(state)
```

A single-DoF joint's value is still a bare float, so only the function name
changes. That is the whole migration for an arm.

---

## Acceptance Criteria

### Must Have

- [ ] `compute_joint_transform/3` returns the correct transform for `:planar`,
      composing `{x, y, θ}` in the plane defined by `axis`
- [ ] `compute_joint_transform/3` returns the correct transform for `:floating`
      from a stored `BB.Math.Transform`
- [ ] `BB.Robot.State` stores multi-DoF configurations, with
      `get_configuration/2`, `set_configuration/3`, `get_all_configurations/1`,
      `set_configurations/2`, `get_chain_configurations/2`
- [ ] The former `get_all_positions/1`, `set_positions/2` and
      `get_chain_positions/2` are **removed**, not retained as shims
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
- [ ] `BB.IK.Solver.solve/6` takes `source_link` ahead of `target_link`, and
      `solve/5` is **removed** rather than kept as a defaulting clause
- [ ] `BB.Robot.Topology.path_between/3`, restricted to a source that is an
      ancestor of the target, erroring otherwise, delegated from
      `BB.Robot.path_between/3`
- [ ] `BB.Robot.root_link/1` accessor, alongside the existing topology
      introspection functions
- [ ] `BB.Motion.move_to/4`, `move_to_multi/4`, and the `solve_only*` functions
      require `:source_link`, with no default, as they already require `:solver`
- [ ] `bb_ik_dls` and `bb_ik_fabrik` implement `solve/6` and derive their chain
      from `path_between/3` rather than `path_to/2`
- [ ] `BB.IK.DLS.Motion` and `BB.IK.FABRIK.Motion` require `:source_link` the
      same way
- [ ] `bb_ik_dls` solves chains containing multi-DoF joints
- [ ] A solve scoped `source_link: :body` on a floating-base robot produces a
      chain containing no multi-DoF joint, and `bb_ik_fabrik` handles it
- [ ] `BB.Message.Sensor.JointState` carries type-appropriate values in all
      three of `positions`, `velocities` and `efforts`, using the existing
      `BB.Message.Geometry.Twist` and `Wrench` for floating joints
- [ ] Every `JointState` consumer across the ecosystem is migrated, not just the
      ones that will see multi-DoF joints
- [ ] Round-trip tests: a configuration set, then read back through FK, produces
      the expected pose for both new types
- [ ] Regression tests proving fixed-base arm kinematics are unchanged

### Should Have

- [ ] A compile-time verifier rejecting `axis` on a `:floating` joint, and
      requiring it on `:planar`
- [ ] Documentation in both solver packages stating which joint types they handle,
      and that a chain containing a multi-DoF joint is outside `bb_ik_fabrik`'s
      competence
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
- [ ] Declaring IK solvers or their chains in the DSL. Solvers continue to be
      called programmatically, and a future motion-planning DSL extension is the
      likely home for declaring them
- [ ] Any enforcement of solver/joint-type compatibility, at compile time or
      runtime. The developer is responsible for asking a solver only for chains it
      can handle; this proposal's contribution is making the chain *expressible*
- [ ] Paths between links that share only a common ancestor, which would require
      composing inverted joint transforms

---

## Open Questions

1. **Root link semantics.** With a floating joint, is the root link still the
   kinematic root, or a reference frame the robot moves *within*? A legged robot
   solved as body-to-foot chains suggests "the root" and "the base a solver works
   from" are different concepts that currently share a name. Adding `source_link`
   makes the distinction expressible without settling what the root *means*, which
   the follow-up frame proposal will have to.

2. **How much of `defn.ex` has to change.** As much as is necessary. The expansion
   may fit entirely in `chain_tensors/3`, or the kernel may need additional masks.
   This affects effort estimation rather than the design, so it wants a spike.

3. **Which frame the floating Jacobian columns are expressed in.** Body frame and
   world frame are both defensible, and the choice changes what an IK solver's
   delta means. Should follow whatever makes `bb_ik_dls` correct with the least
   special-casing, and interacts with question 1.

4. **A planar joint's velocity and effort shape.** The proposal uses `{vx, vy, ω}`
   and `{fx, fy, τ}` to match the configuration's `{x, y, θ}`. The alternative is
   `Twist`/`Wrench` with the out-of-plane components zero, which is more uniform
   with floating joints but lets a producer emit a physically impossible
   out-of-plane value. Minor either way.

5. **Should `path_between/3` return an error or raise on a non-ancestor source?**
   Every other topology lookup returns `nil` for a miss (`get_link/2`,
   `path_to/2`), which argues for consistency — but a non-ancestor source is a
   programming error rather than a lookup miss, and returning `nil` would let it
   surface later as a confusingly empty chain.

---

## References

- [URDF joint specification](http://wiki.ros.org/urdf/XML/joint) — the
  `floating` and `planar` types this enum mirrors
- Proposal 0018 (`BB.Estimator`) — the abstraction that would drive these joints
- Proposal 0006 (`bb_motion_planning`) — mobile-base planning, downstream of this
- Proposal 0023 (`bb_geo`) — the geodetic half of mobile-robot support
- `bb/lib/bb/robot/kinematics.ex` — the stubs this proposal replaces
