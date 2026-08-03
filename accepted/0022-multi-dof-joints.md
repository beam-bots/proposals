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

`BB.Motion` checks the chain's joint types against the solver's declared support
once per call and refuses rather than mis-solving. The permissive default is the
important part: a solver that predates this proposal gets a clear refusal instead
of a wrong answer, with no change to its code.

**Compile-time would be better, and this is the path to it.** `@after_verify` is
already an established pattern in `bb` — `BB.Error` uses
`@after_verify {BB.Error, :__verify_severity_impl__}`, and
`validate_child_spec_behaviours_transformer.ex` notes that verifiers run there.
For a solver check to run at compile time, the robot has to know which solver it
uses, which means **declaring solvers in the DSL**:

```elixir
settings do
  ik_solver {BB.IK.DLS, max_iterations: 100}
end
```

A Spark verifier could then compare `supported_joint_types/0` against the
topology's joint types and warn or fail during compilation, using exactly the
same callback this proposal adds.

That change is worth making on its own merits, independently of multi-DoF joints.
Solver options are currently an untyped keyword list with no schema or
validation, and `bb_ik_dls` and `bb_ik_fabrik` already disagree on their
`max_iterations` defaults — 100 against 50 — with nothing to reconcile them.
Declaring solvers in the DSL would subject them to the same option validation
every other component already gets.

It is out of scope here. This proposal adds the callback and the call-time check,
which is what correctness requires; see Open Questions.

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
- [ ] `BB.IK.Solver` gains an optional `supported_joint_types/0` callback,
      defaulting to the single-DoF types when unimplemented
- [ ] `BB.Motion` refuses a solve whose chain contains a joint type the chosen
      solver doesn't declare support for, rather than mis-solving
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
- [ ] `BB.Motion` logs a warning at the first refused solve naming the offending
      joint and its type, so the cause is obvious without reading a stacktrace
- [ ] Velocity/twist representation for multi-DoF joints
- [ ] `BB.Robot.Topology` path helpers aware of per-joint DoF

### Won't Have

- [ ] World/earth/`map`/`odom` frame conventions or semantics
- [ ] Geodetic conversions (Proposal 0023)
- [ ] Odometry estimators of any kind
- [ ] Mobile-base motion planning (Proposal 0006)
- [ ] Non-holonomic constraints — a `:planar` joint is fully holonomic in this
      proposal, so declaring one does not stop a solver commanding sideways
      motion a differential-drive base can't achieve
- [ ] Declaring IK solvers in the DSL, which is what compile-time solver
      verification needs; see Open Questions

---

## Open Questions

1. **ETS representation of a floating configuration.** A `BB.Math.Transform`
   wraps an Nx tensor, which is heavy to store per-row and per-read. Storing
   seven floats (quaternion plus translation) and materialising a `Transform` on
   read is likely better, but should be measured rather than assumed.

2. **How much of `defn.ex` has to change.** The expansion could happen entirely
   in `chain_tensors/3`, leaving the kernel untouched — or the kernel may need
   additional masks. This wants a spike before committing to an estimate.

3. **Which frame the floating Jacobian columns are expressed in.** Body frame
   and world frame are both defensible and the choice affects what an IK solver's
   delta means. Should follow whatever convention makes `bb_ik_dls` correct with
   the least special-casing.

4. **Whether `:planar` should be non-holonomic-aware.** A differential-drive base
   cannot translate sideways. Modelling that constraint is a solver concern
   rather than a joint concern, but the joint may need to carry a hint. Deferring
   risks a solver commanding physically impossible motion.

5. **Should a floating joint be settable in parts?** Setting only the yaw of a
   floating base is a plausible want, but a partial-update API invites
   inconsistent intermediate states.

6. **Root link semantics.** With a floating joint, is the root link still the
   kinematic root, or is it a reference frame the robot moves *within*? This
   proposal doesn't need an answer, but the follow-up frame proposal will.

7. **Should IK solvers be DSL-declared, so the check can move to compile time?**
   `supported_joint_types/0` gives a correct call-time refusal, but a wrong
   solver choice is a *configuration* error and configuration errors are better
   caught during compilation. Declaring the solver in `settings` would let a
   Spark verifier compare the callback against the topology, and would
   incidentally bring solver options under the same validation as every other
   component's — currently they're an unschema'd keyword list whose defaults two
   solvers already disagree on. Probably its own proposal.

8. **Whether refusal should be an error or a warning.** A hard error is safer,
   but it would break any existing caller that passes FABRIK for a chain
   containing a joint type it never supported — though such a caller is getting
   wrong answers today, so "break" may be the wrong word.

---

## References

- [URDF joint specification](http://wiki.ros.org/urdf/XML/joint) — the
  `floating` and `planar` types this enum mirrors
- Proposal 0018 (`BB.Estimator`) — the abstraction that would drive these joints
- Proposal 0006 (`bb_motion_planning`) — mobile-base planning, downstream of this
- Proposal 0023 (`bb_geo`) — the geodetic half of mobile-robot support
- `bb/lib/bb/robot/kinematics.ex` — the stubs this proposal replaces
