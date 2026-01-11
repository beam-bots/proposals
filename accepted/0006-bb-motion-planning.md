<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal 0006: bb_motion_planning

## Summary

A motion planning package for Beam Bots that provides sampling-based path planning, trajectory interpolation, and time parameterisation. Builds on BB's existing collision detection to enable collision-free motion planning.

## Motivation

BB currently provides:

- **Inverse kinematics** (`BB.IK.Solver`) - solve for joint positions given end-effector pose
- **Motion primitives** (`BB.Motion`) - move to IK-solved positions
- **Collision detection** (`BB.Collision`) - self-collision and environment collision

However, IK alone doesn't guarantee collision-free paths. A robot arm reaching around an obstacle may have a valid goal pose but collide during the motion. Motion planning solves this by searching for collision-free paths through configuration space.

### Use Cases

1. **Pick and place** - Reach into a bin, grasp an object, retract without hitting the bin walls
2. **Obstacle avoidance** - Move around detected obstacles in the workspace
3. **Self-collision avoidance** - Plan motions for robots with complex geometry (humanoids, multi-arm)
4. **Constrained motion** - Keep a cup upright while moving, maintain tool orientation

### Why Not Just Use IK?

IK answers: "What joint positions achieve this pose?"

Motion planning answers: "How do I get from here to there without hitting anything?"

Consider a robot arm that needs to move from one side of a table to the other, with an obstacle in the middle. IK can solve both the start and goal poses, but linear interpolation between them would collide with the obstacle. Motion planning finds the path that goes around.

## Design

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    BB.MotionPlanning.Pipeline               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │  Validate   │→ │   Planner   │→ │  Time Parameterise  │  │
│  │   Start     │  │ (RRT, PRM)  │  │  (splines, TOTG)    │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
         ↓                 ↓                    ↓
┌─────────────────────────────────────────────────────────────┐
│                    BB.MotionPlanning.Scene                  │
│         (robot + obstacles + attached objects)              │
└─────────────────────────────────────────────────────────────┘
         ↓                 ↓                    ↓
┌─────────────────────────────────────────────────────────────┐
│                      BB.Collision                           │
│            (self-collision + environment)                   │
└─────────────────────────────────────────────────────────────┘
```

### Core Components

#### 1. Planning Scene

A managed world model containing the robot and environment:

```elixir
defmodule BB.MotionPlanning.Scene do
  @type t :: %__MODULE__{
    robot: BB.Robot.t(),
    obstacles: %{id :: term() => BB.Collision.obstacle()},
    attached: %{link :: atom() => [{id :: term(), BB.Collision.obstacle()}]},
    allowed_collisions: MapSet.t({atom(), atom() | :environment})
  }

  @doc "Create a scene from a robot definition"
  def new(robot)

  @doc "Add an obstacle to the environment"
  def add_obstacle(scene, id, obstacle)

  @doc "Remove an obstacle"
  def remove_obstacle(scene, id)

  @doc "Attach an object to a link (e.g., grasped object)"
  def attach(scene, link, id, obstacle)

  @doc "Detach an object from a link"
  def detach(scene, link, id)

  @doc "Allow collision between two entities"
  def allow_collision(scene, entity_a, entity_b)

  @doc "Check if a configuration is valid (limits + collision-free)"
  def is_valid?(scene, positions, opts \\ [])

  @doc "Get all obstacles as a list for BB.Collision"
  def all_obstacles(scene)
end
```

The scene wraps `BB.Collision` to provide:
- Obstacle management with IDs for add/remove
- Attached objects that move with links
- Allowed collision pairs (skip checking certain pairs)
- Combined validity check (limits + self-collision + environment)

#### 2. Planner Behaviour

```elixir
defmodule BB.MotionPlanning.Planner do
  @type path :: [positions :: %{atom() => float()}]
  @type goal ::
    {:joint, %{atom() => float()}} |
    {:pose, link :: atom(), BB.Math.Transform.t()} |
    {:position, link :: atom(), BB.Math.Vec3.t()}

  @type meta :: %{
    iterations: non_neg_integer(),
    nodes_explored: non_neg_integer(),
    planning_time_ms: float()
  }

  @type error ::
    :start_in_collision |
    :goal_in_collision |
    :goal_unreachable |
    :timeout |
    :no_solution

  @callback plan(Scene.t(), goal(), keyword()) ::
    {:ok, path(), meta()} | {:error, error()}
end
```

#### 3. RRT-Connect Planner

The primary planner, good for general-purpose motion planning:

```elixir
defmodule BB.MotionPlanning.Planner.RRTConnect do
  @behaviour BB.MotionPlanning.Planner

  @moduledoc """
  Rapidly-exploring Random Trees with bidirectional growth.

  Grows two trees - one from start, one from goal - and attempts
  to connect them. Generally faster than single-tree RRT for
  problems where both start and goal are known.

  ## Options

  - `:max_iterations` - Maximum planning iterations (default: 5000)
  - `:step_size` - Maximum joint movement per extension (default: 0.1 rad)
  - `:goal_bias` - Probability of sampling goal vs random (default: 0.05)
  - `:timeout_ms` - Planning timeout in milliseconds (default: 5000)
  - `:collision_check_resolution` - Interpolation steps for edge validation (default: 10)
  """

  def plan(scene, goal, opts \\ [])
end
```

Algorithm outline:
1. Initialise tree_start with start configuration, tree_goal with goal
2. Loop until connected or max iterations:
   a. Sample random configuration (or goal with `goal_bias` probability)
   b. Find nearest node in tree_start
   c. Extend towards sample (respecting `step_size`)
   d. If extension valid (collision-free), add to tree
   e. Try to connect tree_goal to new node
   f. Swap trees and repeat
3. If connected, extract path from start through connection to goal
4. Optionally smooth path

#### 4. PRM Planner (Optional)

Probabilistic Roadmap for repeated queries in the same environment:

```elixir
defmodule BB.MotionPlanning.Planner.PRM do
  @behaviour BB.MotionPlanning.Planner

  @moduledoc """
  Probabilistic Roadmap planner.

  Builds a roadmap of the configuration space that can be reused
  for multiple queries. Best when planning many paths in a static
  environment.

  ## Two-Phase Operation

  1. **Learning phase**: Build roadmap with `build_roadmap/2`
  2. **Query phase**: Plan paths with `plan/3`

  ## Options

  - `:num_samples` - Nodes to sample for roadmap (default: 1000)
  - `:k_nearest` - Neighbours to connect per node (default: 10)
  - `:max_edge_length` - Maximum connection distance (default: 1.0 rad)
  """

  def build_roadmap(scene, opts \\ [])
  def plan(scene, goal, opts \\ [])
end
```

#### 5. Trajectory Interpolation

Convert geometric paths to time-parameterised trajectories:

```elixir
defmodule BB.MotionPlanning.Trajectory do
  @type waypoint :: %{
    time: float(),
    positions: %{atom() => float()},
    velocities: %{atom() => float()},
    accelerations: %{atom() => float()}
  }

  @type t :: %__MODULE__{
    joint_names: [atom()],
    waypoints: [waypoint()],
    duration: float()
  }

  @doc "Create trajectory from path using specified interpolation"
  def from_path(path, robot, opts \\ [])

  @doc "Sample trajectory at a specific time"
  def sample(trajectory, time)

  @doc "Convert to BB.Message.Actuator.Command.Trajectory"
  def to_command(trajectory)
end
```

#### 6. Interpolation Methods

```elixir
defmodule BB.MotionPlanning.Trajectory.LinearInterpolation do
  @moduledoc "Simple linear interpolation with trapezoidal velocity profile"
end

defmodule BB.MotionPlanning.Trajectory.CubicSpline do
  @moduledoc """
  Cubic spline interpolation ensuring velocity continuity.

  Uses natural cubic splines with zero velocity at endpoints
  or specified boundary velocities.
  """
end

defmodule BB.MotionPlanning.Trajectory.QuinticSpline do
  @moduledoc """
  Quintic polynomial interpolation ensuring acceleration continuity.

  Smoother than cubic but requires more computation.
  """
end
```

#### 7. Time Parameterisation

```elixir
defmodule BB.MotionPlanning.Trajectory.TimeParameterisation do
  @moduledoc """
  Time-optimal trajectory generation respecting joint limits.

  Given a geometric path (positions only), computes the fastest
  trajectory that respects velocity and acceleration limits.

  ## Algorithm

  Uses the Time-Optimal Path Parameterisation (TOPP) approach:
  1. Compute maximum velocity at each path point given limits
  2. Forward pass: accelerate as fast as possible
  3. Backward pass: decelerate to respect future constraints
  4. Integrate to get timestamps

  ## Options

  - `:velocity_scale` - Scale factor for max velocities (0.0-1.0, default: 1.0)
  - `:acceleration_scale` - Scale factor for max accelerations (0.0-1.0, default: 1.0)
  """

  def parameterise(path, robot, opts \\ [])
end
```

#### 8. Path Smoothing

```elixir
defmodule BB.MotionPlanning.PathSmoothing do
  @moduledoc """
  Post-processing to improve path quality.
  """

  @doc """
  Shortcut smoothing - try to connect non-adjacent waypoints directly.

  Iteratively attempts to skip intermediate waypoints if the direct
  path is collision-free. Reduces unnecessary detours.
  """
  def shortcut(path, scene, opts \\ [])

  @doc """
  B-spline smoothing - fit a smooth curve through waypoints.

  Produces a smoother path but may deviate slightly from original waypoints.
  """
  def bspline(path, opts \\ [])
end
```

#### 9. Planning Constraints (Phase 2)

```elixir
defmodule BB.MotionPlanning.Constraint do
  @type t ::
    {:orientation, link :: atom(), axis :: Vec3.t(), tolerance :: float()} |
    {:position_bounds, link :: atom(), min :: Vec3.t(), max :: Vec3.t()} |
    {:joint_bounds, joint :: atom(), min :: float(), max :: float()}

  @doc "Check if constraint is satisfied at configuration"
  def satisfied?(constraint, robot, positions)

  @doc "Project configuration to satisfy constraint (for constrained planning)"
  def project(constraint, robot, positions)
end
```

Constraints allow specifying requirements that must hold along the entire path, not just at start/goal. Examples:
- Keep a cup upright (orientation constraint on end-effector)
- Stay within a workspace region (position bounds)
- Avoid joint limits with margin (tighter joint bounds)

#### 10. Pipeline

High-level orchestration:

```elixir
defmodule BB.MotionPlanning.Pipeline do
  @type plan_request :: %{
    goal: Planner.goal(),
    constraints: [Constraint.t()],
    planner: module(),
    planner_opts: keyword(),
    smoothing: :none | :shortcut | :bspline,
    interpolation: :linear | :cubic | :quintic,
    velocity_scale: float(),
    acceleration_scale: float()
  }

  @doc "Plan a collision-free trajectory"
  def plan(scene, request)

  @doc "Plan with default options"
  def plan(scene, goal) when not is_map(goal)
end
```

Example usage:

```elixir
# Create scene with obstacles
scene =
  BB.MotionPlanning.Scene.new(MyRobot.robot())
  |> BB.MotionPlanning.Scene.add_obstacle(:table,
       BB.Collision.obstacle(:box, table_centre, table_half_extents))
  |> BB.MotionPlanning.Scene.add_obstacle(:mug,
       BB.Collision.obstacle(:cylinder, mug_centre, mug_radius, mug_height))

# Plan to goal pose
goal = {:pose, :end_effector, target_transform}

case BB.MotionPlanning.Pipeline.plan(scene, goal) do
  {:ok, trajectory} ->
    # Execute trajectory
    BB.MotionPlanning.Trajectory.to_command(trajectory)
    |> MyRobot.send_trajectory()

  {:error, :no_solution} ->
    Logger.warning("No collision-free path found")
end
```

#### 11. Command Integration

```elixir
defmodule BB.Command.PlanAndMove do
  use BB.Command

  @moduledoc """
  Plan and execute a collision-free motion to a goal.

  ## Goal Types

  - `{:joint, %{joint => position}}` - Joint space goal
  - `{:pose, link, transform}` - Cartesian pose goal
  - `{:position, link, vec3}` - Position-only goal (orientation free)

  ## Options

  - `:scene` - Planning scene (required if obstacles present)
  - `:planner` - Planner module (default: RRTConnect)
  - `:velocity_scale` - Max velocity scaling 0.0-1.0 (default: 0.5)
  - `:timeout_ms` - Planning timeout (default: 5000)
  """

  def handle_command(%{goal: goal} = args, _from, ctx) do
    scene = Map.get(args, :scene, BB.MotionPlanning.Scene.new(ctx.robot))

    request = %{
      goal: goal,
      planner: Map.get(args, :planner, BB.MotionPlanning.Planner.RRTConnect),
      velocity_scale: Map.get(args, :velocity_scale, 0.5),
      timeout_ms: Map.get(args, :timeout_ms, 5000)
    }

    with {:ok, trajectory} <- BB.MotionPlanning.Pipeline.plan(scene, request),
         :ok <- execute_trajectory(trajectory, ctx) do
      {:ok, :reached}
    end
  end
end
```

### Error Types

```elixir
defmodule BB.Error.MotionPlanning do
  defmodule StartInCollision do
    use BB.Error, class: :kinematics, fields: [:positions, :collisions]
  end

  defmodule GoalInCollision do
    use BB.Error, class: :kinematics, fields: [:goal, :collisions]
  end

  defmodule GoalUnreachable do
    use BB.Error, class: :kinematics, fields: [:goal, :ik_error]
  end

  defmodule PlanningTimeout do
    use BB.Error, class: :kinematics, fields: [:elapsed_ms, :iterations]
  end

  defmodule NoSolution do
    use BB.Error, class: :kinematics, fields: [:iterations, :best_distance]
  end
end
```

### Performance Considerations

**Collision Checking** is the bottleneck in sampling-based planning. BB.Collision is already optimised with:
- Broad phase AABB culling
- Adjacent link exclusion
- Cached mesh bounds

Additional optimisations for motion planning:
- **Lazy collision checking**: Only check edges when they might be part of the solution
- **Hierarchical checking**: Check coarse bounds first, refine only if needed
- **Parallel sampling**: Use `Task.async_stream` for independent collision checks

**Expected Performance**:
- Simple 6-DOF arm, no obstacles: < 100ms
- 6-DOF arm with 5-10 obstacles: 100ms - 1s
- Complex scenes or tight passages: 1-5s

### Dependencies

- `bb` - Core Beam Bots (Robot, Collision, Kinematics, etc.)
- `nx` - For spline computation (already a BB dependency)

No new external dependencies required.

## Acceptance Criteria

### Phase 1: Core Planning

- [ ] `BB.MotionPlanning.Scene` manages obstacles and attached objects
- [ ] `BB.MotionPlanning.Planner.RRTConnect` finds collision-free paths
- [ ] Planner respects joint position limits
- [ ] Planner uses `BB.Collision` for validity checking
- [ ] `BB.MotionPlanning.Trajectory.from_path/3` creates time-stamped trajectories
- [ ] Linear interpolation with trapezoidal velocity profile
- [ ] Trajectories respect joint velocity limits
- [ ] Path shortcut smoothing implemented
- [ ] `BB.MotionPlanning.Pipeline.plan/2` orchestrates the full flow
- [ ] Comprehensive test coverage with various robot configurations
- [ ] Documentation with examples

### Phase 2: Enhanced Trajectories

- [ ] Cubic spline interpolation
- [ ] Quintic spline interpolation
- [ ] Time-optimal parameterisation (TOPP)
- [ ] Acceleration limit enforcement

### Phase 3: Advanced Planning

- [ ] `BB.MotionPlanning.Planner.PRM` with roadmap caching
- [ ] Path constraints (orientation, position bounds)
- [ ] Constrained planning (projection-based)
- [ ] `BB.Command.PlanAndMove` command integration
- [ ] B-spline path smoothing

### Phase 4: Optimisation

- [ ] Lazy collision checking
- [ ] Parallel sampling option
- [ ] Roadmap serialisation for PRM
- [ ] Planning benchmarks and profiling

## Open Questions

1. **Cartesian path planning**: Should we support planning paths where the end-effector follows a specific Cartesian trajectory (like a straight line)? This requires IK at each waypoint and is more constrained than configuration space planning.

2. **Real-time replanning**: Should the planner support real-time obstacle updates and replanning? This would require incremental algorithms and is significantly more complex.

3. **Multi-arm coordination**: How should we handle planning for robots with multiple independent arms? Each arm could plan independently, or we could plan in the combined configuration space.

4. **Integration with BB.Motion.Tracker**: Should planned trajectories integrate with the existing tracker for smooth execution, or use a separate execution path?

## References

- [MoveIt2 Motion Planning Concepts](https://moveit.picknik.ai/main/doc/concepts/motion_planning.html)
- [OMPL - The Open Motion Planning Library](https://ompl.kavrakilab.org/)
- [RRT-Connect Paper (Kuffner & LaValle, 2000)](http://msl.cs.uiuc.edu/~lavalle/papers/KufLav00.pdf)
- [Time-Optimal Path Parameterization](https://arxiv.org/abs/1309.0328)
- [Stanford Trajectory Generation Notes](https://see.stanford.edu/materials/aiircs223a/handout6_Trajectory.pdf)
- [BB.Collision Documentation](https://hexdocs.pm/bb/BB.Collision.html)
