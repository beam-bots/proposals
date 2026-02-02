<!--
SPDX-FileCopyrightText: 2026 James Harton (via Arthur AI Assistant)

SPDX-License-Identifier: Apache-2.0
-->

# Proposal: Motion Filters

**Status:** Draft
**Author:** James Harton (via Arthur)
**Created:** 2026-02-02

---

## Summary

Motion filters provide a pipeline mechanism for transforming actuator commands before they reach hardware. Filters can smooth motion, limit velocity/acceleration, enforce safety bounds, or apply other transformations. They're defined either as reusable named pipelines or inline on individual actuators, enabling declarative configuration of motion characteristics in the robot DSL.

---

## Motivation

### The Problem

Currently, motion characteristics like smoothing, velocity limiting, and acceleration control must be implemented in one of three ways:

1. **In the command/behavior layer** — Clutters high-level logic with low-level concerns
2. **In custom controllers** — Requires boilerplate GenServers for every filtering need
3. **In the actuator driver** — Not reusable, not configurable per-actuator

None of these are satisfactory. Motion filtering is a **cross-cutting concern** that should be:
- **Declarative** — Configured in the robot DSL, not imperative code
- **Composable** — Multiple filters can be chained
- **Reusable** — Same filter logic applied to different actuators
- **Source-agnostic** — Applies to teleop, autonomous behaviors, scripted motions equally

### Use Cases

**1. Teleoperation smoothing**
```elixir
# Raw keyboard/gamepad input is jerky
# Filter smooths it before reaching servos
actuator :shoulder, {BB.Servo.Robotis, controller: :u2d2, id: 1} do
  filter {BB.MotionFilter.Smooth, method: :exponential, alpha: 0.8}
end
```

**2. Safety velocity limiting**
```elixir
# Prevent dangerously fast motion regardless of command source
actuator :wrist, {BB.Servo.Feetech, controller: :uart, id: 5} do
  filter {BB.MotionFilter.LimitVelocity, max: ~u[30 degrees per second]}
end
```

**3. Reusable motion profiles**
```elixir
# Define once, apply to multiple actuators
motion_pipeline :gentle do
  filter {BB.MotionFilter.Smooth, method: :exponential, alpha: 0.8}
  filter {BB.MotionFilter.LimitVelocity, max: ~u[30 degrees per second]}
  filter {BB.MotionFilter.LimitAcceleration, max: ~u[50 degrees per second squared]}
end

actuator :shoulder, {BB.Servo.Robotis, controller: :u2d2, id: 1} do
  pipe_through [:gentle]
end

actuator :elbow, {BB.Servo.Robotis, controller: :u2d2, id: 2} do
  pipe_through [:gentle]
end
```

**4. Preventing jitter**
```elixir
# Ignore tiny position deltas from noisy sensors
actuator :gripper, {BB.Servo.Feetech, controller: :uart, id: 6} do
  filter {BB.MotionFilter.Deadband, threshold: ~u[0.5 degrees]}
end
```

**5. Different profiles for different modes**
```elixir
motion_pipeline :teleop_safe do
  filter {BB.MotionFilter.Smooth, alpha: 0.9}
  filter {BB.MotionFilter.LimitVelocity, max: ~u[20 degrees per second]}
end

motion_pipeline :autonomous_fast do
  filter {BB.MotionFilter.LimitVelocity, max: ~u[90 degrees per second]}
end

# Runtime switching via parameters or commands
actuator :base_motor, {MyDriver, port: 1} do
  pipe_through [param([:motion, :active_pipeline])]
end
```

### Why Not Controllers?

`BB.Controller` is designed for **reactive behaviors** — responding to state changes with conditional logic. Filters are **pure transformations** — they modify commands in-flight regardless of system state.

**Conceptual difference:**

| Controllers | Filters |
|-------------|---------|
| Reactive | Transformative |
| State-driven | Stateless (or minimal state) |
| Conditional logic | Pure functions |
| Trigger actuators | Intercept actuator commands |
| High-level behavior | Low-level signal processing |

Both are needed. Filters complement controllers by handling the low-level signal conditioning that applies universally.

---

## Design

### Core Abstraction

```elixir
defmodule BB.MotionFilter do
  @moduledoc """
  Behaviour for motion filters that transform actuator commands.

  Filters are lightweight, composable transformations applied to commands
  before they reach hardware drivers. They can smooth motion, limit rates,
  enforce safety bounds, or perform other signal processing.

  ## State Management

  Filters can maintain state (e.g., for exponential smoothing, tracking
  previous values). State is isolated per actuator — each actuator instance
  gets its own filter state.

  ## Command Types

  Filters receive typed command messages:
  - `{:position, value, opts}` - Position target
  - `{:velocity, value, opts}` - Velocity target
  - `{:effort, value, opts}` - Effort/torque target

  They must return the same command type (but may modify value/opts).
  """

  @type command ::
          {:position, number(), Keyword.t()}
          | {:velocity, number(), Keyword.t()}
          | {:effort, number(), Keyword.t()}

  @type state :: term()

  @doc """
  Initialize filter state.

  Called once when the actuator starts. Receives resolved options from DSL.
  """
  @callback init(opts :: Keyword.t()) :: {:ok, state()} | {:error, term()}

  @doc """
  Transform a command.

  Receives the incoming command and current filter state. Returns the
  transformed command and updated state.

  Filters may:
  - Modify the command value
  - Add/modify options (e.g., velocity limits)
  - Drop commands (return `:skip`)
  - Pass through unchanged

  ## Examples

      # Modify value
      def filter({:position, pos, opts}, state) do
        smoothed = smooth(pos, state.prev)
        {{:position, smoothed, opts}, %{state | prev: smoothed}}
      end

      # Skip command if too similar to previous
      def filter({:position, pos, opts}, state) do
        if abs(pos - state.prev) < state.threshold do
          {:skip, state}
        else
          {{:position, pos, opts}, %{state | prev: pos}}
        end
      end
  """
  @callback filter(command(), state()) ::
              {command(), state()} | {:skip, state()}

  @doc """
  Clean up filter state.

  Called when the actuator terminates. Optional.
  """
  @callback terminate(state()) :: :ok

  @optional_callbacks [terminate: 1]
end
```

### DSL Integration

**Two usage patterns:**

**1. Inline filters (per-actuator)**

```elixir
actuator :wrist, {BB.Servo.Robotis, controller: :u2d2, id: 3} do
  filter {BB.MotionFilter.Smooth, method: :exponential, alpha: 0.8}
  filter {BB.MotionFilter.LimitVelocity, max: ~u[30 degrees per second]}
end
```

**2. Reusable pipelines**

```elixir
motion_pipeline :slow_and_smooth do
  filter {BB.MotionFilter.Smooth, method: :exponential, alpha: 0.8}
  filter {BB.MotionFilter.LimitVelocity, max: ~u[30 degrees per second]}
  filter {BB.MotionFilter.LimitAcceleration, max: ~u[50 degrees per second squared]}
end

motion_pipeline :aggressive do
  filter {BB.MotionFilter.LimitVelocity, max: ~u[180 degrees per second]}
end

actuator :shoulder, {BB.Servo.Robotis, controller: :u2d2, id: 1} do
  pipe_through [:slow_and_smooth]
end

actuator :gripper, {BB.Servo.Feetech, controller: :uart, id: 4} do
  pipe_through [:aggressive]
end
```

**Constraint:** An actuator may use **either** inline filters **or** `pipe_through`, but not both. This prevents confusion about execution order and makes the DSL easier to reason about.

```elixir
# ❌ Invalid - mixing inline and pipeline
actuator :bad, {MyDriver, id: 1} do
  filter {BB.MotionFilter.Smooth, alpha: 0.5}
  pipe_through [:gentle]  # Error: can't mix inline and pipe_through
end
```

### Pipeline Execution

Filters execute in declaration order:

```
Incoming command
  ↓
Filter 1 (e.g., deadband - skip if too small)
  ↓ (or :skip → abort pipeline)
Filter 2 (e.g., smooth - exponential moving average)
  ↓
Filter 3 (e.g., limit velocity - clamp to max)
  ↓
Actuator driver
```

If any filter returns `:skip`, the pipeline aborts and the command is dropped.

### Integration with Actuator.Server

Filters are managed by `BB.Actuator.Server` as part of command processing:

```elixir
defmodule BB.Actuator.Server do
  # During init, build filter pipeline from DSL
  defp init_filters(dsl_config) do
    filters =
      case dsl_config do
        %{pipe_through: pipelines} ->
          Enum.flat_map(pipelines, &resolve_pipeline/1)

        %{filters: inline_filters} ->
          inline_filters

        _ ->
          []
      end

    Enum.map(filters, fn {mod, opts} ->
      {:ok, state} = mod.init(opts)
      %{module: mod, state: state}
    end)
  end

  # On command receipt, run through pipeline
  def handle_cast({:command, command}, state) do
    case run_filters(command, state.filters) do
      {:ok, filtered_command, updated_filters} ->
        # Send to driver
        apply_command(filtered_command, state)
        {:noreply, %{state | filters: updated_filters}}

      {:skip, updated_filters} ->
        # Command dropped by filter
        {:noreply, %{state | filters: updated_filters}}
    end
  end

  defp run_filters(command, []), do: {:ok, command, []}

  defp run_filters(command, [filter | rest]) do
    case filter.module.filter(command, filter.state) do
      {:skip, new_state} ->
        {:skip, [%{filter | state: new_state} | rest]}

      {new_command, new_state} ->
        updated_filter = %{filter | state: new_state}

        case run_filters(new_command, rest) do
          {:ok, final_command, updated_rest} ->
            {:ok, final_command, [updated_filter | updated_rest]}

          {:skip, updated_rest} ->
            {:skip, [updated_filter | updated_rest]}
        end
    end
  end
end
```

### Common Filter Implementations

**1. Exponential Smoothing**

```elixir
defmodule BB.MotionFilter.Smooth do
  @behaviour BB.MotionFilter

  defstruct [:method, :alpha, :previous]

  @impl BB.MotionFilter
  def init(opts) do
    method = Keyword.get(opts, :method, :exponential)
    alpha = Keyword.fetch!(opts, :alpha)
    {:ok, %__MODULE__{method: method, alpha: alpha, previous: nil}}
  end

  @impl BB.MotionFilter
  def filter({type, value, opts}, state) when state.previous == nil do
    # First sample - no smoothing
    {{type, value, opts}, %{state | previous: value}}
  end

  def filter({type, value, opts}, state) do
    smoothed =
      case state.method do
        :exponential ->
          state.alpha * value + (1 - state.alpha) * state.previous
      end

    {{type, smoothed, opts}, %{state | previous: smoothed}}
  end
end
```

**2. Velocity Limiting**

```elixir
defmodule BB.MotionFilter.LimitVelocity do
  @behaviour BB.MotionFilter

  defstruct [:max_velocity]

  @impl BB.MotionFilter
  def init(opts) do
    max_vel = Keyword.fetch!(opts, :max)
    {:ok, %__MODULE__{max_velocity: max_vel}}
  end

  @impl BB.MotionFilter
  def filter({:velocity, vel, opts}, state) do
    clamped = clamp(vel, -state.max_velocity, state.max_velocity)
    {{:velocity, clamped, opts}, state}
  end

  def filter(command, state) do
    # Pass through non-velocity commands
    {command, state}
  end

  defp clamp(value, min, max) do
    value |> max(min) |> min(max)
  end
end
```

**3. Acceleration Limiting**

```elixir
defmodule BB.MotionFilter.LimitAcceleration do
  @behaviour BB.MotionFilter

  defstruct [:max_accel, :prev_vel, :prev_time]

  @impl BB.MotionFilter
  def init(opts) do
    max_accel = Keyword.fetch!(opts, :max)
    {:ok, %__MODULE__{max_accel: max_accel, prev_vel: nil, prev_time: nil}}
  end

  @impl BB.MotionFilter
  def filter({:velocity, vel, opts}, %{prev_vel: nil} = state) do
    # First sample
    {{:velocity, vel, opts}, %{state | prev_vel: vel, prev_time: now()}}
  end

  def filter({:velocity, vel, opts}, state) do
    dt = now() - state.prev_time
    max_delta = state.max_accel * dt

    # Limit change in velocity
    delta = vel - state.prev_vel
    clamped_delta = clamp(delta, -max_delta, max_delta)
    new_vel = state.prev_vel + clamped_delta

    {{:velocity, new_vel, opts},
     %{state | prev_vel: new_vel, prev_time: now()}}
  end

  def filter(command, state) do
    {command, state}
  end

  defp now, do: System.monotonic_time(:millisecond)
  defp clamp(value, min, max), do: value |> max(min) |> min(max)
end
```

**4. Deadband (Ignore Small Changes)**

```elixir
defmodule BB.MotionFilter.Deadband do
  @behaviour BB.MotionFilter

  defstruct [:threshold, :previous]

  @impl BB.MotionFilter
  def init(opts) do
    threshold = Keyword.fetch!(opts, :threshold)
    {:ok, %__MODULE__{threshold: threshold, previous: nil}}
  end

  @impl BB.MotionFilter
  def filter({type, value, opts}, %{previous: nil} = state) do
    {{type, value, opts}, %{state | previous: value}}
  end

  def filter({type, value, opts}, state) do
    if abs(value - state.previous) < state.threshold do
      {:skip, state}
    else
      {{type, value, opts}, %{state | previous: value}}
    end
  end
end
```

**5. Rate Limiting (Debounce)**

```elixir
defmodule BB.MotionFilter.RateLimit do
  @behaviour BB.MotionFilter

  defstruct [:min_interval_ms, :last_time]

  @impl BB.MotionFilter
  def init(opts) do
    interval = Keyword.fetch!(opts, :min_interval_ms)
    {:ok, %__MODULE__{min_interval_ms: interval, last_time: 0}}
  end

  @impl BB.MotionFilter
  def filter(command, state) do
    now = System.monotonic_time(:millisecond)

    if now - state.last_time >= state.min_interval_ms do
      {command, %{state | last_time: now}}
    else
      {:skip, state}
    end
  end
end
```

### Telemetry

Filters emit telemetry for observability:

```elixir
:telemetry.execute(
  [:bb, :motion_filter, :applied],
  %{duration: duration},
  %{
    robot: robot,
    actuator: actuator_name,
    filter: filter_module,
    command_type: command_type
  }
)

:telemetry.execute(
  [:bb, :motion_filter, :skipped],
  %{},
  %{
    robot: robot,
    actuator: actuator_name,
    filter: filter_module,
    reason: :deadband  # or :rate_limit, etc.
  }
)
```

---

## Package Structure

Motion filters are part of **bb core** (not a separate package) because they're fundamental infrastructure that other packages will depend on.

```
lib/bb/
├── motion_filter.ex                  # Behaviour definition
├── motion_filter/
│   ├── smooth.ex                     # Exponential smoothing
│   ├── limit_velocity.ex             # Velocity clamping
│   ├── limit_acceleration.ex         # Acceleration clamping
│   ├── deadband.ex                   # Ignore small deltas
│   └── rate_limit.ex                 # Debounce
├── dsl/
│   ├── motion_pipeline.ex            # DSL entity for pipelines
│   └── actuator.ex                   # Extended with filter/pipe_through
└── actuator/
    └── server.ex                     # Extended with filter execution
```

---

## Acceptance Criteria

### Must Have

- [ ] `BB.MotionFilter` behaviour with `init/1` and `filter/2` callbacks
- [ ] DSL support for inline filters: `filter {Module, opts}`
- [ ] DSL support for reusable pipelines: `motion_pipeline :name do ... end`
- [ ] DSL support for pipeline references: `pipe_through [:pipeline_name]`
- [ ] Validation: Cannot mix inline filters and `pipe_through` on same actuator
- [ ] Filter execution in `BB.Actuator.Server` command processing
- [ ] State isolation: Each actuator gets its own filter state
- [ ] Pipeline short-circuiting: `:skip` aborts remaining filters
- [ ] `BB.MotionFilter.Smooth` implementation (exponential moving average)
- [ ] `BB.MotionFilter.LimitVelocity` implementation
- [ ] `BB.MotionFilter.Deadband` implementation
- [ ] Documentation with usage examples
- [ ] Tests for filter behaviour, DSL validation, and pipeline execution
- [ ] Telemetry events for filter application and skips

### Should Have

- [ ] `BB.MotionFilter.LimitAcceleration` implementation
- [ ] `BB.MotionFilter.RateLimit` implementation
- [ ] Multiple smoothing methods (exponential, moving average, Kalman)
- [ ] Filter introspection API (query active filters for an actuator)
- [ ] Runtime pipeline switching via parameters
- [ ] LiveView/Kino widgets to visualize filter effects
- [ ] Performance benchmarks (filter overhead per command)

### Won't Have

- [ ] Machine learning-based filters (separate package if needed)
- [ ] FFT/frequency domain filtering (too specialized)
- [ ] Visual filter configuration UI (could be separate tool)
- [ ] Automatic filter tuning (requires system identification)

---

## Open Questions

1. **Parameter support in pipelines:** Can pipeline names be parameters? E.g., `pipe_through [param([:motion, :active_pipeline])]` for runtime switching?

2. **Filter options as parameters:** Should filter options support parameter references? E.g., `filter {BB.MotionFilter.LimitVelocity, max: param([:safety, :max_vel])}`

3. **Conditional filters:** Should filters be able to enable/disable based on robot state? Or is that a job for controllers?

4. **Multi-dimensional smoothing:** For cartesian commands (transform matrices), how should smoothing work? Per-component? SE(3) interpolation?

5. **Filter composition helpers:** Should there be macros or helpers for common filter combinations? E.g., `use_safe_teleop_filters()`

6. **Backwards compatibility:** This adds new DSL syntax. Should old robots continue to work? (Yes - filters are optional)

7. **Filter reordering:** Can users override the order of filters in a pipeline when applying it? Or is order fixed at definition?

---

## References

- [Control Systems Engineering](https://www.worldcat.org/title/control-systems-engineering/oclc/1088936603) — Feedback control theory
- [ROS control_toolbox](http://wiki.ros.org/control_toolbox) — Similar filtering patterns
- [Kalman filtering](https://www.kalmanfilter.net/) — Advanced smoothing techniques
- [Digital signal processing](https://dspguide.com/) — Filter design principles
- [Phoenix Router pipelines](https://hexdocs.pm/phoenix/routing.html#pipelines) — Inspiration for DSL pattern

---

## Impact on Teleop Proposal

This proposal significantly simplifies `bb_teleop`:

**Before (teleop handles smoothing):**
- Teleop needs smoothing/ramping logic
- Teleop needs safety limiting
- Configuration is teleop-specific
- Doesn't apply to non-teleop commands

**After (filters handle smoothing):**
- Teleop just publishes raw commands
- Filters apply uniformly to all command sources
- Configuration is declarative in robot DSL
- Teleop becomes pure input translation

**Revised bb_teleop scope:**
- Input device abstraction only
- Publish standard message types
- No smoothing, no safety logic
- Just: `Device → Message → PubSub`

This makes teleop implementations simpler and more reusable.
