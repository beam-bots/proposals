<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal 0018: BB.Estimator

**Status:** Implemented
**Author:** James Harton
**Created:** 2026-05-26

---

## Summary

A unified state-estimation abstraction in `bb` core for sensor fusion. `BB.Estimator` is a behaviour (parallel to `BB.Sensor`/`BB.Actuator`/`BB.Controller`) for processes that consume one or more input streams and publish derived state. The same contract covers within-sensor fusion (AHRS combining gyro and accelerometer from one IMU into orientation) and cross-sensor fusion (combining multiple sensors of mixed modality into a richer state estimate). An `estimator` DSL entity nests inside `sensor` (single-input form, frame inherited) or `link` (cross-sensor form, frame = link). New `BB.Math.Covariance3`/`Covariance6` types and optional covariance fields on existing payloads make the contract sufficient for Kalman-family algorithms even though no Kalman implementation ships with this proposal. Reference algorithm implementations ship as sibling packages — `bb_ahrs` (Madgwick, Mahony, Complementary) lands alongside this proposal, replacing the existing `ahrs` library.

`BB.Perception.Perceptor` (Proposal 0017) is restructured as a specialisation of `BB.Estimator`: perception modules implement `BB.Estimator`, and the `perceptor` DSL entity is a specialised `estimator` entity that adds perception-specific concerns (sample store, FrameData, freshness gating).

---

## Motivation

State estimation is foundational for any robot that has to act on what its sensors tell it. Beam Bots currently has no shared abstraction for it:

- The existing `OpenLoopPositionEstimator` in `bb_servo_*` drivers is a per-package one-off, hand-written each time, with no consistent supervision or frame-handling story.
- Gus' `ahrs` library demonstrates the within-sensor fusion problem (gyro + accel → orientation) is solvable with three small algorithms, but has no idea about BB's topology, message envelope, units, or supervision.
- Proposal 0017 introduces `BB.Perception.Perceptor` to handle "consume streams, produce derived data," but the shape it converges on is much broader than perception. Without factoring, every non-perception fusion stage (AHRS, EKF, complementary filters) would either reimplement the same shape or wedge itself awkwardly into the perception layer.

### Two kinds of "fusion"

The term covers two distinct problems:

1. **Within-sensor fusion** — combining the different modalities of a single physical sensor. A 6-DOF IMU has a gyro (fast, drifts) and an accelerometer (drift-free gravity reference, noisy under linear acceleration). AHRS algorithms fuse these complementary modalities to produce drift-free orientation. Output is in the sensor's own frame.

2. **Cross-sensor fusion** — combining readings from different physical sensors into an estimate of some target frame's state. Two co-located IMUs averaged for redundancy. An IMU + wheel odometry running through an EKF for 2D base pose. An IMU + GPS for global pose. Output is in a chosen target frame; inputs must be transformed from their source frames before fusing.

Both problems share a structural shape (declared inputs, declared outputs, supervised process, telemetry, health transitions) but differ in their reference-frame story and number of inputs. One behaviour covers both; the DSL distinguishes the two cases.

### Why in `bb` core, not a sibling package

Following the existing pattern:

- `BB.Sensor`, `BB.Actuator`, `BB.Controller` behaviours all live in `bb` core; concrete implementations live in sibling driver packages (`bb_servo_*`, `bb_sensor_*`).
- `BB.IK.Solver` behaviour conceptually does the same — concrete solvers live in `bb_ik_dls`, `bb_ik_fabrik`.

`BB.Estimator` follows that shape. The behaviour, the DSL entity, the framework wrapper, the new math types, and the new message payloads all live in core. Concrete algorithm implementations live in sibling packages (`bb_ahrs` to start; future `bb_estimator_ekf`, etc.).

The behaviour has no new dependencies. Nx is already a core dep, used for kinematics; the new `Covariance3`/`Covariance6` types and any future Kalman-family algorithms will use it without further additions to core.

### Why not extend `BB.Controller`?

`BB.Controller` is "any process that runs alongside the robot." Estimators have a more specific contract — declared inputs from named source paths, frame inheritance from topology placement, hysteresis-debounced health transitions, command invocation on transitions, automatic dt computation, frame-of-reference handling. The contract is too specific to overload onto Controller without weakening Controller's general-purpose role.

### What this proposal does NOT cover

Out of scope:

- Concrete EKF/UKF reference implementations. The contract supports them; concrete algorithms ship as separate sibling packages once a use case lands.
- Robot-level estimators (world-frame localisation, SLAM-shaped state estimation). The link-nested form covers the common cases; world-frame estimation is a separate concern with its own design questions.
- Magnetometer / 9-DOF MARG support in AHRS. Adds in a follow-up once the first magnetometer sensor lands.
- Cross-node estimation. Local-only in v1.
- Replay-based or batched state estimation. Online only in v1.

---

## Design

### Reference frames

The single most important property of the estimator design: **the output frame is determined by where the estimator sits in the topology**, not declared separately.

#### Single-sensor estimator

Nested inside a `sensor`. The estimator's output is in the sensor's frame. Frame transforms are not needed — the algorithm receives raw sensor samples already in the frame it publishes to.

```elixir
link :base_link do
  sensor :imu, BB.Sensor.Bmi232, ... do
    estimator :orientation, {BB.Ahrs.Madgwick, beta: 0.1}
  end
end
```

The published `Imu` message's `frame_id` is the sensor's frame. Conversion to `base_link` orientation (via the static sensor-mounting transform) is a consumer concern.

#### Cross-sensor estimator

Nested inside a `link`. The estimator's output is in the link's frame. Input sources have their own frames; the framework provides each input's static transform-to-target-frame at init time, and the estimator applies them as part of its algorithm.

```elixir
link :base_link do
  sensor :imu, BB.Sensor.Bmi232, ...
  sensor :wheels, BB.Sensor.WheelOdom, ...

  estimator :pose, BB.Fusion.Complementary do
    input :imu, [:sensor, :base_link, :imu, :orientation], driver: true
    input :odom, [:sensor, :base_link, :wheels]
    sync_tolerance ~u(20 millisecond)
  end
end
```

The published `Pose`/`Odometry` message's `frame_id` is `base_link`.

#### Static vs dynamic transforms

For co-located sensors (same parent link), input-to-target transforms are static and known at compile time. The framework precomputes them and passes them in init opts.

For sensors on different links across a moving joint (rare in v1 use cases), transforms are dynamic — they depend on current joint angles. The estimator must query `BB.Robot.Kinematics` (existing module used by FK) to obtain them at message-handling time. The framework does not pre-resolve dynamic transforms.

The compile-time verifier rejects cross-sensor estimators whose inputs span moving joints unless the algorithm module declares it handles dynamic transforms (via a `requires_dynamic_transforms?/0` opt-in on the module).

### `BB.Estimator` behaviour

```elixir
defmodule BB.Estimator do
  @callback init(opts :: keyword()) ::
              {:ok, state :: term()}
              | {:stop, reason :: term()}

  @callback handle_input(
              input :: BB.Message.t() | %{atom() => BB.Message.t()},
              state :: term()
            ) ::
              {:reply, [{atom(), BB.Message.t()}], state :: term()}
              | {:noreply, state :: term()}
              | {:stop, reason :: term(), state :: term()}

  @callback handle_info(msg :: term(), state :: term()) ::
              {:reply, [{atom(), BB.Message.t()}], state :: term()}
              | {:noreply, state :: term()}
              | {:stop, reason :: term(), state :: term()}

  @callback handle_call(req :: term(), from :: GenServer.from(), state :: term()) ::
              {:reply, reply :: term(), [{atom(), BB.Message.t()}], state :: term()}
              | {:reply, reply :: term(), state :: term()}
              | {:noreply, state :: term()}
              | {:stop, reason :: term(), reply :: term(), state :: term()}

  @callback handle_options(new_opts :: keyword(), state :: term()) ::
              {:ok, state :: term()} | {:stop, reason :: term()}

  @callback terminate(reason :: term(), state :: term()) :: :ok

  @callback options_schema() :: Spark.Options.t()

  @optional_callbacks [
    handle_info: 2,
    handle_call: 3,
    handle_options: 2,
    terminate: 2,
    options_schema: 0
  ]
end
```

The callback shape is intentionally identical to the perceptor contract proposed in 0017 — that was always going to converge, and committing to a single behaviour is the simplest way to express the relationship.

#### Reply shape

Estimators emit messages by returning `{:reply, [{output_name, %BB.Message{}}], state}` from any callback. Each `output_name` is an atom matching either:

- An `output :name` block declared on the estimator (multi-output estimators), in which case the message is published to the output's declared path; or
- The conventional `:out` atom for single-output estimators, in which case the message is published to the estimator's default output path.

Single-output is overwhelmingly the common case. Multi-output exists for symmetry with perceptors and for estimators that genuinely produce multiple kinds of derived state (e.g. a Kalman filter that emits both a pose estimate and a velocity estimate as separate payloads).

Returning an empty list emits nothing — useful for accumulators that consume many inputs before producing one output.

#### Init context

Framework-provided init context comes through `opts` under a reserved key:

```elixir
def init(opts) do
  # User config
  beta = Keyword.get(opts, :beta, 0.1)

  # Framework context (always present)
  %BB.Estimator.Context{
    robot: robot,
    target_frame: target,
    transforms: transforms_by_input_name,
    path: path
  } = Keyword.fetch!(opts, :__context__)

  {:ok, %{beta: beta, transforms: transforms_by_input_name, target: target}}
end
```

`transforms` is a map from input name (or the singleton input's source-frame atom for single-input estimators) to a `BB.Math.Transform` representing the static frame-to-target transform. Identity for inputs already in the target frame.

This is a slight wart — magic option key. The alternative is a second arg to `init/2`, but that diverges from `BB.Sensor`/`BB.Actuator`/`BB.Controller`. Worth weighing in open questions.

#### Output payload type checking

Models are trusted to construct correctly-typed payloads. `BB.Message.new/3` validates against the payload's schema at construction time. Downstream consumers (with `message_types:`) get type-matching at delivery. There's no static contract for "this estimator must emit Odometry only" — if the estimator publishes the wrong shape, subscribers reject it.

### `BB.Estimator.Server`

The framework wrapper. One per estimator. Responsibilities:

1. **Subscription.** Subscribes to declared input paths via `BB.PubSub.subscribe/3`.
2. **Dispatch.** For single-input estimators, calls `handle_input/2` with a `BB.Message.t()`. For multi-input, calls with `%{key => BB.Message.t()}` after gathering non-driver inputs.
3. **Multi-input fan-in.** For multi-input estimators, the driver input's arrivals trigger dispatch. Non-driver inputs are pulled from BB's pubsub last-known cache (or `BB.Perception.SampleStore.nearest/3` if a non-driver input is store-backed) keyed off the driver's `monotonic_time`. If any non-driver gap exceeds `sync_tolerance`, drop with telemetry `[:bb, :estimator, :dropped]` reason `:sync_miss`.
4. **dt tracking.** Records last-input `monotonic_time` and passes the delta to algorithms that want it (via a small helper, not via a separate callback).
5. **Output routing.** For each `{name, message}` in the reply, publishes to the corresponding topic via `BB.PubSub.publish/3` (or `BB.Perception.publish/3` for perceptor-specialised estimators with store-backed outputs).
6. **Health transitions.** Tracks consecutive overruns/successes; on threshold transitions, invokes the configured command (`on_degraded`/`on_lost`/`on_recovered`).
7. **`:lost` detection.** Uses GenServer's `{:noreply, state, timeout}` reply with `lost_after` as the timeout; if no input arrives within that window, `handle_info(:timeout, _)` fires the lost transition.
8. **Telemetry.** Emits counters and a latency event (see Telemetry section).

#### Why GenServer

Single timeout mechanism, well-understood supervision behaviour, fits the BB pattern (sensors, actuators, controllers are all `GenServer`-backed). No reason to roll a custom process loop.

### Estimator DSL section

A new DSL section is added by `bb` core. Two forms:

#### Single-sensor form — nested in `sensor`

```elixir
sensor :imu, BB.Sensor.Bmi232, bus: "i2c-1", address: 0x68 do
  estimator :orientation, {BB.Ahrs.Madgwick, beta: 0.1} do
    latency_budget ~u(20 millisecond)
    on_degraded :enter_degraded_mode
  end
end
```

- Frame: inherited from parent sensor.
- Input: implicit — the parent sensor's published messages.
- Output path: `[:sensor, link_name, sensor_name, estimator_name]`. Subscribers wanting "all data about this IMU" can subscribe to the sensor's subtree and receive both raw and estimated data.

#### Cross-sensor form — nested in `link`

```elixir
estimator :pose, BB.Fusion.Complementary do
  input :imu, [:sensor, :base_link, :imu, :orientation], driver: true
  input :odom, [:sensor, :base_link, :wheels]

  sync_tolerance ~u(20 millisecond)
  latency_budget ~u(50 millisecond)
  lost_after ~u(500 millisecond)
  recover_after 10

  on_degraded :enter_degraded_mode
  on_lost :emergency_stop
  on_recovered :resume_normal_operation
end
```

- Frame: parent link.
- Inputs: explicitly declared. Multi-input requires `driver: true` on exactly one input. Single-input form within `link` is also allowed for estimators that consume one sensor but publish to the link's frame rather than the sensor's.
- Output path: `[:estimator, link_name, estimator_name]`. The `:estimator` prefix distinguishes link-level estimator outputs from sensor outputs in subscriptions.

#### Estimator entity options

```elixir
estimator :name, ChildSpec do
  # Inputs (cross-sensor form only)
  input path                                    # single-input shorthand
  input :key, path, driver: true | false        # multi-input

  # Outputs (optional; default is implicit :out)
  output :name, path: [...] (optional override)

  # Timing
  latency_budget   ~u(20 millisecond)
  sync_tolerance   ~u(10 millisecond)
  lost_after       ~u(500 millisecond)
  recover_after    10

  # Transition commands
  on_degraded      :command_name
  on_lost          :command_name
  on_recovered     :command_name
end
```

Where `ChildSpec` is `Module` or `{Module, opts}`. The module must implement `BB.Estimator`. Options are validated against the module's `options_schema/0` at compile time (literal values) and runtime (parameter references).

#### Compile-time verifier

`BB.Dsl.Verifiers.ValidateEstimators`:

- Each `input` path resolves to an existing sensor, sensor-nested estimator, or link-nested estimator (the perception proposal's path resolution extends naturally).
- Single vs. multi-input form is consistent across all `input` blocks on a given estimator.
- `driver: true` set on at most one input; required for multi-input.
- Estimator module exists and implements `BB.Estimator`.
- Configured `on_degraded`/`on_lost`/`on_recovered` command names exist in the robot's `commands` section.
- For cross-sensor estimators with inputs from sensors on different links, either all the links share a static (i.e. fixed-joint) chain, or the algorithm module opts in to dynamic transforms via `requires_dynamic_transforms?/0`.
- `latency_budget`, `lost_after`, `sync_tolerance` are positive duration units when present.
- `recover_after` is a positive integer.
- No cycles in the estimator dependency graph (an estimator consuming another estimator's output, etc.).

Path resolution and static-transform precomputation are pre-cached via `Spark.Dsl.Transformer.persist/3` so the verifier doesn't re-walk the topology per estimator.

### Health transitions

No structured health payload, no monitor process, no health pubsub channel. Instead, three configurable commands fire on hysteresis-debounced transitions:

- `on_degraded` — fires when the estimator transitions from `:healthy` to `:degraded`. Triggers: latency-budget overrun, sync miss, stale input, algorithm-reported disagreement.
- `on_lost` — fires when the estimator transitions to `:lost` (no input within `lost_after`). Detected via GenServer timeout.
- `on_recovered` — fires when the estimator transitions back to `:healthy` from any non-healthy state, after `recover_after` consecutive in-budget completions.

#### State model

Three states (no `:failed` — process death is an OTP/supervisor concern, not a health state):

```
:healthy  ⇄  :degraded
   │            │
   └─→ :lost ←──┘
```

Recovery always goes via `:degraded` first (one successful input transitions `:lost → :degraded`; `recover_after` consecutive successes then transition `:degraded → :healthy`).

#### Command invocation

Transitions dispatch the configured command via the robot's existing command system. The command's `handle_command/3` receives metadata in its args:

```elixir
%{
  estimator: :pose,
  reason: :latency_overrun | :stale_input | :sync_miss | :lost | :recovered,
  source_path: [atom],          # for input-related transitions
  previous_state: :healthy | :degraded | :lost,
  new_state: :healthy | :degraded | :lost
}
```

If a transition fires for an estimator that doesn't have the corresponding command configured (e.g. `on_lost` not set), the transition still happens internally — telemetry still emits — but no command is invoked.

#### Why this shape

Two reasons:

1. **Developer-defined policy.** What to do when perception degrades is a robot-specific decision. Some robots should stop. Some should switch to a slower control loop. Some should emit a status message and continue. Hard-coding any of these into the framework is wrong; surfacing the transition as a command lets the developer encode their policy using existing BB primitives.

2. **Existing state-machine integration for free.** Commands already integrate with the robot's state machine via `allowed_states`. Want certain operations blocked during degraded perception? Have `on_degraded` invoke a command that transitions the state machine to a `:degraded` state, and configure other commands' `allowed_states` accordingly. No new mechanism required.

### `BB.Math.Covariance3` and `BB.Math.Covariance6`

Typed wrappers around Nx tensors, following the pattern of `Vec3`/`Quaternion`/`Transform`:

```elixir
defmodule BB.Math.Covariance3 do
  defstruct [:tensor]

  @type t :: %__MODULE__{tensor: Nx.Tensor.t()}

  @spec new(Nx.Tensor.t()) :: t()
  def new(tensor)  # asserts shape {3, 3}

  @spec diagonal([float()] | Nx.Tensor.t()) :: t()
  def diagonal(diag)

  @spec from_tensor(Nx.Tensor.t()) :: t()
  def from_tensor(tensor)

  @spec to_tensor(t()) :: Nx.Tensor.t()
  def to_tensor(%__MODULE__{tensor: t}), do: t
end

defmodule BB.Math.Covariance6 do
  # As above, shape {6, 6}
end
```

Same `from_tensor`/`to_tensor` boundary as `Transform`. Algorithms can operate on the raw Nx tensors internally; payloads carry typed wrappers.

### Payload changes

#### `BB.Message.Sensor.Imu` — additive

Adds three optional fields, defaulting to `nil`:

- `orientation_covariance :: BB.Math.Covariance3.t() | nil`
- `angular_velocity_covariance :: BB.Math.Covariance3.t() | nil`
- `linear_acceleration_covariance :: BB.Math.Covariance3.t() | nil`

Drivers with known noise characteristics fill them in; ones that don't, don't. Algorithms that require them check for `nil` and either fall back to a configured default or raise. Backwards-compatible with all existing `Imu` consumers.

#### `BB.Message.Estimator.Pose` — new

```elixir
defmodule BB.Message.Estimator.Pose do
  defstruct [:transform, :covariance]

  @type t :: %__MODULE__{
          transform: BB.Math.Transform.t(),
          covariance: BB.Math.Covariance6.t() | nil
        }
end
```

`transform` is the SE(3) pose of the estimator's frame in the world (or a declared reference frame). `covariance` is optional.

#### `BB.Message.Estimator.Odometry` — new

```elixir
defmodule BB.Message.Estimator.Odometry do
  defstruct [:pose, :twist, :pose_covariance, :twist_covariance]

  @type t :: %__MODULE__{
          pose: BB.Math.Transform.t(),
          twist: %{linear: BB.Math.Vec3.t(), angular: BB.Math.Vec3.t()},
          pose_covariance: BB.Math.Covariance6.t() | nil,
          twist_covariance: BB.Math.Covariance6.t() | nil
        }
end
```

Mirrors the ROS `nav_msgs/Odometry` shape — pose + twist with separate covariances. The canonical output for IMU-plus-odometry EKFs.

### Chaining

Estimators consume other estimators' outputs by pointing `input` at their paths:

```elixir
link :base_link do
  sensor :imu, BB.Sensor.Bmi232, ... do
    estimator :orientation, BB.Ahrs.Madgwick
  end

  estimator :pose, BB.Fusion.Complementary do
    input :orientation, [:sensor, :base_link, :imu, :orientation], driver: true
    input :odom, [:sensor, :base_link, :wheels]
  end
end
```

The verifier walks the dependency graph and rejects cycles.

### Telemetry

- `[:bb, :estimator, :input]` — counter per input delivered.
  - Measurements: `%{count: 1}`
  - Metadata: `%{robot: atom, estimator: atom, source_path: [atom]}`

- `[:bb, :estimator, :output]` — counter per emitted message.
  - Measurements: `%{count: 1}`
  - Metadata: `%{robot: atom, estimator: atom, output: atom, payload_module: module}`

- `[:bb, :estimator, :latency]` — duration from driver input `monotonic_time` to emission.
  - Measurements: `%{duration: native_time}`
  - Metadata: `%{robot: atom, estimator: atom, output: atom}`

- `[:bb, :estimator, :dropped]` — counter per drop.
  - Measurements: `%{count: 1}`
  - Metadata: `%{robot: atom, estimator: atom, source_path: [atom], reason: :stale_input | :sync_miss}`

- `[:bb, :estimator, :transition]` — counter per health transition.
  - Measurements: `%{count: 1}`
  - Metadata: `%{robot: atom, estimator: atom, from: atom, to: atom, reason: atom}`

### Error types

```elixir
defmodule BB.Error.Estimator do
  defmodule StaleInput do
    use BB.Error, class: :state, fields: [:path, :age_ms, :budget_ms]
    defimpl BB.Error.Severity, do: (def severity(_), do: :warning)
  end

  defmodule SyncMiss do
    use BB.Error, class: :state,
      fields: [:driver_path, :input_path, :gap_ms, :tolerance_ms]
    defimpl BB.Error.Severity, do: (def severity(_), do: :warning)
  end

  defmodule MissingCovariance do
    use BB.Error, class: :invalid, fields: [:estimator, :field]
    defimpl BB.Error.Severity, do: (def severity(_), do: :error)
  end
end
```

### Relationship to Proposal 0017 (bb_perception)

This proposal restructures 0017's `BB.Perception.Perceptor` as a specialisation of `BB.Estimator`:

- Perception modules implement `BB.Estimator` (not a separate behaviour).
- The `perceptor` DSL entity is a specialised `estimator` entity that adds perception-specific options: `freshness`, store-backed outputs (via the SampleStore), `FrameData`-aware output routing.
- `BB.Perception.Perceptor.Server` is `BB.Estimator.Server` plus the sample-store/FrameData/perception-specific pieces.
- `BB.Perception.Health` (the structured payload + monitor process) is dropped in favour of the same `on_degraded`/`on_lost`/`on_recovered` command-invocation mechanism.

0017 will be updated to reflect this — the perceptor behaviour section becomes a pointer to this proposal, the health observable section is removed, and the `degraded_state` section-level option is replaced by per-perceptor transition commands.

---

## Package structure

```
bb (core) — new additions:
├── lib/bb/
│   ├── estimator.ex                          # behaviour
│   ├── estimator/
│   │   ├── server.ex                         # framework wrapper
│   │   ├── supervisor.ex
│   │   └── context.ex                        # init context struct
│   ├── dsl/
│   │   ├── estimator.ex                      # entity
│   │   ├── input.ex
│   │   ├── output.ex
│   │   ├── transformers/
│   │   │   └── resolve_estimator_transforms.ex
│   │   └── verifiers/
│   │       └── validate_estimators.ex
│   ├── math/
│   │   ├── covariance3.ex
│   │   └── covariance6.ex
│   ├── message/
│   │   ├── sensor/
│   │   │   └── imu.ex                        # gains optional covariance fields
│   │   └── estimator/
│   │       ├── pose.ex
│   │       └── odometry.ex
│   └── error/
│       └── estimator/                        # error types
```

```
bb_ahrs — new sibling package:
├── lib/
│   └── bb/
│       └── ahrs/
│           ├── madgwick.ex
│           ├── mahony.ex
│           └── complementary.ex
├── test/
├── mix.exs
└── README.md
```

### Dependencies for `bb_ahrs`

```elixir
defp deps do
  [
    {:bb, bb_dep("~> 0.13")},
    {:nx, "~> 0.7"}
  ]
end
```

`bb_ahrs` algorithms are plain Elixir for the online path (per-update cost is too small to benefit from Nx dispatch). Future `defn` variants for batched replay/parameter-sweep use cases can be added without breaking the contract.

### Companion package landscape

| Package | Role | Pulls in |
|---|---|---|
| `bb_ahrs` | Madgwick/Mahony/Complementary for within-sensor IMU fusion | `nx` |
| `bb_estimator_ekf` (future) | EKF reference implementations | `nx` |
| `bb_estimator_ukf` (future) | UKF reference implementations | `nx` |
| `bb_sensor_bmi232` | BMI232 IMU driver — publishes raw accel/gyro | hardware |
| `bb_sensor_lis3dh` | LIS3DH accelerometer driver — publishes raw accel | hardware |
| `bb_perception` (0017, revised) | Perception extends `BB.Estimator` | `bb`, perception adapters |

---

## User experience

### Within-sensor fusion: single IMU

```elixir
defmodule MyRobot do
  use BB

  topology do
    link :base_link do
      sensor :imu, BB.Sensor.Bmi232, bus: "i2c-1", address: 0x68 do
        estimator :orientation, {BB.Ahrs.Madgwick, beta: 0.1}
      end
    end
  end
end
```

A consumer subscribes to fused orientation:

```elixir
BB.subscribe(MyRobot, [:sensor, :base_link, :imu, :orientation])
# receives {:bb, path, %BB.Message{payload: %BB.Message.Sensor.Imu{
#   orientation: %BB.Math.Quaternion{...},
#   angular_velocity: %BB.Math.Vec3{...},
#   linear_acceleration: %BB.Math.Vec3{...}
# }}}
```

### Choosing the algorithm

The `algorithm` option (passed as the second tuple element) determines the implementation. Swapping is a one-line change with no other code touched:

```elixir
estimator :orientation, {BB.Ahrs.Mahony, kp: 2.0, ki: 0.05}
# or
estimator :orientation, {BB.Ahrs.Complementary, alpha: 0.98}
```

### Accel-only tilt

A LIS3DH publishes raw acceleration only. There's no AHRS-style fusion possible (no gyro), but tilt-from-gravity is:

```elixir
sensor :tilt_sensor, BB.Sensor.Lis3dh, bus: "i2c-1", address: 0x18 do
  estimator :tilt, {BB.Ahrs.GravityTilt, smoothing: 0.9}
end
```

`BB.Ahrs.GravityTilt` is a hypothetical roll/pitch estimator from the gravity vector. The framework treats it identically to any other estimator — it's just a different algorithm module.

### Cross-sensor fusion: redundant IMUs

Two co-located IMUs averaged for noise reduction:

```elixir
link :base_link do
  sensor :imu_a, BB.Sensor.Bmi232, bus: "i2c-1", address: 0x68 do
    estimator :orientation, BB.Ahrs.Madgwick
  end
  sensor :imu_b, BB.Sensor.Bmi232, bus: "i2c-1", address: 0x69 do
    estimator :orientation, BB.Ahrs.Madgwick
  end

  estimator :orientation, BB.Fusion.WeightedAverage do
    input :a, [:sensor, :base_link, :imu_a, :orientation]
    input :b, [:sensor, :base_link, :imu_b, :orientation]
    # No driver: WeightedAverage emits on whichever input arrives latest
  end
end
```

### Cross-sensor fusion: IMU + wheel odometry (EKF-shaped)

The contract that future EKF implementations will use:

```elixir
estimator :pose, {BB.Estimator.Ekf,
  initial_pose: BB.Math.Transform.identity(),
  process_noise: BB.Math.Covariance6.diagonal([0.01, 0.01, 0.0, 0.0, 0.0, 0.001])
} do
  input :imu, [:sensor, :base_link, :imu, :orientation], driver: true
  input :odom, [:sensor, :base_link, :wheels]
  sync_tolerance ~u(20 millisecond)
  latency_budget ~u(50 millisecond)
end
```

No EKF reference implementation ships with this proposal — the contract above is what one *would* look like, and the verifier accepts it. The first concrete EKF lands in a separate proposal/package.

### Health-as-commands

```elixir
commands do
  command :degraded_perception do
    handler MyApp.Commands.SwitchToSlowMode
    allowed_states [:idle, :executing]
  end

  command :emergency_stop do
    handler MyApp.Commands.Stop
    allowed_states [:idle, :executing, :degraded]
  end

  command :resume_normal do
    handler MyApp.Commands.Resume
    allowed_states [:degraded]
  end
end

topology do
  link :base_link do
    sensor :imu, ... do
      estimator :orientation, BB.Ahrs.Madgwick do
        latency_budget ~u(20 millisecond)
        lost_after ~u(500 millisecond)
        on_degraded :degraded_perception
        on_lost :emergency_stop
        on_recovered :resume_normal
      end
    end
  end
end
```

### Chained estimators

```elixir
link :base_link do
  sensor :imu, BB.Sensor.Bmi232, ... do
    estimator :orientation, BB.Ahrs.Madgwick     # raw → orientation
  end

  estimator :pose, BB.Fusion.Complementary do    # orientation + odom → pose
    input :orient, [:sensor, :base_link, :imu, :orientation], driver: true
    input :odom, [:sensor, :base_link, :wheels]
  end

  estimator :smoothed_pose, BB.Fusion.LowPass do # pose → smoothed pose
    input [:estimator, :base_link, :pose]
  end
end
```

The verifier rejects cycles in this graph.

---

## Acceptance Criteria

### Must Have

#### Phase 1 — Estimator behaviour and DSL

- [ ] `BB.Estimator` behaviour with `init`, `handle_input`, `handle_info`, `handle_call`, `handle_options`, `terminate`, `options_schema` callbacks.
- [ ] `BB.Estimator.Server` framework wrapper handling subscription, single-vs-multi input dispatch, driver/sync_tolerance multi-input fan-in, dt tracking, output routing, telemetry.
- [ ] `BB.Estimator.Context` struct delivered to estimator `init/1` via the `__context__` opt.
- [ ] `estimator` DSL entity, nestable inside `sensor` (single-input) or `link` (single or multi-input).
- [ ] `input :key, path, driver: true|false` form for multi-input estimators.
- [ ] `output :name, ...` form for multi-output estimators (default implicit `:out`).
- [ ] Compile-time verifier: path resolution, single/multi consistency, driver constraint, cycle detection, command name resolution, transform feasibility.
- [ ] Single-output estimators publish to `[:sensor, link, sensor, estimator]` (sensor-nested) or `[:estimator, link, estimator]` (link-nested).
- [ ] Static frame-to-target-frame transforms precomputed and delivered via init context.

#### Phase 2 — Health transitions

- [ ] `latency_budget`, `lost_after`, `sync_tolerance`, `recover_after` options on the estimator entity.
- [ ] `on_degraded`, `on_lost`, `on_recovered` command-name options on the estimator entity; verifier checks the command exists.
- [ ] Hysteresis-debounced state transitions: `:healthy ⇄ :degraded`, both → `:lost` via GenServer timeout, `:lost → :degraded` on next input.
- [ ] Configured commands invoked on transitions with `{estimator, reason, source_path, previous_state, new_state}` args.
- [ ] Telemetry events: `:input`, `:output`, `:latency`, `:dropped`, `:transition`.

#### Phase 3 — Math types and payloads

- [ ] `BB.Math.Covariance3` and `BB.Math.Covariance6` with `new/1`, `diagonal/1`, `from_tensor/1`, `to_tensor/1`.
- [ ] Optional `*_covariance` fields on `BB.Message.Sensor.Imu`.
- [ ] `BB.Message.Estimator.Pose` payload.
- [ ] `BB.Message.Estimator.Odometry` payload.
- [ ] `BB.Error.Estimator.{StaleInput, SyncMiss, MissingCovariance}` error types.

#### Phase 4 — `bb_ahrs` sibling package

- [ ] `bb_ahrs` package created, replacing Gus' `ahrs` library.
- [ ] `BB.Ahrs.Madgwick`, `BB.Ahrs.Mahony`, `BB.Ahrs.Complementary` implementing `BB.Estimator`.
- [ ] Algorithms use `BB.Math.Quaternion`/`Vec3` (not their own quaternion type).
- [ ] Algorithms accept the standard estimator init context (target_frame, transforms).
- [ ] Linear acceleration rejection (existing `accel_threshold` mechanism preserved).
- [ ] Test suite: tilt sequences, drift behaviour, accel-outlier rejection, dt sensitivity.

### Should Have

- [ ] Chaining: estimator-consumes-estimator-output supported by path resolution.
- [ ] Documentation: README, tutorial covering within-sensor AHRS, tutorial covering cross-sensor fusion, tutorial covering health-via-commands.
- [ ] `mix.usage_rules` documentation.

### Won't Have (deferred)

- [ ] Concrete EKF/UKF reference implementations. The contract supports them; concrete algorithms ship as separate proposals/packages.
- [ ] Robot-level estimators (`estimators do … end` at the robot level, for world-frame state). The link-nested form covers v1 use cases.
- [ ] Magnetometer / 9-DOF MARG support in AHRS. Lands when the first magnetometer sensor lands.
- [ ] Dynamic transforms across moving joints. Algorithms can opt in via `requires_dynamic_transforms?/0`; framework provides no convenience layer in v1.
- [ ] Cross-node estimation.
- [ ] Replay/batched estimation. The behaviour is online-only in v1.
- [ ] Hot reload of estimator parameters without restart (parameters can change at runtime via `handle_options/2` if the algorithm implements it).

---

## Open Questions

1. **Init context delivery shape.** The proposal uses a magic `:__context__` opt key. The alternative is a second arg to `init/2`, diverging from `BB.Sensor`/`BB.Actuator`/`BB.Controller`. Magic key wins on consistency; explicit arg wins on cleanliness. Worth deciding before implementation.

2. **Path namespace for cross-sensor estimators.** The proposal uses `[:estimator, link, name]` for link-nested estimators and `[:sensor, link, sensor, name]` for sensor-nested. The split distinguishes by topology placement, which is semantically clean but means subscribers wanting "all data for this link" need two subscriptions. Alternative: always use `[:sensor, link, ...]` and treat estimators as virtual sensors. The current split feels right but worth confirming.

3. **Output payload type discipline.** No static contract requires an estimator to emit a particular payload type. Should there be a `output_kinds/0` callback (analogous to perception's deliberately-omitted `input_kinds`) for tooling, or is runtime type-matching at the subscriber side sufficient?

4. **`recover_after` semantics for `:lost → :healthy`.** The state model has `:lost → :degraded → :healthy` (no direct lost-to-healthy transition). After recovery from `:lost`, do we require `recover_after` consecutive successes from scratch, or do we count earlier in-budget successes? Probably start-from-scratch for predictable behaviour, but worth being explicit.

5. **Command invocation when robot is in a state that disallows the command.** If `on_lost: :emergency_stop` fires but the robot is in a state where `:emergency_stop` isn't in `allowed_states`, what happens? Options: (a) silently log + telemetry, (b) raise/crash the estimator, (c) the command system handles it (currently the latter — invalid command attempts return errors). Probably (c), but worth documenting that estimators can fire transitions for commands the state machine will reject.

6. **Algorithm modules that need to know the wall-clock time vs monotonic time.** Most algorithms only care about dt, but some (e.g. magnetometer adapters using a world magnetic-field model) need geographic/wall-clock info. Not in scope for v1, but worth noting that algorithms can read `BB.Message.wall_time/1` directly from inputs.

7. **Algorithm package naming.** `bb_ahrs` is the obvious one. For future Kalman algorithms, `bb_estimator_ekf` (one package, many EKF variants) vs `bb_ekf_imu_odom` (one package per concrete EKF). The latter matches the existing servo-driver naming pattern more closely. Worth choosing before the first EKF lands.

---

## References

- [Proposal 0017: bb_perception](0017-bb-perception.md) — perception's perceptor becomes a specialised estimator. This proposal is a prerequisite for 0017's restructured form.
- [Gus' `ahrs` library](https://github.com/gworkman/ahrs) — informs the algorithm shapes; ported into `bb_ahrs`.
- [Madgwick AHRS paper](https://x-io.co.uk/downloads/madgwick_internal_report.pdf) — algorithm reference.
- [Mahony AHRS paper](https://hal.inria.fr/inria-00488376/document) — algorithm reference.
- [ROS sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html) — payload-type precedent for `Imu` covariance fields.
- [ROS nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) — payload-type precedent for `BB.Message.Estimator.Odometry`.
- [BB.Sensor](../../bb/lib/bb/sensor.ex) — behaviour pattern this proposal mirrors.
- [BB.Math.Transform](../../bb/lib/bb/math/transform.ex) — pattern for typed Nx wrappers.
- [Iterative Closest Point and Kalman Filtering for State Estimation](https://www.cs.unc.edu/~welch/kalman/) — Kalman filter reference for future EKF implementations.
