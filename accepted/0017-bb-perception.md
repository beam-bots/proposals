<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal 0017: bb_perception

**Status:** Draft
**Author:** James Harton
**Created:** 2026-05-24

---

## Summary

A perception interface package for Beam Bots that provides the common contract every camera, LIDAR, depth sensor, vision model adapter, and SLAM consumer agrees on. `bb_perception` ships the perception-shaped payload types (`Image`, `LaserScan`, `PointCloud`, `Detections`, etc.), an ETS-backed sample store for high-rate large-payload streams, the `BB.Vision.Model` behaviour, a `perceptor` DSL section for wiring sensor streams through inference, and a `BB.Perception.Health` observable for surfacing degradation. It is intentionally deps-light so driver packages (`bb_camera_*`, `bb_lidar_*`) and heavy inference adapter packages (`bb_vision_yolo`, `bb_vision_bumblebee`, `bb_vision_ortex`, `bb_vision_hailo`) can depend on a single contract without pulling in Nx, EXLA, Ortex, or Bumblebee transitively.

---

## Motivation

Beam Bots is a generalised robotics framework. Users range from "Raspberry Pi 5 with a HAILO module" to "workstation with the latest GPU." Perception is the area where this heterogeneity bites hardest:

- **Frame data is large.** A 1080p RGB frame is ~6MB; a 64-line LIDAR sweep is several MB; a depth map is comparable. Passing these through process mailboxes without care wastes memory and CPU.
- **Inference backends vary.** yolo_elixir wraps Ortex. Bumblebee wraps Nx.Serving. HAILO has its own NIF. Evision has a DNN module. None of these should be a hard dependency of a robot that just wants to publish a LIDAR scan.
- **Consumers vary.** A SLAM module wants keyframes pinned for loop closure. A visual-servoing controller wants the freshest detection within a latency budget. A recording tool wants every sample. A live dashboard wants the latest available.
- **Failure modes vary.** A camera disconnect, a model load failure, a NIF segfault, and a sample-drop overrun are all "perception is degraded" but mean very different things to downstream behaviours.

Without a shared abstraction, every driver invents its own message types, every consumer reimplements freshness gating, every model adapter is a one-off, and the framework can't make any guarantees about resilience.

### Why an extension, not core

Most robots don't have cameras or LIDAR. A servo-driven arm with encoder feedback needs none of this. Putting perception in core would:

- Add `Nx`-shaped dependencies (eventually) to every BB deployment.
- Bloat compile times and deploy artifacts for users who never publish a frame.
- Couple core's release cadence to perception's, slowing both.

The existing servo-driver pattern (one core behaviour, multiple sibling driver packages) is the proven shape; perception follows the same pattern with `bb_perception` playing the role of the contract layer.

### Why one interface package, not many

Drivers and consumers both need to recognise perception payload types. If each driver defined its own `Image` struct, consumers couldn't write driver-agnostic code. If each model adapter defined its own behaviour, drivers couldn't be swapped behind perceptors. `bb_perception` is small, deps-light, and the single agreed contract.

### What this proposal does NOT cover

Out of scope:

- Specific camera/LIDAR drivers (`bb_camera_v4l2`, `bb_lidar_rplidar`, etc.) — sibling packages, separate proposals.
- Specific inference adapters (`bb_vision_yolo`, `bb_vision_bumblebee`, etc.) — sibling packages, separate proposals.
- SLAM algorithms (`bb_slam_*`) — depend on this package, separate proposals.
- `bb_replay` (record/replay of sample streams) — separate proposal.
- Device-resident tensor handles (HAILO/CUDA zero-copy) — deferred to a future revision of this package.

---

## Design

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        bb_perception                            │
│                                                                 │
│  ┌────────────────┐  ┌──────────────────┐  ┌────────────────┐   │
│  │ Payload Types  │  │   SampleStore    │  │ Vision.Model   │   │
│  │  (structs)     │  │  (per-stream)    │  │  (behaviour)   │   │
│  └────────────────┘  └──────────────────┘  └────────────────┘   │
│                                                                 │
│  ┌────────────────┐  ┌──────────────────┐  ┌────────────────┐   │
│  │  perceptor DSL │  │ Perception.Health│  │ Perception.    │   │
│  │   (section)    │  │  (observable)    │  │ Required (mix) │   │
│  └────────────────┘  └──────────────────┘  └────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
        ↑                       ↑                       ↑
        │                       │                       │
   ┌────┴─────┐         ┌───────┴────────┐    ┌────────┴────────┐
   │ Drivers  │         │ Inference      │    │  Algorithms      │
   │ (cameras,│         │ Adapters       │    │  (SLAM, fusion,  │
   │  LIDAR)  │         │ (YOLO, Bumble) │    │   tracking)      │
   └──────────┘         └────────────────┘    └──────────────────┘
```

### Removal from core

`BB.Message.Sensor.Image` and `BB.Message.Sensor.LaserScan` currently live in `bb`. Nothing in core machinery consumes them. They were added in anticipation of perception work and are misplaced. This proposal moves both into `bb_perception` as part of the same release.

This is a breaking change for any external code that referenced these modules in `bb`, but since neither type is consumed by any current code path or sibling package, the migration cost is "rewrite one `alias` line." It will be flagged in `bb`'s changelog and called out in the `bb_perception` migration guide.

### Payload types

All under `BB.Perception.Message.*`, following the existing BB payload naming convention (`*State` for snapshots, naked-noun for single-purpose readings, `VerbObject` for events).

**Sensor readings:**

```elixir
BB.Perception.Message.Image       # 2D image (RGB, RGBA, mono, bayer)
BB.Perception.Message.PointCloud  # 3D point cloud (any number of points)
BB.Perception.Message.LaserScan   # planar laser scan (existing shape)
BB.Perception.Message.Depth       # depth map (per-pixel range)
BB.Perception.Message.Stereo      # synchronised left/right pair
BB.Perception.Message.IR          # thermal / IR image
```

**Inference outputs:**

```elixir
BB.Perception.Message.Detections  # [{bbox, class, confidence}, ...]
BB.Perception.Message.Masks       # segmentation masks per instance
BB.Perception.Message.Keypoints   # pose estimation keypoints per instance
BB.Perception.Message.DepthMap    # predicted depth (vs. measured Depth)
BB.Perception.Message.Embedding   # feature vector
BB.Perception.Message.Classification # class label(s) with confidence
```

Each is a `use BB.Message`-derived module with a Spark-validated schema. Shapes follow ROS `sensor_msgs` / `vision_msgs` precedent where applicable.

### SampleStore

A per-stream ETS-backed index for high-rate, large-payload streams. Reading and writing samples for fusion, replay seeking, keyframe pinning, and multi-rate sampling are all things pubsub cannot do well on its own; the store provides them as an additive index on the same data that pubsub already fans out.

#### Identity

A "stream" is addressed by `(robot, path)` where `path` is the same hierarchical path BB pubsub uses (e.g. `[:sensor, :base_link, :camera_front]`). Each store-backed stream gets a named ETS table owned by a supervised `BB.SampleStore.Server` process.

#### Storage shape

Samples are stored as `BB.Message` values directly — no parallel `BB.Sample` struct. The envelope already carries `monotonic_time`, `wall_time`, `node`, `frame_id`, `payload`, and `robot`. ETS table is `:ordered_set` on `monotonic_time` with `read_concurrency: true` and `write_concurrency: false`.

Refc binaries (camera frame data, point clouds) are shared by reference between ETS and any subscriber that received the message via pubsub — no copy.

#### Read API

```elixir
BB.SampleStore.latest(robot, path)
  #=> {:ok, %BB.Message{}} | :empty

BB.SampleStore.fetch(robot, path, id)
  #=> {:ok, %BB.Message{}} | :not_found

BB.SampleStore.nearest(robot, path, time_or_message)
  #=> {:ok, %BB.Message{}} | :empty

BB.SampleStore.before(robot, path, time_or_message)
  #=> {:ok, %BB.Message{}} | :empty

BB.SampleStore.after(robot, path, time_or_message)
  #=> {:ok, %BB.Message{}} | :empty

BB.SampleStore.range(robot, path, t_start, t_end)
  #=> [%BB.Message{}]

BB.SampleStore.since(robot, path, time_or_message)
  #=> [%BB.Message{}]
```

Time arguments polymorphically accept either an integer `monotonic_time` or a `BB.Message.t()` (whose `monotonic_time` is used). Lookups are O(log n) on an ordered_set. `nearest` ties break in favour of the earlier sample (causal, no future-data leakage).

#### Pinning

For SLAM keyframes and similar long-lived references:

```elixir
BB.SampleStore.pin(robot, path, id)     #=> :ok | :not_found
BB.SampleStore.unpin(robot, path, id)   #=> :ok
BB.SampleStore.pinned(robot, path)      #=> [%BB.Message{}]
```

Pinned samples are exempt from all retention policies.

#### Write API

```elixir
BB.Perception.stream(robot, path, message)  #=> :ok | {:error, :no_store}
```

`stream/3` writes to the configured store and then dispatches via `BB.publish/3`. It is a strict superset of `publish` for store-backed paths; calling it on a path without a declared store returns `{:error, :no_store}` rather than falling through silently. This makes the DSL/code dependency loud at runtime.

Drivers pick per-message: a camera that always wants storage always calls `stream`; a driver that publishes both raw and corrected versions can `publish` one and `stream` the other.

#### Retention

Declared at compile time via the `streams` DSL section (below). Retentions are typed: a `pos_integer` is a count cap, a unit compatible with `:second` is a duration cap, a unit compatible with `:byte` is a memory cap. Multiple retentions of different kinds compose as "any triggers eviction."

The store's overrun behaviour is configured via `on_overrun :ignore | :warn | :degrade`. `:degrade` emits a `BB.Perception.Health` degradation event into the health channel.

Eviction always operates on the oldest unpinned sample. Pinned samples never evict.

#### Introspection

```elixir
BB.SampleStore.stats(robot, path)
#=> %{
#     count: 30, bytes: 184_320_000,
#     oldest: t1, newest: t2,
#     pinned: 4, dropped_since: 12
#   }
```

#### Distribution

Stores are strictly local to the producing node. Cross-node consumers subscribe via pubsub for sample arrival events; if they need point-in-time lookups (e.g. for fusion), they perform `:erpc` calls against the producing node's store explicitly. The store API does not auto-RPC, to keep latency behaviour honest.

### Streams DSL Section

Sample-store configuration is declared in a top-level `streams` section. This indirection (rather than a `store do ... end` block inside the `sensor` entity) is necessary because Spark supports section patching but not entity patching — `bb_perception` cannot add a child block to the core `sensor` entity from an extension.

```elixir
streams do
  stream [:sensor, :camera_front] do
    retention 30
    on_overrun :ignore
  end

  stream [:sensor, :head, :lidar_top] do
    retention ~u(2 second)
    retention ~u(200 megabyte)
    on_overrun :degrade
  end
end
```

The `stream` entity:

- Takes a path as its identifying argument.
- Accepts multiple `retention` declarations (each validated against the union type `{:or, [:pos_integer, unit_type(compatible: :second), unit_type(compatible: :byte)]}`).
- Accepts a single `on_overrun` option (`:ignore | :warn | :degrade`, default `:ignore`).

#### Compile-time verifier

`BB.Perception.Dsl.Verifiers.ValidateStreams`:

- Every `stream` path resolves to an existing sensor or perceptor output.
- At least one `retention` declared per stream.
- No more than one retention per kind (no two byte caps, no two duration caps, no two count caps).
- All durations and byte values strictly positive.
- `on_overrun` is one of the allowed atoms.

### BB.Vision.Model behaviour

A pure callback module pattern, matching `BB.Sensor`. The behaviour describes inference; a framework-provided `BB.Vision.Model.Server` wraps it with supervision, latency policy, freshness gating, and telemetry.

```elixir
defmodule BB.Vision.Model do
  @callback load(opts :: keyword()) ::
              {:ok, state :: term()} | {:error, term()}

  @callback predict(input, state :: term()) ::
              {:ok, struct()}
              | {:ok, %{atom() => struct()}}
              | {:error, term()}
            when input: BB.Message.t() | %{atom() => BB.Message.t()}

  @callback unload(state :: term()) :: :ok

  @callback output_kinds() :: atom() | %{atom() => atom()}
  @callback input_kinds() :: [module()] | %{atom() => module()}
  @callback options_schema() :: Spark.Options.t()

  @optional_callbacks [unload: 1, options_schema: 0]
end
```

`load/1` and `predict/2` are required. `output_kinds/0` returns either an atom (single output) or a map of atom → kind (named multi-output). `input_kinds/0` mirrors this for inputs.

Adapter example skeleton:

```elixir
defmodule BB.Vision.Models.Yolo do
  use BB.Vision.Model,
    options_schema: [
      weights: [type: :string, required: true],
      classes: [type: {:list, :string}, required: true],
      score_threshold: [type: :float, default: 0.5],
      iou_threshold: [type: :float, default: 0.5]
    ]

  @impl BB.Vision.Model
  def load(opts), do: # wrap YOLO.load/1 from yolo_elixir

  @impl BB.Vision.Model
  def predict(%BB.Message{payload: %BB.Perception.Message.Image{} = img}, state) do
    # call YOLO.detect, return {:ok, %Detections{...}}
  end

  @impl BB.Vision.Model
  def output_kinds, do: :detection

  @impl BB.Vision.Model
  def input_kinds, do: [BB.Perception.Message.Image]
end
```

The adapter lives in a sibling package (`bb_vision_yolo`) that depends on `bb_perception` for the behaviour and the payload types, and on `yolo_elixir` for the actual inference. `bb_perception` itself does not depend on `yolo_elixir`, `bumblebee`, `ortex`, `nx`, or any vision library.

### BB.Vision.Model.Server

The orchestrator. One per perceptor. Owns the model state and dispatches inference to worker processes rather than running it in its own process, so the server stays responsive to stale-input drops and crash signals while inference is in flight.

Flow:

1. Server receives input message from its perceptor (cast).
2. **Intake staleness check** — if `now - input.monotonic_time` > `latency_budget × N`, drop with telemetry `:perception_lost(:stale_input)`. Default N=2.
3. Server spawns a worker (Task) with the input + model state.
4. Worker calls adapter's `predict/2`, sends `{:done, ref, result}` back.
5. **Result freshness check** — if the input is no longer the latest unprocessed one, drop the result silently. A newer inference is in flight or about to start.
6. Otherwise, forward result to perceptor.

Cancellation of in-flight inference is *not* attempted. Killing a NIF mid-call is unsafe (`Task.shutdown` leaves the NIF running and may corrupt state). The honest pattern is "let it finish, discard if stale."

#### v1 limitations

- **Singleton worker** — one inference at a time per perceptor. Pooling (`concurrency: n`) is deferred to a later revision.
- **Batching** — punted to the adapter. `Nx.Serving`-backed adapters (Bumblebee) batch internally; yolo_elixir-style adapters do not. The server contract is synchronous one-input-per-call.
- **NIF segfault recovery** — if Ortex/EXLA/HAILO segfaults, the BEAM dies. Mitigation (running perception on a separate BEAM node) is deferred.
- **Hot weight reload** — restart-to-reload only.

### perceptor DSL Section

A new top-level section added by `bb_perception`. Ties a sensor stream (or another perceptor's output) through a model to an output topic, with latency policy and optional output storage.

```elixir
perceptors do
  perceptor :front_objects do
    input [:sensor, :camera_front]

    model BB.Vision.Models.Yolo,
      weights: "yolov8n.onnx",
      classes: :coco

    latency_budget ~u(50 millisecond)
    on_overrun :degrade
    required_for [:pick_object, :move_to_pose]

    store do
      retention ~u(2 second)
    end
  end

  perceptor :stereo_depth do
    input :left,  [:sensor, :head, :camera_left], driver: true
    input :right, [:sensor, :head, :camera_right]

    model BB.Vision.Models.StereoDepth, weights: "..."

    latency_budget ~u(100 millisecond)
    sync_tolerance ~u(50 millisecond)
    on_overrun :warn
  end
end
```

#### Inputs

- **Single-input form** — `input path` (no key). Model receives a `BB.Message.t()`.
- **Multi-input form** — `input key, path` (keyed). Model receives `%{key => BB.Message.t()}`.
- **Driver** — `driver: true` on exactly one input in a multi-input perceptor. The driver's arrivals trigger inference; non-driver inputs are pulled via `BB.SampleStore.nearest/2` keyed off the driver's `monotonic_time`.
- **Sync tolerance** — `sync_tolerance` (per-perceptor or per-input) gates how far apart in time the driver and non-driver inputs can be. Defaults to the `latency_budget`. Outside the window: skip the inference and emit a degradation event.

Multi-input perceptors require their non-driver source streams to be store-backed; the verifier checks this.

#### Model

`model Module, opts` — the module must implement `BB.Vision.Model`. Options are validated against the model's `options_schema/0` at compile time.

#### Outputs

The perceptor publishes to `[:perception, perceptor_name]` by default. For single-output models, the message carries the model's single payload struct. For multi-output models, payloads are published to keyed sub-topics: `[:perception, perceptor_name, :detections]`, `[:perception, perceptor_name, :keypoints]`, etc.

`frame_id` on the output is taken from the driver input's `frame_id` by default.

#### Output storage

A `store do ... end` block inside the `perceptor` entity decorates the output topic. The block accepts the same `retention` and `on_overrun` options as the `streams` section. (This block IS inside a perceptor entity, but the `perceptor` entity is owned by `bb_perception`, so no patching is needed.)

For multi-output perceptors, the store block can be repeated per output:

```elixir
perceptor :pose_and_detect do
  input [:sensor, :camera_front]
  model BB.Vision.Models.YoloPose, ...

  output :detections do
    store retention: ~u(2 second)
  end

  output :keypoints do
    store retention: ~u(2 second)
  end
end
```

#### Latency policy

- `latency_budget` — end-to-end (input `captured_at` → output published). Required.
- `on_overrun` — `:ignore | :warn | :degrade`. Default `:ignore`.
- `lost_after` — number of consecutive budgets without output before health transitions to `:lost`. Default 5. Accepts a parameter reference.
- `recover_after` — number of consecutive in-budget completions to transition `:degraded → :healthy`. Default 10. Accepts a parameter reference.

#### `required_for`

Names commands for which this perceptor must be healthy. The DSL accepts a list of command names; the transformer builds a reverse lookup function on the robot module: `MyRobot.perceptors_required_for(command_name) :: [perceptor_path]`. See `BB.Perception.Required` below.

#### Chaining

Perceptors can consume other perceptors' outputs:

```elixir
perceptor :raw_detections do
  input [:sensor, :camera_front]
  model BB.Vision.Models.Yolo, ...
end

perceptor :tracked_objects do
  input [:perception, :raw_detections]
  model BB.Vision.Models.SimpleTracker
end
```

The verifier walks the dependency graph and rejects cycles.

#### Compile-time verifier

`BB.Perception.Dsl.Verifiers.ValidatePerceptors`:

- Each `input` path resolves to an existing sensor or perceptor output.
- Each input's payload type matches the model's declared `input_kinds/0`.
- Single vs. multi-input form is consistent with the model's declared input shape.
- `driver: true` set on at most one input; required for multi-input perceptors.
- Non-driver input source streams in multi-input perceptors are store-backed.
- Model module exists and implements `BB.Vision.Model`.
- Output topic doesn't collide with any sensor path.
- `latency_budget` is a positive duration unit.
- `on_overrun` is one of the allowed atoms.
- No cycles in the perceptor dependency graph.
- Every command name in `required_for` exists in the robot.

### BB.Perception.Health

A first-class observable for perception state. Separate from `BB.Safety` because perception doesn't actuate hardware — it informs decisions. Downstream consumers (commands, controllers, dashboards) subscribe to the health channel and decide their own policy.

#### States

- `:healthy` — producing output within budget.
- `:degraded` — overruns, stale inputs, sync misses, or `:degrade` store overruns. Output continues, quality compromised.
- `:lost` — no output for `lost_after × latency_budget`. Treat as unavailable.
- `:failed` — supervisor crash detected; recovery underway.

Hysteresis prevents flapping: `degraded → healthy` requires `recover_after` consecutive in-budget completions.

#### Health message

```elixir
%BB.Perception.Health{
  state: :healthy | :degraded | :lost | :failed,
  reasons: [:latency_overrun | :stale_input | :sync_miss |
            :store_overrun | :no_output | :crashed],
  since: monotonic_time
}
```

Wrapped in a `BB.Message` envelope and published on `[:perception_health, ...path...]`. Subscribers use BB's existing subtree-matching pubsub.

Continuous metrics (latency percentiles, drop counts, throughput) are emitted as `:telemetry` events via `BB.Telemetry`, not embedded in the Health message. Different consumers, different channels.

#### Monitor

One `BB.Perception.Health.Monitor` GenServer per perceptor (and per store-backed sensor stream with `:degrade` overrun). Lives in the perceptor's supervision subtree. Subscribes to:

- Model server worker-completion telemetry (latency).
- Sample store overrun events.
- Perceptor server process monitor (crash detection).
- A periodic check for "no output in `lost_after × budget`" → `:lost`.

#### Introspection

Last-known state is queryable without subscribing:

```elixir
BB.Perception.Health.current(robot, path)
  #=> {:ok, %BB.Perception.Health{}} | :unknown
```

Implemented as `GenServer.call(monitor, :current)` in v1; can move to ETS-backed reads later if call latency becomes a concern.

#### Failure handling

When supervisors give up restarting a perceptor (exceeded `max_restarts`), the perceptor's subtree dies and propagates up. The Health.Monitor dies with it — it does not fake a state after its perceptor is permanently dead. Downstream consumers receive the last genuine state message before the cascade (typically `:failed`), then no further messages until the robot is restarted. This is the correct outcome for "things really gone wrong."

### BB.Perception.Required (command mix-in)

The bridge between perception health and command execution. Opt-in on the command side to avoid coupling core's command system to perception.

```elixir
defmodule MyApp.Commands.PickObject do
  use BB.Command
  use BB.Perception.Required

  def handle_command(args, from, ctx) do
    # ...
  end
end
```

What `use BB.Perception.Required` does:

1. On command init, looks itself up via `ctx.robot.perceptors_required_for(command_name)`.
2. Subscribes to `[:perception_health, ...perceptor_path...]` for each.
3. Calls `BB.Perception.Health.current/2` to seed initial state.
4. Refuses to start (returns a `BB.Error.Perception.Degraded` error) unless all required perceptors are `:healthy`.
5. During execution, if any required perceptor's health drops below `:healthy`, the command preempts itself with a typed error.

The two-handshake design (perceptor declares `required_for`, command opts in via `use`) makes the dependency explicit at both ends. The verifier can optionally warn if a perceptor's `required_for` lists a command module that does not `use BB.Perception.Required`.

### Error types

```elixir
defmodule BB.Error.Perception do
  defmodule StoreOverrun do
    use BB.Error, class: :state, fields: [:path, :retention, :dropped]
  end

  defmodule StaleInput do
    use BB.Error, class: :state, fields: [:path, :age_ms, :budget_ms]
  end

  defmodule SyncMiss do
    use BB.Error, class: :state, fields: [:driver_path, :input_path, :gap_ms, :tolerance_ms]
  end

  defmodule Degraded do
    use BB.Error, class: :state, fields: [:perceptors]
  end

  defmodule ModelLoadFailed do
    use BB.Error, class: :hardware, fields: [:model, :reason]
  end

  defmodule NoStore do
    use BB.Error, class: :invalid, fields: [:path]
  end
end
```

All implement `BB.Error.Severity`.

---

## Package Structure

```
bb_perception/
├── lib/
│   ├── bb/
│   │   ├── perception.ex                       # top-level: stream/3, publish helpers
│   │   ├── perception/
│   │   │   ├── application.ex
│   │   │   ├── dsl.ex                          # Spark DSL extension
│   │   │   ├── dsl/
│   │   │   │   ├── perceptors.ex               # section
│   │   │   │   ├── perceptor.ex                # entity
│   │   │   │   ├── input.ex
│   │   │   │   ├── output.ex
│   │   │   │   ├── streams.ex                  # section
│   │   │   │   ├── stream.ex                   # entity
│   │   │   │   ├── store.ex                    # nested entity in perceptor/output
│   │   │   │   ├── retention.ex                # nested entity
│   │   │   │   ├── transformers/
│   │   │   │   └── verifiers/
│   │   │   ├── health.ex
│   │   │   ├── health/monitor.ex
│   │   │   ├── required.ex                     # command mix-in
│   │   │   ├── message/
│   │   │   │   ├── image.ex
│   │   │   │   ├── point_cloud.ex
│   │   │   │   ├── laser_scan.ex
│   │   │   │   ├── depth.ex
│   │   │   │   ├── stereo.ex
│   │   │   │   ├── ir.ex
│   │   │   │   ├── detections.ex
│   │   │   │   ├── masks.ex
│   │   │   │   ├── keypoints.ex
│   │   │   │   ├── depth_map.ex
│   │   │   │   ├── embedding.ex
│   │   │   │   └── classification.ex
│   │   ├── sample_store.ex                     # public API
│   │   ├── sample_store/
│   │   │   ├── server.ex                       # per-stream owner
│   │   │   ├── retention.ex                    # eviction policies
│   │   │   └── supervisor.ex
│   │   ├── vision/
│   │   │   ├── model.ex                        # behaviour
│   │   │   └── model/server.ex                 # orchestrator
│   │   └── error/perception/                   # error types
├── test/
├── mix.exs
└── README.md
```

### Dependencies

```elixir
defp deps do
  [
    {:bb, bb_dep("~> 0.13")},   # depends on Image/LaserScan being removed
    {:spark, "~> 2.0"},
    {:localize, "~> 0.37"},
    {:telemetry, "~> 1.0"}
  ]
end
```

No `nx`, no `ortex`, no `bumblebee`, no `evision`. Driver and adapter packages bring those.

### Companion package landscape

Not part of this proposal, but the shape `bb_perception` is designed around:

| Package | Role | Pulls in |
|---|---|---|
| `bb_camera_v4l2` | Linux camera driver | `evision` or raw V4L2 |
| `bb_camera_rtsp` | Network camera driver | `membrane_rtsp` or similar |
| `bb_lidar_rplidar` | Slamtec RPLidar driver | serial port driver |
| `bb_lidar_velodyne` | Velodyne LIDAR driver | UDP networking |
| `bb_vision_yolo` | yolo_elixir adapter | `yolo_elixir` |
| `bb_vision_bumblebee` | Bumblebee/Nx.Serving adapter | `bumblebee`, `nx`, `exla` |
| `bb_vision_ortex` | Raw ONNX adapter | `ortex` |
| `bb_vision_hailo` | HAILO accelerator adapter | HAILO NIF |
| `bb_slam_*` | SLAM algorithms | algorithm-specific |
| `bb_replay` | Record/replay sample streams | `bb_perception` only |

---

## User Experience

### Minimal use: store-backed camera

```elixir
defmodule MyRobot do
  use BB,
    extensions: [BB.Perception.Dsl]

  topology do
    link :base_link do
      sensor :camera, BB.Camera.V4L2, device: "/dev/video0"
    end
  end

  streams do
    stream [:sensor, :base_link, :camera] do
      retention 30
    end
  end
end
```

A consumer reads the latest frame whenever it wants:

```elixir
{:ok, %BB.Message{payload: %BB.Perception.Message.Image{} = img}} =
  BB.SampleStore.latest(MyRobot, [:sensor, :base_link, :camera])
```

### Object detection perceptor

```elixir
defmodule MyRobot do
  use BB, extensions: [BB.Perception.Dsl]

  topology do
    link :base_link do
      sensor :camera_front, BB.Camera.V4L2, device: "/dev/video0"
    end
  end

  streams do
    stream [:sensor, :base_link, :camera_front] do
      retention 30
    end
  end

  perceptors do
    perceptor :front_objects do
      input [:sensor, :base_link, :camera_front]

      model BB.Vision.Models.Yolo,
        weights: "priv/yolov8n.onnx",
        classes: :coco

      latency_budget ~u(50 millisecond)
      on_overrun :warn
    end
  end
end
```

A controller subscribes to detections:

```elixir
BB.PubSub.subscribe(MyRobot, [:perception, :front_objects])
# receives {:bb, path, %BB.Message{payload: %BB.Perception.Message.Detections{}}}
```

### LIDAR feeding SLAM with keyframe pinning

```elixir
streams do
  stream [:sensor, :base_link, :lidar_top] do
    retention ~u(2 second)
    retention ~u(200 megabyte)
    on_overrun :degrade
  end
end
```

The SLAM module pins keyframes for loop closure:

```elixir
def handle_info({:bb, _, %BB.Message{} = scan}, state) do
  if keyframe?(scan, state) do
    BB.SampleStore.pin(state.robot, [:sensor, :base_link, :lidar_top], scan.monotonic_time)
  end
  {:noreply, update_state(state, scan)}
end
```

### Stereo depth with input synchronisation

```elixir
perceptors do
  perceptor :stereo_depth do
    input :left,  [:sensor, :head, :camera_left], driver: true
    input :right, [:sensor, :head, :camera_right]

    model BB.Vision.Models.StereoDepth, weights: "priv/stereo.onnx"

    latency_budget ~u(100 millisecond)
    sync_tolerance ~u(20 millisecond)
    on_overrun :degrade
  end
end
```

### Command guarded by perception health

```elixir
defmodule MyApp.Commands.PickObject do
  use BB.Command
  use BB.Perception.Required

  def handle_command(_args, _from, ctx) do
    # Only runs if obstacle_detection and object_detection perceptors are :healthy
    {:ok, :grasped}
  end
end
```

```elixir
perceptors do
  perceptor :obstacle_detection do
    input [:sensor, :head, :lidar]
    model BB.Vision.Models.Obstacles, ...
    required_for [MyApp.Commands.PickObject]
  end

  perceptor :object_detection do
    input [:sensor, :head, :camera]
    model BB.Vision.Models.Yolo, ...
    required_for [MyApp.Commands.PickObject]
  end
end
```

---

## Acceptance Criteria

### Phase 1: Foundations

- [ ] `Image` and `LaserScan` removed from `bb`; equivalent payloads land in `bb_perception` with namespaced module names.
- [ ] Additional payload types: `PointCloud`, `Depth`, `Stereo`, `IR`, `Detections`, `Masks`, `Keypoints`, `DepthMap`, `Embedding`, `Classification`.
- [ ] `BB.Perception.Dsl` extension loads cleanly via `use BB, extensions: [BB.Perception.Dsl]`.
- [ ] `streams` DSL section with `stream` entity, `retention` (union-typed, multi-occurrence, max one per kind), `on_overrun`.
- [ ] Compile-time verifier for `streams` (paths resolve, retentions present and well-typed, no duplicate kinds, `on_overrun` valid).
- [ ] `BB.SampleStore.Server` per declared stream, supervised, with ETS-backed storage and pluggable retention.
- [ ] Read API: `latest`, `fetch`, `nearest`, `before`, `after`, `range`, `since`, `pin`, `unpin`, `pinned`, `stats`.
- [ ] `BB.Perception.stream/3` writes to store + dispatches via pubsub; returns `{:error, :no_store}` for paths without a declared store.
- [ ] Refc binary handling verified — no per-subscriber copies for large payloads.
- [ ] Comprehensive tests for retention semantics, including replay-clock behaviour (`{:duration, _}` measured against newest sample, not wall clock).

### Phase 2: Inference

- [ ] `BB.Vision.Model` behaviour defined with required and optional callbacks.
- [ ] `BB.Vision.Model.Server` orchestrator with worker-based dispatch, intake and result freshness gates.
- [ ] `perceptors` DSL section with `perceptor` entity, `input` (single + multi-input with driver), `model`, `latency_budget`, `on_overrun`, `sync_tolerance`, `required_for`, output `store` blocks.
- [ ] Compile-time verifier for `perceptors` (input types match `input_kinds`, single-vs-multi consistent, driver constraint, non-driver inputs store-backed, no cycles, `required_for` references exist).
- [ ] Single-output perceptors publish to `[:perception, perceptor_name]`.
- [ ] Multi-output perceptors publish to `[:perception, perceptor_name, key]` per declared output.
- [ ] Default `frame_id` taken from driver input.
- [ ] Latency budget enforced on both intake (drop stale inputs) and post-inference (drop stale results).
- [ ] At least one reference adapter exists (suggest `bb_vision_yolo` as the first sibling package) demonstrating the contract.

### Phase 3: Health & integration

- [ ] `BB.Perception.Health` payload + monitor process.
- [ ] State machine with hysteresis (`lost_after`, `recover_after` configurable via DSL + parameters).
- [ ] Health messages published on `[:perception_health, ...path...]`.
- [ ] `BB.Perception.Health.current/2` query.
- [ ] Telemetry events for latency, drops, throughput emitted via `BB.Telemetry`.
- [ ] `BB.Perception.Required` mix-in for commands; gates command startup and preempts on degradation.
- [ ] Reverse-lookup function `perceptors_required_for/1` generated on the robot module.
- [ ] Verifier warns when a perceptor's `required_for` names a command that doesn't `use BB.Perception.Required`.

### Phase 4: Chaining & polish

- [ ] Perceptor chaining (perceptor consumes another perceptor's output).
- [ ] Cycle detection in the perceptor graph.
- [ ] Output `frame_id` override option.
- [ ] Documentation: README, tutorial covering camera → detection → command, tutorial covering LIDAR + SLAM-style pinning.
- [ ] `mix.usage_rules` documentation.

### Won't have (deferred)

- [ ] Device-resident tensor handles (HAILO/CUDA zero-copy). Mitigation: payloads carry host-resident binaries; H/D copies happen on the adapter boundary.
- [ ] Inference worker pooling (`concurrency: n`).
- [ ] Cross-node sample store lookups (consumers `:erpc` explicitly if needed).
- [ ] Hot reload of model weights without restart.
- [ ] Declarative safety state transitions from health events (a separate controller subscribing to health can do this).
- [ ] Sample store eviction event publishing (only the per-perceptor health channel surfaces overruns in v1).
- [ ] Recorded replay support (separate `bb_replay` proposal).

---

## Open Questions

1. **Should `BB.Perception.Required` also handle preemption based on `:degraded`, or only `:lost`?** Strict reading: any state below `:healthy` blocks. Lenient: `:lost` blocks, `:degraded` just warns. Per-command override probably useful — possibly via `use BB.Perception.Required, require: [:healthy]` vs. `require: [:healthy, :degraded]`.

2. **Where does store retention's "duration" clock come from for replay?** Decided: newest sample's `monotonic_time`, not `System.monotonic_time/0`. Worth documenting prominently because it's surprising for anyone expecting wall-clock semantics.

3. **Should multi-output perceptors be allowed to mix store-backed and non-store-backed outputs?** Currently the design allows it (each `output` block independently declares `store`). Worth confirming no subtle issues with the supervision tree shape.

4. **`required_for` referencing a command module vs. command name.** The DSL accepts atoms. Should these be command module references (`MyApp.Commands.PickObject`) or DSL command names (`:pick_object`)? Module references are more rigorous (resolved at compile time, survive renames in editors); name references are shorter. Probably module references, with a verifier that catches typos.

5. **Should the sample store support a "tee" mode where it ALSO writes to disk?** Useful for ad-hoc recording without a full replay package. Could be a v2 retention adapter (`{:disk, path}`).

6. **What's the right behaviour when a perceptor's `input` source sensor has not yet been added to the topology (because of conditional compilation)?** Probably compile-time error — better to fail loudly than ship a degraded robot.

7. **Does the DSL need a way to opt a perceptor OUT of automatic supervision** (e.g. for a perceptor that should only run during specific commands)? Plausible v2 feature.

---

## References

- [BB.Sensor behaviour](../../bb/lib/bb/sensor.ex) — the pattern this proposal mirrors for `BB.Vision.Model`.
- [BB.Message](../../bb/lib/bb/message.ex) — envelope already carries `monotonic_time`, `wall_time`, `node`, `frame_id`.
- [BB.PubSub](../../bb/lib/bb/pub_sub.ex) — hierarchical pubsub the perception channel is built on.
- [BB.Safety](../../bb/lib/bb/safety.ex) — the contract perception health deliberately does not extend.
- [Proposal 0003: bb_dataset](0003-bb-dataset.md) — adjacent concern; recording sample streams ties into perception.
- [yolo_elixir](https://github.com/poeticoding/yolo_elixir) — reference behaviour-based wrapper; informed the `BB.Vision.Model` shape.
- [Bumblebee + Nx.Serving](https://github.com/elixir-nx/bumblebee) — the batching/distribution model the adapter for it will wrap.
- [Ortex](https://github.com/elixir-nx/ortex) — ONNX runtime wrapper, the underlying inference engine for yolo_elixir and a candidate for a generic ONNX adapter.
- [ROS sensor_msgs](http://wiki.ros.org/sensor_msgs) — payload-type precedent (`Image`, `LaserScan`, `PointCloud2`).
- [ROS vision_msgs](http://wiki.ros.org/vision_msgs) — payload-type precedent for `Detections`, `Masks`, `Keypoints`.
- [ROS message_filters ApproximateTime](http://wiki.ros.org/message_filters) — multi-input synchronisation precedent.
- [Iceoryx zero-copy IPC](https://iceoryx.io/) — the model behind the (deferred) device-handle approach.
