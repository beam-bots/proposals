<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal 0017: bb_perception

**Status:** Draft
**Author:** James Harton
**Created:** 2026-05-24
**Revised:** 2026-05-25

---

## Summary

A perception interface package for Beam Bots that provides the common contract every camera, LIDAR, depth sensor, preprocessing stage, vision-model adapter, and SLAM consumer agrees on. `bb_perception` ships the perception-shaped payload types (`Image`, `LaserScan`, `PointCloud`, `Detections`, etc.), an ETS-backed sample store for high-rate large-payload streams, a `BB.Perception.Perceptor` behaviour for OTP-flavoured processing nodes, a `perceptors` DSL section for wiring sensor streams through processing pipelines, and a `BB.Perception.Health` observable for surfacing degradation. It is intentionally deps-light so driver packages (`bb_camera_*`, `bb_lidar_*`) and heavy adapter packages (`bb_perception_yolo`, `bb_perception_bumblebee`, `bb_perception_ortex`, `bb_perception_hailo`) can depend on a single contract without pulling in EXLA, Ortex, Bumblebee, evision, or vendor NIFs transitively.

Everything in `bb_perception` lives under the `BB.Perception` namespace.

---

## Motivation

Beam Bots is a generalised robotics framework. Users range from "Raspberry Pi 5 with a HAILO module" to "workstation with the latest GPU." Perception is the area where this heterogeneity bites hardest:

- **Frame data is large.** A 1080p RGB frame is ~6MB; a 64-line LIDAR sweep is several MB; a depth map is comparable. Passing these through process mailboxes without care wastes memory and CPU.
- **Inference backends vary.** yolo_elixir wraps Ortex. Bumblebee wraps Nx.Serving. HAILO has its own NIF. Evision has a DNN module. None of these should be a hard dependency of a robot that just wants to publish a LIDAR scan.
- **Consumers vary.** A SLAM module wants keyframes pinned for loop closure. A visual-servoing controller wants the freshest detection within a latency budget. A recording tool wants every sample. A live dashboard wants the latest available.
- **Pipelines vary.** A raw camera stream often passes through several stages — decode, undistort, rotate, colour-correct — before it reaches an inference step. Each stage wants the same supervision, freshness, and storage semantics as the inference step.
- **Failure modes vary.** A camera disconnect, a model load failure, a NIF segfault, and a sample-drop overrun are all "perception is degraded" but mean very different things to downstream behaviours.

Without a shared abstraction, every driver invents its own message types, every consumer reimplements freshness gating, every adapter is a one-off, and the framework can't make any guarantees about resilience.

### Why an extension, not core

Most robots don't have cameras or LIDAR. A servo-driven arm with encoder feedback needs none of this. `bb` already depends on `nx` for forward/inverse kinematics, reference-frame transforms, and the rest of the maths stack — that's a fixed cost every BB deployment pays. The avoidable cost is the heavier ML inference stack: EXLA/XLA, Ortex/ONNX runtime, Bumblebee, evision, and vendor NIFs (HAILO, CUDA). Putting perception in core would:

- Pull those heavier dependencies (or their transitive closure) into every BB deployment.
- Bloat compile times and deploy artifacts for users who never publish a frame.
- Couple core's release cadence to perception's, slowing both.

The existing servo-driver pattern (one core behaviour, multiple sibling driver packages) is the proven shape; perception follows the same pattern with `bb_perception` playing the role of the contract layer.

### Why one interface package, not many

Drivers and consumers both need to recognise perception payload types. If each driver defined its own `Image` struct, consumers couldn't write driver-agnostic code. If each adapter defined its own behaviour, drivers couldn't be swapped behind perceptors. `bb_perception` is small, deps-light, and the single agreed contract.

### What this proposal does NOT cover

Out of scope:

- Specific camera/LIDAR drivers (`bb_camera_v4l2`, `bb_lidar_rplidar`, etc.) — sibling packages, separate proposals.
- Specific inference and preprocessing adapters (`bb_perception_yolo`, `bb_perception_bumblebee`, `bb_perception_h265`, etc.) — sibling packages, separate proposals.
- SLAM algorithms (`bb_slam_*`) — depend on this package, separate proposals.
- `bb_replay` (record/replay of sample streams) — separate proposal.
- Cross-node sample store lookups — consumers `:erpc` explicitly if needed.
- Concrete device-resident frame data implementations (HAILO/CUDA buffers) — the `BB.Perception.FrameData` protocol defining the contract ships in v1; the device-specific implementations live in sibling adapter packages.

---

## Design

### Architecture overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        bb_perception                            │
│                                                                 │
│  ┌────────────────┐  ┌──────────────────┐  ┌────────────────┐   │
│  │ Payload Types  │  │   SampleStore    │  │   Perceptor    │   │
│  │  (structs)     │  │  (per-stream)    │  │   (behaviour)  │   │
│  └────────────────┘  └──────────────────┘  └────────────────┘   │
│                                                                 │
│  ┌────────────────┐  ┌──────────────────┐                       │
│  │  Dsl extension │  │ Perception.Health│                       │
│  │  (sections)    │  │  (observable)    │                       │
│  └────────────────┘  └──────────────────┘                       │
└─────────────────────────────────────────────────────────────────┘
        ↑                       ↑                       ↑
        │                       │                       │
   ┌────┴─────┐         ┌───────┴────────┐    ┌────────┴────────┐
   │ Drivers  │         │ Perceptor      │    │  Algorithms     │
   │ (cameras,│         │ adapters       │    │  (SLAM, fusion, │
   │  LIDAR)  │         │ (codecs, YOLO) │    │   tracking)     │
   └──────────┘         └────────────────┘    └─────────────────┘
```

### Removal from core

`BB.Message.Sensor.Image` and `BB.Message.Sensor.LaserScan` currently live in `bb`. Nothing in core machinery consumes them. They were added in anticipation of perception work and are misplaced. This proposal moves both into `bb_perception` as `BB.Perception.Message.Image` and `BB.Perception.Message.LaserScan` as part of the same release.

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
BB.Perception.Message.Detections     # [{bbox, class, confidence}, ...]
BB.Perception.Message.Masks          # segmentation masks per instance
BB.Perception.Message.Keypoints      # pose estimation keypoints per instance
BB.Perception.Message.DepthMap       # predicted depth (vs. measured Depth)
BB.Perception.Message.Embedding      # feature vector
BB.Perception.Message.Classification # class label(s) with confidence
```

Each is a `use BB.Message`-derived module with a Spark-validated schema. Shapes follow ROS `sensor_msgs` / `vision_msgs` precedent where applicable.

Message identity uses the envelope's existing `monotonic_time` field — no new id field is introduced. This is sufficient because sample stores are local to the producing node, and `monotonic_time` collisions are avoided by `insert_new` (see SampleStore).

### Frame data carriers

The `data` field on perception payloads (`Image.data`, `PointCloud.points`, `LaserScan.ranges`, etc.) holds bulk data that may not always be a host-resident binary. A 1080p frame from a HAILO-attached camera may live on the NPU's memory; a depth map from a CUDA-aware sensor may live on the GPU; a future replay tool may want to memory-map frame data from disk. Consumers shouldn't have to branch on where the bytes live.

A protocol gives data carriers a uniform interface, regardless of residency:

```elixir
defprotocol BB.Perception.FrameData do
  @doc """
  Materialise the data as host-resident iodata, triggering any device→host
  copy if needed. Idempotent for already-host-resident data. Consumers that
  need a single contiguous binary call `IO.iodata_to_binary/1` on the result;
  consumers writing to a socket or file can hand the iodata straight through.
  """
  @spec to_iodata(t()) :: iodata()
  def to_iodata(data)

  @doc "Logical size in bytes, regardless of current residency."
  @spec byte_size(t()) :: non_neg_integer()
  def byte_size(data)

  @doc "Where the data currently lives."
  @spec residency(t()) :: :host | {:device, atom()}
  def residency(data)
end

defimpl BB.Perception.FrameData, for: BitString do
  def to_iodata(b), do: b
  def byte_size(b), do: Kernel.byte_size(b)
  def residency(_), do: :host
end
```

Perception payloads carry a designated frame-data field whose value implements `FrameData`:

```elixir
defmodule BB.Perception.Message.Image do
  defstruct [:height, :width, :encoding, :is_bigendian, :step, :data]

  use BB.Message,
    schema: [
      # ... existing fields ...
      data: [
        type: {:custom, __MODULE__, :validate_frame_data, []},
        required: true
      ]
    ]

  @impl BB.Message
  def frame_data_field, do: :data

  def validate_frame_data(value, _opts) do
    if BB.Perception.FrameData.impl_for(value),
      do: {:ok, value},
      else: {:error, "expected a value implementing BB.Perception.FrameData"}
  end
end
```

`frame_data_field/0` is an optional callback added to `BB.Message` in `bb` core; the callback returns the atom name of the field that carries `FrameData`, or `nil` for payloads with no bulk data (the default). `bb` itself doesn't act on the value — `bb_perception` is the consumer, and other future systems (`bb_replay`, dataset tooling) can opt in independently.

#### Consumer pattern

The canonical access pattern after retrieving a sample:

```elixir
{:ok, %BB.Message{payload: %Image{data: data}}} =
  BB.Perception.SampleStore.latest(robot, [:sensor, :base_link, :camera_front])

# Always-host-binary path
bytes = data |> BB.Perception.FrameData.to_iodata() |> IO.iodata_to_binary()

# Streaming-to-socket path (no flatten)
:gen_tcp.send(socket, BB.Perception.FrameData.to_iodata(data))

# Residency-aware path (skip D→H copy if you can consume the device handle)
case BB.Perception.FrameData.residency(data) do
  :host -> use_host_binary(BB.Perception.FrameData.to_iodata(data))
  {:device, :hailo} -> use_hailo_handle(data)
  {:device, _} -> use_host_binary(BB.Perception.FrameData.to_iodata(data))
end
```

#### Device-resident example

A HAILO-attached camera publishing without copying frames through host memory:

```elixir
defmodule BB.HailoBuffer do
  defstruct [:handle, :size, :device]
end

defimpl BB.Perception.FrameData, for: BB.HailoBuffer do
  def to_iodata(%{handle: h}), do: HailoNif.read_to_host(h)
  def byte_size(%{size: s}), do: s
  def residency(%{device: d}), do: {:device, d}
end

# In the camera driver
{:ok, msg} = BB.Message.new(BB.Perception.Message.Image, :camera_front,
  data: %BB.HailoBuffer{handle: h, size: 6_220_800, device: :hailo_0},
  width: 1920, height: 1080, encoding: :rgb8, step: 5760, is_bigendian: false
)
BB.Perception.publish(robot, [:sensor, :base_link, :camera_front], msg)
```

The driver publishes; the store holds the envelope with the small `HailoBuffer` struct as `data`; consumers materialise host iodata on demand via `to_iodata/1`. Adapters that can consume the device handle directly (e.g. a HAILO inference adapter receiving frames from the same HAILO device) skip the round-trip entirely by checking `residency/1`.

#### Construction direction

There is no `from_iodata/1` or "push" operation in the protocol. Building a payload with new frame data is normal Elixir — `BB.Message.new/3` with a `data:` value, or struct update on an existing payload. Replay loading, decompression, transcoding, and similar flows construct new payloads through the existing `BB.Message.new/3` path; the protocol is only needed for read-side polymorphism.

#### Store accounting

The SampleStore uses the protocol for byte-cap retention:

```elixir
defp payload_byte_size(%BB.Message{payload: payload}) do
  mod = payload.__struct__
  with true <- function_exported?(mod, :frame_data_field, 0),
       field when not is_nil(field) <- mod.frame_data_field() do
    BB.Perception.FrameData.byte_size(Map.get(payload, field))
  else
    _ -> 0
  end
end
```

Payloads with no `frame_data_field/0` (or returning `nil`) contribute zero to byte-cap accounting — they're still subject to count and duration caps.

### SampleStore

A per-stream ETS-backed index for high-rate, large-payload streams. Reading and writing samples for fusion, replay seeking, keyframe pinning, and multi-rate sampling are all things pubsub cannot do well on its own; the store provides them as an additive index on the same data that pubsub already fans out.

#### Identity

A "stream" is addressed by `(robot, path)` where `path` is the same hierarchical path BB pubsub uses (e.g. `[:sensor, :base_link, :camera_front]` for a link-attached sensor, `[:sensor, :gps]` for a robot-level sensor, or `[:perception, :decoded_camera]` for a perceptor output). Each store-backed stream gets a named ETS table owned by a supervised `BB.Perception.SampleStore.Server` process.

#### Storage shape

Samples are stored as `BB.Message` values directly — no parallel sample struct. The envelope already carries `monotonic_time`, `wall_time`, `node`, `frame_id`, `payload`, and `robot`. ETS table is `:ordered_set` on `monotonic_time` with `read_concurrency: true` and `write_concurrency: false`.

Refc binaries (camera frame data, point clouds) are shared by reference between ETS and any subscriber that received the message via pubsub — no copy.

#### Collision avoidance

`System.monotonic_time(:nanosecond)` is strictly monotonic per scheduler but not across processes. To avoid silent overwrites when two messages from different processes share a `monotonic_time`, the server uses `:ets.insert_new/2`. On collision it increments by 1ns and retries (in practice almost never triggered; the second ETS op is cheap when it does). The stored `monotonic_time` reflects the value actually used as the key, which may differ from `System.monotonic_time/0` at construction by a handful of nanoseconds in pathological cases.

#### Read API

```elixir
BB.Perception.SampleStore.latest(robot, path)
  #=> {:ok, %BB.Message{}} | :empty

BB.Perception.SampleStore.fetch(robot, path, id)
  #=> {:ok, %BB.Message{}} | :not_found

BB.Perception.SampleStore.nearest(robot, path, time_or_message)
  #=> {:ok, %BB.Message{}} | :empty

BB.Perception.SampleStore.before(robot, path, time_or_message)
  #=> {:ok, %BB.Message{}} | :empty

BB.Perception.SampleStore.after(robot, path, time_or_message)
  #=> {:ok, %BB.Message{}} | :empty

BB.Perception.SampleStore.range(robot, path, t_start, t_end)
  #=> [%BB.Message{}]

BB.Perception.SampleStore.since(robot, path, time_or_message)
  #=> [%BB.Message{}]
```

`id` is the sample's `monotonic_time` (integer). Time arguments polymorphically accept either an integer `monotonic_time` or a `BB.Message.t()` (whose `monotonic_time` is used). Lookups are direct `:ets.select` calls with match specs on the key — O(log n) on the ordered_set, no allocation. `nearest` ties break in favour of the earlier sample (causal, no future-data leakage).

#### Pinning

For SLAM keyframes and similar long-lived references:

```elixir
BB.Perception.SampleStore.pin(robot, path, id)    #=> :ok | :not_found
BB.Perception.SampleStore.unpin(robot, path, id)  #=> :ok
BB.Perception.SampleStore.pinned(robot, path)     #=> [%BB.Message{}]
```

Pinned samples are exempt from all retention policies.

#### Write API

```elixir
BB.Perception.publish(robot, path, message)  #=> :ok | {:error, :no_store}
```

`publish/3` writes to the configured store (setting `:robot` on the envelope before insert) and then dispatches via `BB.PubSub.publish/3`. It is a strict superset of `BB.publish/3` for store-backed paths; calling it on a path without a declared store returns `{:error, :no_store}` rather than falling through silently.

Sensors and perceptors that need sample storage have a hard dependency on `bb_perception` and use `BB.Perception.publish/3` for their store-backed outputs. The same driver can still call `BB.publish/3` directly for incidental messages that aren't sample-store backed (e.g. a camera publishing its own onboard temperature reading). Drivers with no store-backed outputs don't need to depend on `bb_perception` at all.

#### Retention

Declared at compile time via the `streams` DSL section (below). Retentions are typed: a `pos_integer` is a count cap, a unit compatible with `:second` is a duration cap, a unit compatible with `:byte` is a memory cap. Multiple retentions of different kinds compose as "any triggers eviction."

The store's overrun behaviour is configured via `on_overrun :ignore | :warn | :degrade`. `:degrade` emits a `BB.Perception.Health` degradation event into the health channel.

Eviction always operates on the oldest unpinned sample. Pinned samples never evict.

#### Introspection

```elixir
BB.Perception.SampleStore.stats(robot, path)
#=> %{
#     count: 30, bytes: 184_320_000,
#     oldest: t1, newest: t2,
#     pinned: 4, dropped_since: 12
#   }
```

#### Distribution

Stores are strictly local to the producing node. Cross-node consumers subscribe via pubsub for sample arrival events; if they need point-in-time lookups (e.g. for fusion), they perform `:erpc` calls against the producing node's store explicitly. The store API does not auto-RPC, to keep latency behaviour honest.

### Streams DSL section

Sample-store configuration is declared in a top-level `streams` section. This indirection (rather than a `store do ... end` block inside the `sensor` entity) is necessary because Spark supports section patching but not entity patching — `bb_perception` cannot add a child block to the core `sensor` entity from an extension.

```elixir
streams do
  stream [:sensor, :base_link, :camera_front] do
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

Paths address the same topics BB pubsub uses. Robot-level sensors (declared in the top-level `sensors` block in `bb`) have paths like `[:sensor, :name]`; link- or joint-attached sensors have paths like `[:sensor, :base_link, :camera_front]`. The verifier resolves both forms.

#### Compile-time verifier

`BB.Perception.Dsl.Verifiers.ValidateStreams`:

- Every `stream` path resolves to an existing sensor or perceptor output.
- At least one `retention` declared per stream.
- No more than one retention per kind (no two byte caps, no two duration caps, no two count caps).
- All durations and byte values strictly positive.
- `on_overrun` is one of the allowed atoms.

Path resolution is pre-cached at compile time via `Spark.Dsl.Transformer.persist/3` so the verifier doesn't re-walk the topology per stream.

### BB.Perception.Perceptor behaviour

A pure callback module, matching the existing pattern for `BB.Sensor`, `BB.Actuator`, and `BB.Controller`. A framework-provided `BB.Perception.Perceptor.Server` wraps the user module with supervision, input subscription, intake freshness gating, multi-input fan-in, output routing, and telemetry.

```elixir
defmodule BB.Perception.Perceptor do
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

The behaviour is intentionally generic. A perceptor might be an inference adapter, a codec, a geometric transform, a fusion node, or anything else that takes one or more input streams and produces one or more output streams. Calling them all "perceptors" reflects how the wider robotics ecosystem groups these stages.

#### Reply shape

Perceptors emit messages by returning `{:reply, [{output_name, %BB.Message{}}], state}` from any callback. Each `output_name` is an atom matching either:

- An `output :name` block declared in the DSL (multi-output perceptors), in which case the message is published to `[:perception, perceptor_name, output_name]`; or
- The conventional `:out` atom (single-output perceptors), in which case the message is published to `[:perception, perceptor_name]`.

Returning an empty list emits nothing; useful for accumulators that consume many inputs before producing one output. The model can return any combination of names in any callback.

The framework injects `:robot` and stamps the published path on dispatch (same behaviour as `BB.PubSub.publish/3`). Models are responsible for `frame_id`, `payload`, and any temporal anchoring (e.g. carrying through the driver input's `monotonic_time` if the model wants telemetry to correlate output latency to that input).

#### Type checking

There are no `input_kinds` / `output_kinds` callbacks. The model is trusted to construct correctly-typed payloads. Type errors surface at message construction time via `BB.Message.new/2`'s Spark.Options validation — payloads that don't validate against their schema raise on construction.

Downstream consumers (perceptors with `input [:perception, :other_perceptor]`, or pubsub subscribers with `message_types:`) get type-matching at runtime. The static contract is "you said this perceptor produces detections; if you publish masks instead, your consumers will reject them."

#### Adapter example skeletons

A YOLO detector:

```elixir
defmodule BB.Perception.Vision.Yolo do
  use BB.Perception.Perceptor,
    options_schema: [
      weights: [type: :string, required: true],
      classes: [type: {:list, :string}, required: true],
      score_threshold: [type: :float, default: 0.5],
      iou_threshold: [type: :float, default: 0.5]
    ]

  alias BB.Perception.Message.Detections
  alias BB.Message

  @impl BB.Perception.Perceptor
  def init(opts) do
    {:ok, model} = YOLO.load_model(weights: opts[:weights], classes: opts[:classes])
    {:ok, %{model: model, opts: opts}}
  end

  @impl BB.Perception.Perceptor
  def handle_input(%Message{payload: %BB.Perception.Message.Image{} = img} = input, state) do
    detections = YOLO.detect(state.model, img,
      score_threshold: state.opts[:score_threshold],
      iou_threshold: state.opts[:iou_threshold]
    )

    {:ok, msg} = Message.new(Detections, input.frame_id, detections: detections)

    {:reply, [out: msg], state}
  end
end
```

An H.265 decoder (pre-processing stage):

```elixir
defmodule BB.Perception.Codec.H265 do
  use BB.Perception.Perceptor

  @impl BB.Perception.Perceptor
  def init(_opts), do: {:ok, %{decoder: H265.new()}}

  @impl BB.Perception.Perceptor
  def handle_input(%BB.Message{payload: encoded} = input, state) do
    case H265.decode(state.decoder, encoded.data) do
      {:ok, raw_frame} ->
        {:ok, msg} = BB.Message.new(BB.Perception.Message.Image, input.frame_id,
          height: raw_frame.height,
          width: raw_frame.width,
          encoding: :rgb8,
          step: raw_frame.width * 3,
          data: raw_frame.data
        )
        {:reply, [out: msg], state}

      {:error, _} ->
        {:noreply, state}
    end
  end
end
```

An accumulating tracker (multi-frame state, emits one tracked-objects message per N detections):

```elixir
defmodule BB.Perception.Tracking.SimpleTracker do
  use BB.Perception.Perceptor,
    options_schema: [window: [type: :pos_integer, default: 5]]

  @impl BB.Perception.Perceptor
  def init(opts), do: {:ok, %{window: opts[:window], buffer: []}}

  @impl BB.Perception.Perceptor
  def handle_input(%BB.Message{} = detections, %{buffer: buf, window: n} = state) do
    buffer = [detections | buf] |> Enum.take(n)
    if length(buffer) == n do
      {:ok, msg} = build_tracks(buffer)
      {:reply, [out: msg], %{state | buffer: []}}
    else
      {:noreply, %{state | buffer: buffer}}
    end
  end
end
```

An async inference adapter (Task-based concurrency under model control):

```elixir
defmodule BB.Perception.Vision.AsyncYolo do
  use BB.Perception.Perceptor, options_schema: [weights: [type: :string, required: true]]

  @impl BB.Perception.Perceptor
  def init(opts) do
    {:ok, model} = YOLO.load_model(weights: opts[:weights])
    {:ok, %{model: model, pending: nil}}
  end

  @impl BB.Perception.Perceptor
  def handle_input(%BB.Message{} = input, state) do
    ref = make_ref()
    parent = self()
    Task.start(fn ->
      result = YOLO.detect(state.model, input.payload)
      send(parent, {:inference_done, ref, input, result})
    end)
    {:noreply, %{state | pending: ref}}
  end

  @impl BB.Perception.Perceptor
  def handle_info({:inference_done, ref, input, result}, %{pending: ref} = state) do
    {:ok, msg} = build_detections_msg(input, result)
    {:reply, [out: msg], %{state | pending: nil}}
  end

  def handle_info({:inference_done, _stale_ref, _, _}, state) do
    {:noreply, state}
  end
end
```

### BB.Perception.Perceptor.Server

The framework wrapper. One per perceptor. Responsibilities:

1. **Subscription.** Subscribes to declared input paths via `BB.PubSub.subscribe/3`.
2. **Intake freshness.** If `now - input.monotonic_time > latency_budget × N` (default N=2), drop with telemetry `[:bb, :perception, :dropped]` and skip dispatch. Models that want raw access can declare `freshness: :off` on the perceptor entity to bypass.
3. **Multi-input fan-in.** For perceptors with multiple `input` blocks, the driver's arrivals trigger inference; non-driver inputs are pulled via `BB.Perception.SampleStore.nearest/3` keyed off the driver's `monotonic_time`. If any non-driver gap exceeds `sync_tolerance`, drop with telemetry `[:bb, :perception, :dropped]` reason `:sync_miss`.
4. **Dispatch.** Calls `handle_input/2` with a `BB.Message.t()` (single-input form) or `%{key => BB.Message.t()}` (multi-input form).
5. **Output routing.** For each `{name, message}` in the reply, looks up `name` in the DSL's declared outputs and publishes to the corresponding topic via `BB.Perception.publish/3` (if store-backed) or `BB.PubSub.publish/3` (if not).
6. **Health transitions.** Tracks consecutive overruns/successes; on threshold transitions, publishes `BB.Perception.Health` and (if state crossed below `:healthy`) transitions the robot into the configured `degraded_state`.

#### v1 limitations

- **NIF segfault recovery.** If a NIF segfaults, the BEAM dies. Mitigation (running perception on a separate BEAM node) is deferred.
- **Hot weight reload.** Restart-to-reload only. Models can implement weight-swapping via `handle_call/3` if they want it.
- **Cancellation of in-flight inference.** Not attempted. Models that spawn Tasks own the cancellation policy; the canonical pattern is "let it finish, discard if stale" (see `AsyncYolo` example above).

### Perceptors DSL section

A new top-level section added by `bb_perception`. Declares perceptor processes — their child specs, inputs, outputs, and policy.

```elixir
perceptors degraded_state: :degraded do
  perceptor :front_objects, {BB.Perception.Vision.Yolo, weights: "yolov8n.onnx", classes: :coco} do
    input [:sensor, :base_link, :camera_front]
    latency_budget ~u(50 millisecond)
    on_overrun :degrade

    store retention: ~u(2 second)
  end

  perceptor :stereo_depth, {BB.Perception.Vision.StereoDepth, weights: "stereo.onnx"} do
    input :left,  [:sensor, :head, :camera_left], driver: true
    input :right, [:sensor, :head, :camera_right]
    latency_budget ~u(100 millisecond)
    sync_tolerance ~u(50 millisecond)
    on_overrun :warn
  end

  perceptor :pose_and_detect, {BB.Perception.Vision.YoloPose, weights: "..."} do
    input [:sensor, :base_link, :camera_front]
    latency_budget ~u(80 millisecond)

    output :detections, store: [retention: ~u(2 second)]
    output :keypoints,  store: [retention: ~u(2 second)]
  end
end
```

#### Section-level options

- `degraded_state` — atom; the operational state the robot is moved to when any perceptor's health crosses below `:healthy`. Default `:degraded`. The verifier checks this state name exists in the robot's `states do … end` section.

#### Perceptor entity

The perceptor entity follows the same `child_spec` shape as `sensor`/`actuator`/`controller`:

```elixir
perceptor :name, ChildSpec do
  # body
end
```

Where `ChildSpec` is either `Module` or `{Module, opts}`. The module must implement `BB.Perception.Perceptor`. Options are validated against the module's `options_schema/0` at compile time (literal values) and again at runtime when parameter references resolve.

#### Inputs

- **Single-input form** — `input path` (no key). Model receives a `BB.Message.t()`.
- **Multi-input form** — `input key, path` (keyed). Model receives `%{key => BB.Message.t()}`.
- **Driver** — `driver: true` on exactly one input in a multi-input perceptor. The driver's arrivals trigger inference; non-driver inputs are pulled via `BB.Perception.SampleStore.nearest/3` keyed off the driver's `monotonic_time`.
- **Sync tolerance** — `sync_tolerance` (per-perceptor) gates how far apart in time the driver and non-driver inputs can be. Defaults to the `latency_budget`. Outside the window: skip the inference and emit a degradation event.

Multi-input perceptors require their non-driver source streams to be store-backed; the verifier checks this.

#### Outputs

For perceptors with no `output` blocks, the output is implicit and named `:out`. The framework publishes the perceptor's reply messages to `[:perception, perceptor_name]`. Optional inline `store` option on the perceptor itself decorates the implicit output:

```elixir
perceptor :foo, MyModule do
  input [:sensor, :cam]
  store retention: ~u(2 second)
end
```

For multi-output perceptors, declare each output explicitly. Each `output :name` block can carry an optional `store:` keyword:

```elixir
output :detections, store: [retention: ~u(2 second), on_overrun: :degrade]
output :keypoints,  store: [retention: ~u(2 second)]
```

The model's reply tuples must use names matching declared outputs. Unknown names raise at runtime.

`frame_id` on emitted messages comes from whatever the model sets. The framework doesn't second-guess it.

#### Latency policy

- `latency_budget` — end-to-end (driver input `monotonic_time` → output published). Required when freshness gating is enabled. Used both for intake drop threshold and for health-monitor overrun detection.
- `freshness` — `:on | :off`. Default `:on`. When `:off`, the framework dispatches every input regardless of staleness.
- `on_overrun` — `:ignore | :warn | :degrade`. Default `:ignore`.
- `lost_after` — number of consecutive budgets without output before health transitions to `:lost`. Default 5. Accepts a parameter reference.
- `recover_after` — number of consecutive in-budget completions to transition `:degraded → :healthy`. Default 10. Accepts a parameter reference.

#### Chaining

Perceptors consume other perceptors' outputs by pointing `input` at a perception path:

```elixir
perceptor :decoded_camera, BB.Perception.Codec.H265 do
  input [:sensor, :base_link, :camera_front]
end

perceptor :rotated, {BB.Perception.Geometry.Rotate, degrees: 90} do
  input [:perception, :decoded_camera]
end

perceptor :detections, {BB.Perception.Vision.Yolo, weights: "..."} do
  input [:perception, :rotated]
end
```

The verifier walks the dependency graph and rejects cycles.

#### Compile-time verifier

`BB.Perception.Dsl.Verifiers.ValidatePerceptors`:

- Each `input` path resolves to an existing sensor or perceptor output. Path lookups are pre-cached via `Spark.Dsl.Transformer.persist/3`.
- Single vs. multi-input form is consistent across all `input` blocks on a perceptor.
- `driver: true` set on at most one input; required for multi-input perceptors.
- Non-driver input source streams in multi-input perceptors are store-backed.
- Perceptor module exists and implements `BB.Perception.Perceptor`.
- Output topic doesn't collide with any sensor path.
- `latency_budget` is a positive duration unit when `freshness: :on`.
- `on_overrun` is one of the allowed atoms.
- No cycles in the perceptor dependency graph.
- Section-level `degraded_state` references an existing state in `states do … end`.

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

Wrapped in a `BB.Message` envelope and published on `[:perception_health, perceptor_name]` (or `[:perception_health, ...sensor_path...]` for store-backed sensors with `:degrade` overrun). Subscribers use BB's existing subtree-matching pubsub.

Continuous metrics (latency percentiles, drop counts, throughput) are emitted as `:telemetry` events via `BB.Telemetry`, not embedded in the Health message. Different consumers, different channels.

#### Monitor

One `BB.Perception.Health.Monitor` GenServer per perceptor (and per store-backed sensor stream with `:degrade` overrun). Lives in the perceptor's supervision subtree. Subscribes to:

- Perceptor server `:input`/`:output`/`:latency` telemetry.
- Sample store overrun events.
- Perceptor server process monitor (crash detection).
- A periodic check for "no output in `lost_after × budget`" → `:lost`.

#### State machine integration

The perceptor server itself drives robot state transitions. When health transitions below `:healthy`, it calls `BB.Robot.Runtime.transition_to(robot, degraded_state)` (idempotent if the robot is already in that state). When health recovers, the server queries `BB.Perception.Health.current/2` for all sibling perceptors; if all are `:healthy`, it transitions the robot back to `:idle`.

This is the singleton-state model: one degraded state for all perception failures, declared at the section level. The existing `allowed_states` mechanism on commands handles command gating for free — commands that should be blocked during degraded operation simply omit `degraded_state` from their `allowed_states`. Commands that should still work in degraded mode (e.g. emergency stop, diagnostics) include it.

There is no separate `BB.Perception.Required` command mix-in. The state machine is the gate.

#### Introspection

Last-known state is queryable without subscribing:

```elixir
BB.Perception.Health.current(robot, path)
  #=> {:ok, %BB.Perception.Health{}} | :unknown
```

Implemented as `GenServer.call(monitor, :current)` in v1; can move to ETS-backed reads later if call latency becomes a concern.

#### Failure handling

When supervisors give up restarting a perceptor (exceeded `max_restarts`), the perceptor's subtree dies and propagates up. The Health.Monitor dies with it — it does not fake a state after its perceptor is permanently dead. Downstream consumers receive the last genuine state message before the cascade (typically `:failed`), then no further messages until the robot is restarted. This is the correct outcome for "things really gone wrong."

### Telemetry

No spans. The OTP-flavoured perceptor callbacks don't have a natural start/stop pair — accumulators consume many inputs before emitting, async inference returns out-of-band. Instead, counters and a latency event:

- `[:bb, :perception, :input]` — counter per input delivered to a perceptor.
  - Measurements: `%{count: 1}`
  - Metadata: `%{robot: atom, perceptor: atom, source_path: [atom]}`

- `[:bb, :perception, :output]` — counter per message emitted.
  - Measurements: `%{count: 1}`
  - Metadata: `%{robot: atom, perceptor: atom, output: atom, payload_module: module}`

- `[:bb, :perception, :latency]` — duration from input `monotonic_time` to emission.
  - Measurements: `%{duration: native_time}`
  - Metadata: `%{robot: atom, perceptor: atom, output: atom}`
  - Latency is computed by reading `monotonic_time` off the emitted message — the model is expected to carry through the driver input's `monotonic_time` when output latency is meaningful. For accumulators, models typically use the newest input's `monotonic_time`.

- `[:bb, :perception, :dropped]` — counter per intake freshness drop.
  - Measurements: `%{count: 1}`
  - Metadata: `%{robot: atom, perceptor: atom, source_path: [atom], reason: :stale_input | :sync_miss}`

- `[:bb, :perception, :store_overrun]` — counter per sample-store overrun event.
  - Measurements: `%{count: 1, dropped: integer}`
  - Metadata: `%{robot: atom, path: [atom], retention: term}`

### Error types

```elixir
defmodule BB.Perception.Error do
  defmodule StoreOverrun do
    use BB.Error, class: :state, fields: [:path, :retention, :dropped]
    defimpl BB.Error.Severity, do: (def severity(_), do: :warning)
  end

  defmodule StaleInput do
    use BB.Error, class: :state, fields: [:path, :age_ms, :budget_ms]
    defimpl BB.Error.Severity, do: (def severity(_), do: :warning)
  end

  defmodule SyncMiss do
    use BB.Error, class: :state,
      fields: [:driver_path, :input_path, :gap_ms, :tolerance_ms]
    defimpl BB.Error.Severity, do: (def severity(_), do: :warning)
  end

  defmodule ModelLoadFailed do
    use BB.Error, class: :hardware, fields: [:perceptor, :reason]
    defimpl BB.Error.Severity, do: (def severity(_), do: :error)
  end

  defmodule NoStore do
    use BB.Error, class: :invalid, fields: [:path]
    defimpl BB.Error.Severity, do: (def severity(_), do: :error)
  end

  defmodule UnknownOutput do
    use BB.Error, class: :invalid, fields: [:perceptor, :name, :declared]
    defimpl BB.Error.Severity, do: (def severity(_), do: :error)
  end
end
```

---

## Package structure

```
bb_perception/
├── lib/
│   ├── bb/
│   │   ├── perception.ex                       # top-level: publish/3 helper
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
│   │   │   │   ├── retention.ex
│   │   │   │   ├── transformers/
│   │   │   │   └── verifiers/
│   │   │   ├── perceptor.ex                    # behaviour
│   │   │   ├── perceptor/server.ex             # framework wrapper
│   │   │   ├── perceptor/supervisor.ex
│   │   │   ├── sample_store.ex                 # public API
│   │   │   ├── sample_store/
│   │   │   │   ├── server.ex                   # per-stream owner
│   │   │   │   ├── retention.ex
│   │   │   │   └── supervisor.ex
│   │   │   ├── health.ex
│   │   │   ├── health/monitor.ex
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
│   │   │   └── error/                          # error types
├── test/
├── mix.exs
└── README.md
```

### Dependencies

```elixir
defp deps do
  [
    {:bb, bb_dep("~> 0.13")},
    {:spark, "~> 2.0"},
    {:localize, "~> 0.37"},
    {:telemetry, "~> 1.0"}
  ]
end
```

`nx` is inherited transitively via `bb` (which uses it for kinematics and transforms); `bb_perception` adds no further ML inference dependencies. No `ortex`, no `bumblebee`, no `exla`, no `evision`, no vendor NIFs — driver and adapter packages bring those.

### Companion package landscape

Not part of this proposal, but the shape `bb_perception` is designed around:

| Package | Role | Pulls in |
|---|---|---|
| `bb_camera_v4l2` | Linux camera driver | `evision` or raw V4L2 |
| `bb_camera_rtsp` | Network camera driver | `membrane_rtsp` or similar |
| `bb_lidar_rplidar` | Slamtec RPLidar driver | serial port driver |
| `bb_lidar_velodyne` | Velodyne LIDAR driver | UDP networking |
| `bb_perception_h265` | H.265 decoder perceptor | codec NIF |
| `bb_perception_geometry` | Rotate/crop/colour-convert perceptors | pure Elixir or evision |
| `bb_perception_yolo` | yolo_elixir adapter | `yolo_elixir` |
| `bb_perception_bumblebee` | Bumblebee/Nx.Serving adapter | `bumblebee`, `nx`, `exla` |
| `bb_perception_ortex` | Raw ONNX adapter | `ortex` |
| `bb_perception_hailo` | HAILO accelerator adapter | HAILO NIF |
| `bb_slam_*` | SLAM algorithms | algorithm-specific |
| `bb_replay` | Record/replay sample streams | `bb_perception` only |

---

## User experience

### Minimal use: store-backed camera

```elixir
defmodule MyRobot do
  use BB, extensions: [BB.Perception.Dsl]

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

A consumer reads the latest frame:

```elixir
{:ok, %BB.Message{payload: %BB.Perception.Message.Image{} = img}} =
  BB.Perception.SampleStore.latest(MyRobot, [:sensor, :base_link, :camera])
```

### Object detection pipeline

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
    perceptor :front_objects,
      {BB.Perception.Vision.Yolo, weights: "priv/yolov8n.onnx", classes: :coco} do
      input [:sensor, :base_link, :camera_front]
      latency_budget ~u(50 millisecond)
      on_overrun :warn
    end
  end
end
```

A controller subscribes to detections:

```elixir
BB.subscribe(MyRobot, [:perception, :front_objects])
# receives {:bb, path, %BB.Message{payload: %BB.Perception.Message.Detections{}}}
```

### Multi-stage pipeline

```elixir
perceptors do
  perceptor :decoded_camera, BB.Perception.Codec.H265 do
    input [:sensor, :base_link, :camera_front]
  end

  perceptor :rotated, {BB.Perception.Geometry.Rotate, degrees: 90} do
    input [:perception, :decoded_camera]
  end

  perceptor :detections,
    {BB.Perception.Vision.Yolo, weights: "yolov8n.onnx", classes: :coco} do
    input [:perception, :rotated]
    latency_budget ~u(80 millisecond)
  end
end
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

```elixir
def handle_info({:bb, _, %BB.Message{} = scan}, state) do
  if keyframe?(scan, state) do
    BB.Perception.SampleStore.pin(state.robot,
      [:sensor, :base_link, :lidar_top], scan.monotonic_time)
  end
  {:noreply, update_state(state, scan)}
end
```

### Stereo depth with input synchronisation

```elixir
perceptors do
  perceptor :stereo_depth,
    {BB.Perception.Vision.StereoDepth, weights: "priv/stereo.onnx"} do
    input :left,  [:sensor, :head, :camera_left], driver: true
    input :right, [:sensor, :head, :camera_right]
    latency_budget ~u(100 millisecond)
    sync_tolerance ~u(20 millisecond)
    on_overrun :degrade
  end
end
```

### Commands gated by perception health

```elixir
states do
  state :degraded, doc: "Perception unhealthy; precision motion disabled"
end

perceptors degraded_state: :degraded do
  perceptor :obstacle_detection, {BB.Perception.Vision.Obstacles, ...} do
    input [:sensor, :head, :lidar]
    latency_budget ~u(100 millisecond)
  end
end

commands do
  command :pick_object do
    handler MyApp.Commands.PickObject
    allowed_states [:idle]    # not :degraded — auto-gated by state machine
  end

  command :emergency_stop do
    handler MyApp.Commands.Stop
    allowed_states [:idle, :degraded, :executing]
  end
end
```

When `:obstacle_detection` degrades, the perceptor server transitions the robot to `:degraded`. `pick_object` is now blocked by the existing `allowed_states` machinery; `emergency_stop` still works. When the perceptor recovers and all sibling perceptors are also healthy, the robot transitions back to `:idle`.

---

## Acceptance criteria

### Phase 1: Foundations

- [ ] `Image` and `LaserScan` removed from `bb`; equivalent payloads land in `bb_perception` with namespaced module names.
- [ ] Additional payload types: `PointCloud`, `Depth`, `Stereo`, `IR`, `Detections`, `Masks`, `Keypoints`, `DepthMap`, `Embedding`, `Classification`.
- [ ] `BB.Perception.Dsl` extension loads cleanly via `use BB, extensions: [BB.Perception.Dsl]`.
- [ ] `streams` DSL section with `stream` entity, `retention` (union-typed, multi-occurrence, max one per kind), `on_overrun`.
- [ ] Compile-time verifier for `streams` (paths resolve via persisted topology cache, retentions present and well-typed, no duplicate kinds, `on_overrun` valid).
- [ ] `BB.Perception.SampleStore.Server` per declared stream, supervised, with ETS-backed storage, `insert_new` collision avoidance, and pluggable retention.
- [ ] Read API: `latest`, `fetch`, `nearest`, `before`, `after`, `range`, `since`, `pin`, `unpin`, `pinned`, `stats`.
- [ ] `BB.Perception.publish/3` writes to store + dispatches via pubsub; returns `{:error, :no_store}` for paths without a declared store.
- [ ] Refc binary handling verified — no per-subscriber copies for large payloads (test via `:erlang.process_info(pid, :binary)`).
- [ ] `BB.Perception.FrameData` protocol with `to_iodata/1`, `byte_size/1`, `residency/1`; `BitString` impl.
- [ ] `frame_data_field/0` optional callback added to `BB.Message` in `bb`; perception payloads carrying bulk data implement it.
- [ ] SampleStore byte-cap retention reads payload sizes via `BB.Perception.FrameData.byte_size/1`.

### Phase 2: Perceptor behaviour

- [ ] `BB.Perception.Perceptor` behaviour with `init`, `handle_input`, `handle_info`, `handle_call`, `handle_options`, `terminate`, `options_schema` callbacks.
- [ ] `BB.Perception.Perceptor.Server` framework wrapper handles subscription, intake freshness, multi-input fan-in, dispatch, and output routing.
- [ ] `perceptors` DSL section with `perceptor` entity (sensor-style child_spec), `input` (single + multi-input with driver), `output`, `latency_budget`, `freshness`, `on_overrun`, `sync_tolerance`, inline `store` options.
- [ ] Section-level `degraded_state` option with verifier check against `states`.
- [ ] Compile-time verifier for `perceptors` (paths resolve, single-vs-multi consistent, driver constraint, non-driver inputs store-backed, no cycles).
- [ ] Single-output perceptors publish to `[:perception, perceptor_name]` from reply tuples tagged `:out`.
- [ ] Multi-output perceptors publish to `[:perception, perceptor_name, name]` per declared output.
- [ ] Latency budget enforced on intake (drop stale inputs with telemetry).
- [ ] At least one reference perceptor adapter exists demonstrating the contract.

### Phase 3: Health & state machine integration

- [ ] `BB.Perception.Health` payload + monitor process.
- [ ] State machine with hysteresis (`lost_after`, `recover_after` configurable via DSL + parameters).
- [ ] Health messages published on `[:perception_health, ...path...]`.
- [ ] `BB.Perception.Health.current/2` query.
- [ ] Telemetry events: `:input`, `:output`, `:latency`, `:dropped`, `:store_overrun`.
- [ ] Perceptor server drives robot state transitions: degradation → `degraded_state`, recovery → `:idle` when all siblings healthy.

### Phase 4: Chaining & polish

- [ ] Perceptor chaining (perceptor consumes another perceptor's output).
- [ ] Cycle detection in the perceptor graph.
- [ ] Documentation: README, tutorial covering camera → decode → detect, tutorial covering LIDAR + SLAM-style pinning, tutorial covering accumulator and async-inference patterns.
- [ ] `mix.usage_rules` documentation.

### Won't have (deferred)

- [ ] Concrete device-resident `FrameData` implementations (HAILO/CUDA zero-copy). The `BB.Perception.FrameData` protocol ships in v1 so sibling packages (`bb_perception_hailo`, etc.) can add device-resident impls without further changes to `bb_perception`; this proposal does not include any such impl beyond `BitString`.
- [ ] Cross-node sample store lookups (consumers `:erpc` explicitly if needed).
- [ ] Hot reload of model weights without restart (perceptors can implement via `handle_call/3` if they want it).
- [ ] Declarative safety state transitions from health events (a separate controller subscribing to health can do this).
- [ ] Sample store eviction event publishing (only the per-perceptor health channel surfaces overruns in v1).
- [ ] Recorded replay support (separate `bb_replay` proposal).

---

## Open questions

1. **Single-output reply shorthand.** The proposal uses `:out` as the implicit name in `{:reply, [{:out, msg}], state}`. Worth supporting `{:reply, %BB.Message{}, state}` as a shorthand that wraps to `[{:out, msg}]` — more ergonomic for the common case, no loss of consistency since the framework normalises on the way through.

2. **Store retention duration clock.** Decided: newest sample's `monotonic_time`, not `System.monotonic_time/0`. Worth documenting prominently because it's surprising for anyone expecting wall-clock semantics.

3. **Multi-output perceptors mixing store-backed and non-store-backed outputs.** Currently allowed (each `output` block independently declares `store`). Worth confirming no subtle issues with the supervision tree shape.

4. **Conditionally-compiled topology.** What's the right behaviour when a perceptor's `input` source sensor has not yet been added to the topology because of conditional compilation? Probably compile-time error — better to fail loudly than ship a degraded robot.

5. **Opting out of automatic supervision.** Should the DSL support marking a perceptor as "only run during specific commands"? Plausible v2 feature; for v1 perceptors always run.

6. **Bypassing the store on `BB.Perception.publish/3`.** Some callers might want to publish a message to a store-backed path without storing it (e.g. test injection). Currently the answer is "use `BB.publish/3` directly" but the path is store-backed by definition. Probably leave as-is and revisit if a concrete need emerges.

7. **Degraded-state precedence with multiple robots / multi-perceptor degradation.** Singleton `degraded_state` means any perceptor's degradation transitions to the same target state. If different perceptors should drive different states (e.g. vision fault → `:vision_degraded`, IMU fault → `:imu_degraded`), a future revision can promote `degraded_state` to a per-perceptor option. For v1 the singleton form is enough.

---

## References

- [BB.Sensor behaviour](../../bb/lib/bb/sensor.ex) — the pattern this proposal mirrors for `BB.Perception.Perceptor`.
- [BB.Message](../../bb/lib/bb/message.ex) — envelope carries `monotonic_time`, `wall_time`, `node`, `frame_id`, `robot`.
- [BB.PubSub](../../bb/lib/bb/pub_sub.ex) — hierarchical pubsub the perception channel is built on.
- [BB.Safety](../../bb/lib/bb/safety.ex) — the contract perception health deliberately does not extend.
- [Proposal 0003: bb_dataset](0003-bb-dataset.md) — adjacent concern; recording sample streams ties into perception.
- [yolo_elixir](https://github.com/poeticoding/yolo_elixir) — reference behaviour-based wrapper; informed the original behaviour shape.
- [Bumblebee + Nx.Serving](https://github.com/elixir-nx/bumblebee) — the batching/distribution model the adapter for it will wrap.
- [Ortex](https://github.com/elixir-nx/ortex) — ONNX runtime wrapper, the underlying inference engine for yolo_elixir and a candidate for a generic ONNX adapter.
- [ROS sensor_msgs](http://wiki.ros.org/sensor_msgs) — payload-type precedent (`Image`, `LaserScan`, `PointCloud2`).
- [ROS vision_msgs](http://wiki.ros.org/vision_msgs) — payload-type precedent for `Detections`, `Masks`, `Keypoints`.
- [ROS message_filters ApproximateTime](http://wiki.ros.org/message_filters) — multi-input synchronisation precedent.
- [Iceoryx zero-copy IPC](https://iceoryx.io/) — the model behind the (deferred) device-handle approach.
