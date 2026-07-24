<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal 0017: bb_perception

**Status:** Draft
**Author:** James Harton
**Created:** 2026-05-24
**Revised:** 2026-07-22

---

## Summary

A perception interface package for Beam Bots that provides the common contract every camera, LIDAR, depth sensor, preprocessing stage, vision-model adapter, and SLAM consumer agrees on. `bb_perception` ships perception-shaped payload types (`Image`, `LaserScan`, `PointCloud`, `Detections`, etc.), an ETS-backed sample store for high-rate large-payload streams, a thin `perceptors` DSL for wiring processing pipelines through the existing `BB.Estimator` runtime, and `FrameData`-aware handling for device-resident data. It does not define a second processing behaviour, server, health state machine, or telemetry contract: every perceptor implements `BB.Estimator` and runs in `BB.Estimator.Server`. Store-backed outputs use `BB.Perception.publish/3`, which stores the canonical message envelope before publishing that same envelope through `BB.PubSub`.

The package is intentionally deps-light so driver packages (`bb_camera_*`, `bb_lidar_*`) and heavy adapter packages (`bb_perception_yolo`, `bb_perception_bumblebee`, `bb_perception_ortex`, `bb_perception_hailo`) can depend on a single contract without pulling in EXLA, Ortex, Bumblebee, evision, or vendor NIFs transitively.

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
- Concrete device-resident frame data implementations (HAILO/CUDA buffers) — the `BB.Perception.FrameData` protocol and ownership contract ship in v1; device-specific implementations live in sibling adapter packages.

---

## Design

### Architecture overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        bb_perception                            │
│                                                                 │
│  ┌────────────────┐  ┌──────────────────┐  ┌────────────────┐   │
│  │ Payload Types  │  │   SampleStore    │  │ Perceptors DSL │   │
│  │  (structs)     │  │  (per-stream)    │  │ (thin adapter) │   │
│  └────────────────┘  └──────────────────┘  └───────┬────────┘   │
│                                                    │ lowers to  │
│  ┌────────────────┐  ┌──────────────────┐          ▼            │
│  │   FrameData    │  │ Perception.publish│  BB.Estimator entity │
│  │   protocol     │  │ store → PubSub    │  + Server (in core)  │
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

This is a breaking change for external code that references either module. No current sibling package uses them, but external use cannot be assumed away: the move requires a coordinated `bb` release, changelog entry, and `bb_perception` migration guide covering both module names and the `FrameData`-based bulk fields.

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

#### Numeric array convention

Bulk numerical arrays use typed, shaped `Nx.Tensor` values rather than lists of floats or untyped binaries. The tensor preserves element type and shape while payload fields provide semantic metadata such as units, axis names, point-field descriptors, or image encoding:

| Payload field | V1 representation |
|---|---|
| `LaserScan.ranges` / `intensities` | rank-1 `{:f, 32}` tensor |
| `PointCloud.points` | rank-2 tensor plus point-field descriptors |
| `Depth.data` / `DepthMap.data` | rank-2 tensor; payload declares units/encoding |
| `Masks.data` | rank-3 boolean or `{:u, 8}` tensor |
| `Keypoints.data` | tensor whose final axis carries coordinates/confidence |
| `Embedding.data` | rank-1 floating-point tensor |

Encoded byte streams, such as compressed images, remain binaries with an explicit payload encoding. Decoded images may use either a binary with width/height/stride metadata or a shaped tensor, according to their encoding. Bulk numeric fields do not use `[float()]`; moving the current `LaserScan` shape is therefore a data-schema migration, not only a module rename.

Message identity uses the canonical stored envelope's `monotonic_time` field; no new id field is introduced. Sample stores are local to the producing node. If insertion resolves a timestamp collision, the adjusted envelope becomes canonical and the same envelope is subsequently delivered to PubSub subscribers.

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
  def to_iodata(binary) when is_binary(binary), do: binary
  def byte_size(binary) when is_binary(binary), do: Kernel.byte_size(binary)
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
  def frame_data_fields, do: [:data]

  def validate_frame_data(value, _opts) do
    cond do
      is_binary(value) ->
        {:ok, value}

      is_bitstring(value) ->
        {:error, "expected a byte-aligned binary"}

      BB.Perception.FrameData.impl_for(value) ->
        {:ok, value}

      true ->
        {:error, "expected a value implementing BB.Perception.FrameData"}
    end
  end
end
```

`frame_data_fields/0` is an optional callback added to `BB.Message` in `bb` core. It returns every field containing a `FrameData` carrier and defaults to `[]`. A list supports payloads such as stereo pairs or masks without a later callback change. `bb` itself doesn't act on the values — `bb_perception` is the consumer, and other future systems (`bb_replay`, dataset tooling) can opt in independently. The built-in `BitString` implementation accepts byte-aligned binaries only because arbitrary bitstrings are not valid iodata.

V1 also implements `FrameData` for `Nx.Tensor`. `to_iodata/1` materialises tensor data through `Nx.to_binary/1`, `byte_size/1` derives logical bytes from the tensor shape and element type without forcing a device-to-host copy, and `residency/1` reports `:host` for the binary backend or the corresponding device backend. Adapters needing a stable zero-copy device handle use their own carrier struct rather than relying on Nx backend internals.

#### Carrier lifetime

A device-resident carrier must own a BEAM-managed reference to its underlying buffer. The bytes and device handle remain valid for as long as the carrier term is reachable, including through ETS, subscriber mailboxes, and pinned samples; resource release occurs through the carrier's NIF/resource destructor after the final reference is collected. Borrowed handles that a driver may recycle while the term remains reachable do not satisfy `FrameData` and must copy into an owning carrier before publication. The protocol deliberately has no explicit `release/1`: one consumer cannot invalidate data still retained by another.

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

  if function_exported?(mod, :frame_data_fields, 0) do
    mod.frame_data_fields()
    |> Enum.reduce(0, fn field, total ->
      total + BB.Perception.FrameData.byte_size(Map.fetch!(payload, field))
    end)
  else
    0
  end
end
```

Payloads with no `frame_data_fields/0` callback, or returning `[]`, contribute zero to byte-cap accounting — they're still subject to count and duration caps.

### SampleStore

A per-stream ETS-backed index for high-rate, large-payload streams. Reading and writing samples for fusion, replay seeking, keyframe pinning, and multi-rate sampling are all things pubsub cannot do well on its own; the store provides them as an additive index on the same data that pubsub already fans out.

#### Identity

A "stream" is addressed by `(robot, path)` where `path` is the same hierarchical path BB pubsub uses (e.g. `[:sensor, :base_link, :camera_front]` for a link-attached sensor, `[:sensor, :gps]` for a robot-level sensor, or `[:perception, :decoded_camera]` for a perceptor output). Each store-backed stream gets a named ETS table owned by a supervised `BB.Perception.SampleStore.Server` process.

#### Storage shape

Samples are stored as `{monotonic_time, %BB.Message{}}` tuples — no parallel sample struct. ETS requires tuple objects and uses the first element as the `:ordered_set` key; the envelope remains the canonical returned value and already carries `monotonic_time`, `wall_time`, `node`, `frame_id`, `payload`, and `robot`. The table enables `read_concurrency: true` and keeps `write_concurrency: false` because its server serialises writes.

Within the local BEAM, refc binaries (camera frame data, point clouds) are shared by reference between ETS and PubSub subscribers — no per-subscriber binary copy.

#### Collision avoidance

`System.monotonic_time(:nanosecond)` is strictly monotonic per scheduler but not across processes. To avoid silent overwrites when two messages share a `monotonic_time`, the server uses `:ets.insert_new/2`. On collision it increments the timestamp by 1ns and retries. The server returns the adjusted canonical envelope to `BB.Perception.publish/3`; that exact value is both retained and published.

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

`id` is the sample's canonical `monotonic_time` (integer). Time arguments polymorphically accept either an integer timestamp or a `BB.Message.t()`. Point and nearest-neighbour lookups use ordered-set key traversal (`:ets.prev/2` and `:ets.next/2`); range operations use match specifications. `nearest` ties break in favour of the earlier sample to avoid future-data leakage.

#### Pinning

For SLAM keyframes and similar long-lived references:

```elixir
BB.Perception.SampleStore.pin(robot, path, id)    #=> :ok | :not_found
BB.Perception.SampleStore.unpin(robot, path, id)  #=> :ok
BB.Perception.SampleStore.pinned(robot, path)     #=> [%BB.Message{}]
```

Pinned samples are exempt from all retention policies. `stats/2` reports pinned count and bytes. If insertion cannot satisfy every configured cap after evicting all eligible older samples, the server removes the newly inserted sample, returns `BB.Perception.Error.StoreOverrun`, and publishes nothing; it does not remain silently over cap or publish an envelope that is no longer stored. Pin ownership and automatic release remain open design questions; callers must unpin samples they no longer need.

#### Write API

```elixir
BB.Perception.publish(robot, path, message)
#=> :ok | {:error, BB.Error.t()}
```

`publish/3` provides store-before-publish ordering for a declared local stream:

1. Resolve the store for `(robot, path)`; a missing declaration returns `BB.Perception.Error.NoStore`.
2. Set `message.robot` before insertion.
3. Insert synchronously, resolve any timestamp collision, and run retention accounting.
4. If storage fails, return the error and publish nothing.
5. Publish the exact canonical stored envelope through `BB.PubSub.publish/3` only after insertion succeeds.
6. If PubSub raises, leave the canonical envelope stored and surface the publication failure. Storage is not rolled back and retry is not idempotent.

A subscriber therefore never receives an envelope that failed storage, although the converse is not guaranteed: an envelope can remain stored after PubSub failure, and normal retention may evict it later. Direct `BB.PubSub.publish/3` on a declared store-backed path bypasses storage and is unsupported; the framework does not globally intercept PubSub.

Sensors and perceptors that need sample storage have a hard dependency on `bb_perception` and use `BB.Perception.publish/3` for their store-backed outputs. The same driver can still call `BB.publish/3` directly for incidental messages that aren't sample-store backed (e.g. a camera publishing its own onboard temperature reading). Drivers with no store-backed outputs don't need to depend on `bb_perception` at all.

#### Retention

Declared at compile time via the `streams` DSL section (below). Retentions are typed: a `pos_integer` is a count cap, a unit compatible with `:second` is a duration cap, a unit compatible with `:byte` is a memory cap. Multiple retentions of different kinds compose as "any triggers eviction."

Duration retention is measured in stream time from the newest canonical sample's `monotonic_time`, not from wall time. A stopped stream therefore retains its final window until another sample arrives; duration retention is not a wall-clock TTL.

The store's overrun behaviour is configured via `on_overrun :ignore | :warn`. Retention and pin-pressure events belong to the store, not the estimator health state machine. Both settings return insertion failure when caps cannot be met; `:warn` additionally emits a log entry. Store-overrun telemetry is always emitted.

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

Stores and BB PubSub are strictly local to the producing node. Cross-node transport requires an explicit bridge outside this proposal. A remote process that already knows the producing node may perform an `:erpc` store lookup, but the store API does not auto-RPC and no cross-node subscription is implied.

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
    on_overrun :warn
  end
end
```

The `stream` entity:

- Takes a path as its identifying argument.
- Accepts multiple `retention` declarations (each validated against the union type `{:or, [:pos_integer, unit_type(compatible: :second), unit_type(compatible: :byte)]}`).
- Accepts a single `on_overrun` option (`:ignore | :warn`, default `:ignore`).

Paths address the same topics BB pubsub uses. Robot-level sensors (declared in the top-level `sensors` block in `bb`) have paths like `[:sensor, :name]`; link- or joint-attached sensors have paths like `[:sensor, :base_link, :camera_front]`. The verifier resolves both forms.

The perception DSL transformer contributes a robot-scoped store supervisor through core's extension-child hook. Store children start before topology producers, so a correctly declared camera or perceptor cannot race its store during normal robot startup. They terminate with that robot and are never discovered through application-global state.

#### Compile-time verifier

`BB.Perception.Dsl.Verifiers.ValidateStreams`:

- Every `stream` path resolves to an existing sensor or perceptor output.
- At least one `retention` declared per stream.
- No more than one retention per kind (no two byte caps, no two duration caps, no two count caps).
- All durations and byte values strictly positive.
- `on_overrun` is `:ignore` or `:warn`.

Path resolution is pre-cached at compile time via `Spark.Dsl.Transformer.persist/3` so the verifier doesn't re-walk the topology per stream.

### Perceptors as estimators

Perceptor modules implement `BB.Estimator` from `bb` core. The callback contract (`init`, `handle_input`, `handle_info`, `handle_call`, `handle_options`, `terminate`, `options_schema`) is shared between perception and other forms of state estimation. There is no `BB.Perception.Perceptor` behaviour and no perception-specific server.

A perceptor might be an inference adapter, a codec, a geometric transform, a fusion node, or anything else that takes one or more input streams and produces one or more output streams. Calling them all "perceptors" reflects how the wider robotics ecosystem groups these stages — and the `BB.Estimator` contract is general enough to cover both perception and lighter-weight numerical fusion.

#### Reply shape

Perceptors emit messages by returning `{:reply, [{output_name, %BB.Message{}}], state}` from any callback. Each `output_name` is an atom matching either:

- An `output :name` block declared in the DSL (multi-output perceptors), in which case the message is published to `[:perception, perceptor_name, output_name]`; or
- The conventional `:out` atom (single-output perceptors), in which case the message is published to `[:perception, perceptor_name]`.

Returning an empty list emits nothing; useful for accumulators that consume many inputs before producing one output. The model can return any combination of names in any callback.

The publisher injects `:robot`; the source path is delivered separately in the PubSub tuple and is not stored in the envelope. Models are responsible for `frame_id`, `payload`, and any temporal anchoring they need.

#### Type checking

There are no `input_kinds` / `output_kinds` callbacks. The model is trusted to construct correctly-typed payloads. `BB.Message.new/3` returns validation errors from Spark.Options; callers that deliberately want raising behaviour use `BB.Message.new!/3`.

Downstream consumers (perceptors with `input [:perception, :other_perceptor]`, or pubsub subscribers with `message_types:`) get type-matching at runtime. The static contract is "you said this perceptor produces detections; if you publish masks instead, your consumers will reject them."

#### Adapter example skeletons

A YOLO detector:

```elixir
defmodule BB.Perception.Vision.Yolo do
  use BB.Estimator,
    options_schema: [
      weights: [type: :string, required: true],
      classes: [type: {:list, :string}, required: true],
      score_threshold: [type: :float, default: 0.5],
      iou_threshold: [type: :float, default: 0.5]
    ]

  alias BB.Perception.Message.Detections
  alias BB.Message

  @impl BB.Estimator
  def init(opts) do
    {:ok, model} = YOLO.load_model(weights: opts[:weights], classes: opts[:classes])
    {:ok, %{model: model, opts: opts}}
  end

  @impl BB.Estimator
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
  use BB.Estimator

  @impl BB.Estimator
  def init(_opts), do: {:ok, %{decoder: H265.new()}}

  @impl BB.Estimator
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
  use BB.Estimator,
    options_schema: [window: [type: :pos_integer, default: 5]]

  @impl BB.Estimator
  def init(opts), do: {:ok, %{window: opts[:window], buffer: []}}

  @impl BB.Estimator
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
  use BB.Estimator, options_schema: [weights: [type: :string, required: true]]

  @impl BB.Estimator
  def init(opts) do
    {:ok, model} = YOLO.load_model(weights: opts[:weights])
    {:ok, %{model: model, pending: nil}}
  end

  @impl BB.Estimator
  def handle_input(%BB.Message{} = input, state) do
    ref = make_ref()
    parent = self()
    Task.start(fn ->
      result = YOLO.detect(state.model, input.payload)
      send(parent, {:inference_done, ref, input, result})
    end)
    {:noreply, %{state | pending: ref}}
  end

  @impl BB.Estimator
  def handle_info({:inference_done, ref, input, result}, %{pending: ref} = state) do
    {:ok, msg} = build_detections_msg(input, result)
    {:reply, [out: msg], %{state | pending: nil}}
  end

  def handle_info({:inference_done, _stale_ref, _, _}, state) do
    {:noreply, state}
  end
end
```

### Runtime reuse

The `perceptors` DSL is syntax over the existing estimator runtime. At compile time each `perceptor` is lowered to a robot-level `BB.Dsl.Estimator` configuration with perception output paths. At runtime every perceptor is a `BB.Estimator.Server`; `bb_perception` does not copy, wrap, proxy, or specialise that server.

This requires small, generic additions to `bb` core:

1. **Robot-level estimators.** Core accepts estimator entities that are not nested under a sensor or link and supervises them under `BB.TopologySupervisor`, using its existing restart budget and force-disarm-on-exhaustion behaviour. Their target frame is `nil`; emitted messages retain the model-supplied `frame_id`. Estimator names participate in robot-wide process-name uniqueness validation.
2. **Pluggable output publication.** An estimator output carries a `publish_with: {module, function}` pair, defaulting to `{BB.PubSub, :publish}`. Store-backed perceptor outputs lower to `{BB.Perception, :publish}`. The estimator calls either function with `(robot, path, message)` and accepts only `:ok`; `{:error, reason}` stops the estimator with that reason, while raises retain normal process-failure semantics. Output telemetry is emitted only after `:ok`.
3. **Generic intake freshness.** A `max_input_age` estimator option drops stale local inputs before callback dispatch and feeds the existing `:stale_input` health transition. It is independent of `latency_budget`, which continues to measure callback execution time.
4. **Recovery accounting.** Every successful dispatch advances estimator recovery hysteresis, even when `latency_budget` is omitted. Freshness, synchronisation, or lost transitions must not become permanent merely because callback timing is unconfigured.
5. **Effective outputs.** Estimator verification and cycle detection use explicit output path overrides, allowing `[:perception, ...]` chains to resolve correctly. Explicit multi-output declarations do not also synthesise an undeclared `:out` output.
6. **Bulk-data metadata.** The optional `BB.Message.frame_data_fields/0` callback described above lets packages account for bulk fields without core depending on `bb_perception`.
7. **Extension-owned children.** Robot DSL extensions can contribute ordered children to `BB.TopologySupervisor`. `bb_perception` contributes one store supervisor per robot, starts all declared stores before sensor/controller/link producers, and stops them with the robot. Store-supervisor exhaustion follows topology failure semantics.
8. **Driver liveness.** For multi-input estimators, only driver arrivals or successful dispatches reset `lost_after`; a live auxiliary stream cannot hide a lost driver.
9. **Consistent timing units.** Estimator callback and input-to-output latency measurements convert both operands to nanoseconds before subtraction and document measurements in nanoseconds.

Multi-input perceptors use `BB.Estimator.Server`'s existing latest-envelope fan-in and `sync_tolerance`. Algorithms needing historical or nearest-neighbour lookup call `BB.Perception.SampleStore` explicitly; v1 does not add a second automatic fan-in implementation.

#### v1 limitations

- **NIF segfault recovery.** If a NIF segfaults, the BEAM dies. Mitigation (running perception on a separate BEAM node) is deferred.
- **Hot weight reload.** Restart-to-reload only. Models can implement weight-swapping via `handle_call/3` if they want it.
- **Cancellation of in-flight inference.** Not attempted. Models that spawn Tasks own the cancellation policy; the canonical pattern is "let it finish, discard if stale" (see `AsyncYolo` example above).
- **Async latency accounting.** Core `latency_budget` and latency telemetry cover synchronous `handle_input/2` execution. Work completed later through `handle_info/2`, such as `AsyncYolo`, must emit adapter-specific end-to-end telemetry if required; v1 does not infer asynchronous task boundaries.

### Perceptors DSL section

A new top-level section added by `bb_perception`. It provides perception terminology, conventional output paths, and inline sample-store declarations while lowering execution to robot-level estimators.

```elixir
perceptors do
  perceptor :front_objects, {BB.Perception.Vision.Yolo, weights: "yolov8n.onnx", classes: :coco} do
    input [:sensor, :base_link, :camera_front]
    latency_budget ~u(50 millisecond)
    on_degraded :enter_degraded_mode

    store retention: ~u(2 second)
  end

  perceptor :stereo_depth, {BB.Perception.Vision.StereoDepth, weights: "stereo.onnx"} do
    input :left,  [:sensor, :head, :camera_left], driver?: true
    input :right, [:sensor, :head, :camera_right]
    latency_budget ~u(100 millisecond)
    sync_tolerance ~u(50 millisecond)
  end

  perceptor :pose_and_detect, {BB.Perception.Vision.YoloPose, weights: "..."} do
    input [:sensor, :base_link, :camera_front]
    latency_budget ~u(80 millisecond)

    output :detections, store: [retention: ~u(2 second)]
    output :keypoints,  store: [retention: ~u(2 second)]
  end
end
```

The `perceptors` section has no section-level options. Inputs, timing, health transitions, parameter handling, callback dispatch, and telemetry retain their `BB.Estimator` semantics.

#### Perceptor entity

The perceptor entity follows the same `child_spec` shape as `sensor`/`actuator`/`controller`:

```elixir
perceptor :name, ChildSpec do
  # body
end
```

Where `ChildSpec` is either `Module` or `{Module, opts}`. The module must implement `BB.Estimator`. Options are validated against the module's `options_schema/0` at compile time for literal values and again at runtime when parameter references resolve. The perceptor transformer produces core estimator configuration; it does not produce a different process type.

#### Inputs

- **Single-input form** — `input path` (no key). Model receives a `BB.Message.t()`.
- **Multi-input form** — `input key, path` (keyed). Model receives `%{key => BB.Message.t()}`.
- **Driver** — `driver?: true` on exactly one input in a multi-input perceptor. The driver's arrivals trigger inference; the estimator server snapshots its most recently received non-driver envelopes.
- **Sync tolerance** — `sync_tolerance` gates how far apart the driver and non-driver envelopes may be. Outside the window the server skips callback dispatch, emits estimator dropped telemetry, and applies the existing `:sync_miss` health transition.

Inputs do not need sample stores for automatic fan-in. A model that requires nearest-neighbour or historical samples declares the relevant streams as store-backed and queries the store explicitly.

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
output :detections, store: [retention: ~u(2 second), on_overrun: :warn]
output :keypoints,  store: [retention: ~u(2 second)]
```

The model's reply tuples must use names matching declared outputs. Unknown names raise at runtime. The transformer sets each output's effective path and `publish_with` pair: store-backed outputs use `{BB.Perception, :publish}`; all others retain `{BB.PubSub, :publish}`.

`frame_id` on emitted messages comes from whatever the model sets. The framework doesn't second-guess it.

#### Latency policy

- `max_input_age` — maximum local age of the driver envelope at intake. When omitted there is no age gate. Cross-node monotonic timestamps are not comparable and are rejected when this option is set.
- `latency_budget` — maximum synchronous `handle_input/2` execution time before the estimator transitions to `:degraded` with `:latency_overrun`.
- `lost_after` — no-input duration before the estimator transitions to `:lost`.
- `recover_after` — consecutive in-budget completions required to transition from `:degraded` to `:healthy`; inherited default is 1.
- `on_degraded` / `on_lost` / `on_recovered` — command names fired by the existing estimator transition machinery.

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
- `driver?: true` set on exactly one input for multi-input perceptors.
- Perceptor module exists and implements `BB.Estimator`.
- Output topic doesn't collide with any sensor path.
- `max_input_age`, `latency_budget`, `lost_after`, and `sync_tolerance` are positive duration units when present.
- Configured `on_degraded`/`on_lost`/`on_recovered` command names resolve to commands declared in the robot's `commands` section.
- Effective output paths are included in the estimator catalogue and no cycles exist in the combined estimator/perceptor dependency graph.

### Health transitions

Perception uses the core `BB.Estimator` transition state machine: `:healthy`, `:degraded`, and `:lost`, hysteresis-debounced recovery, timeout-based lost detection, and per-estimator transition commands. The recovery-accounting correction listed under runtime reuse is a generic core fix, not a perception-specific state machine. There is no separate health payload, channel, monitor, or perceptor health state.

The relevant estimator transition reasons are:

- `:latency_overrun` — callback execution exceeded `latency_budget`.
- `:stale_input` — the incoming local envelope exceeded `max_input_age`.
- `:sync_miss` — multi-input synchronisation exceeded `sync_tolerance` or an input was missing.
- `:lost` and `:recovered` — existing estimator timeout and recovery transitions.

Sample-store retention does not change estimator health. Store pressure is observable through store telemetry and logging.

The `allowed_states` mechanism on commands handles state-machine gating for free: a perceptor's `on_degraded` command can transition the robot to a degraded operational state, and other commands' `allowed_states` decide whether to permit operation while degraded. This is the same pattern as Proposal 0018 — perception just inherits it.

#### Process death

When supervisors give up restarting a perceptor, failure propagates through the existing robot supervision tree. There is no `:failed` health state: process death is an OTP concern, not a state-machine value. Consumers that need direct notification may monitor the registered estimator process.

### Relationship to bb_policy

Perception and policy share an integration boundary, not a runtime. A perceptor consumes messages and publishes derived messages continuously, including while the robot is disarmed. A policy consumes a validated observation snapshot and produces effects that are applied only while armed.

The companion `bb_policy` design plans to collect perception exactly as it collects any other message input: a policy declares named source paths through an optional `inputs/1` callback, the policy runtime caches the latest complete `BB.Message` envelope for each path, and `BB.Policy.observe/3` converts those envelopes into model-specific tensors. Payloads are not stripped at the package boundary because timestamps, source node, and frame identity are required for freshness and alignment checks. Joint-only policies declare no message inputs and retain the existing empty-map behaviour. This remains separate work under [bb_policy issue #9](https://github.com/beam-bots/bb_policy/issues/9), not an API shipped by this proposal.

That design makes missing, stale, cross-node, or misaligned required inputs prevent inference and new effect publication. Because previous velocity and effort commands may remain active when a process merely stops producing effects, invalidation after the policy has applied an effect requests a planned direct safety-originated disarm before a bounded policy terminates; a standing policy controller requests the same intervention. Perception health transitions remain independent and may invoke additional robot-specific commands.

Reusable decode, rotate, undistort, detection, and embedding stages remain perceptors. Processing tied to a particular policy's trained input contract — tensor layout, resizing, feature selection, and normalisation — remains in that policy. `bb_perception` does not invoke policies and `bb_policy` does not own perception supervision or sample retention.

### Telemetry

Perceptors emit core estimator events (`[:bb, :estimator, :input]`, `:output`, `:latency`, `:dropped`, and `:transition`) where the estimator server has the corresponding synchronous driver context. `bb_perception` does not duplicate them under a second namespace; asynchronous adapters own any additional end-to-end timing event.

The package owns one additional event for its storage concern:

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

  defmodule NoStore do
    use BB.Error, class: :invalid, fields: [:path]
    defimpl BB.Error.Severity, do: (def severity(_), do: :error)
  end
end
```

Stale-input, synchronisation, and unknown-output failures remain core estimator errors. Model-loading errors belong to the adapter that loads the model.

---

## Package structure

```
bb_perception/
├── lib/
│   ├── bb/
│   │   ├── perception.ex                       # top-level: publish/3 helper
│   │   ├── perception/
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
│   │   │   │   │   └── lower_perceptors.ex     # perceptor → estimator
│   │   │   │   └── verifiers/
│   │   │   ├── sample_store.ex                 # public API
│   │   │   ├── sample_store/
│   │   │   │   ├── server.ex                   # per-stream owner
│   │   │   │   ├── retention.ex
│   │   │   │   └── supervisor.ex
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
    {:bb, bb_dep("~> 0.23")},
    {:nx, "~> 0.12"},
    {:spark, "~> 2.0"},
    {:localize, "~> 0.37"},
    {:telemetry, "~> 1.0"}
  ]
end
```

Depends on `bb` for `BB.Estimator`, robot-level estimator wiring, pluggable output publication, intake freshness, and transition-state-machine mechanics. `~> 0.23` is the intended first core release containing those additions and must be adjusted if they ship under a different version. `nx` is a direct dependency because numeric payload schemas and the `FrameData` implementation use `Nx.Tensor`; core already carries the same runtime cost. `bb_perception` adds no inference backend. No `ortex`, `bumblebee`, `exla`, `evision`, or vendor NIFs — driver and adapter packages bring those.

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

The camera driver must publish frames on this store-backed path with `BB.Perception.publish/3`; direct `BB.PubSub.publish/3` would bypass the declared store and is unsupported.

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
    on_overrun :warn
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
    input :left,  [:sensor, :head, :camera_left], driver?: true
    input :right, [:sensor, :head, :camera_right]
    latency_budget ~u(100 millisecond)
    sync_tolerance ~u(20 millisecond)
    on_degraded :enter_degraded_mode
  end
end
```

### Commands gated by perception health

```elixir
states do
  state :degraded, doc: "Perception unhealthy; precision motion disabled"
end

commands do
  command :enter_degraded do
    handler MyApp.Commands.EnterDegraded   # transitions state machine to :degraded
    allowed_states [:idle, :executing]
  end

  command :recover_perception do
    handler MyApp.Commands.RecoverPerception  # transitions state machine back to :idle
    allowed_states [:degraded]
  end

  command :pick_object do
    handler MyApp.Commands.PickObject
    allowed_states [:idle]    # not :degraded — auto-gated by state machine
  end

  command :emergency_stop do
    handler MyApp.Commands.Stop
    allowed_states [:idle, :degraded, :executing]
  end
end

perceptors do
  perceptor :obstacle_detection, {BB.Perception.Vision.Obstacles, ...} do
    input [:sensor, :head, :lidar]
    latency_budget ~u(100 millisecond)

    on_degraded :enter_degraded
    on_recovered :recover_perception
  end
end
```

When `:obstacle_detection` degrades, the perceptor invokes `:enter_degraded`, which transitions the robot to `:degraded`. `pick_object` is now blocked by the existing `allowed_states` machinery; `emergency_stop` still works. When the perceptor recovers, `:recover_perception` runs and transitions the robot back to `:idle`. Policy is entirely the developer's — what counts as "degraded" and what to do about it are encoded in the commands, not the framework.

---

## Acceptance criteria

### Phase 1: Foundations

- [ ] `Image` and `LaserScan` removed from `bb`; equivalent payloads land in `bb_perception` with namespaced module names.
- [ ] Additional payload types: `PointCloud`, `Depth`, `Stereo`, `IR`, `Detections`, `Masks`, `Keypoints`, `DepthMap`, `Embedding`, `Classification`.
- [ ] Every payload schema specifies required/optional fields, tensor element types and ranks, cross-field shape invariants, units/encodings, and validation tests; no numeric list or binary convention is implicit.
- [ ] `BB.Perception.Dsl` extension loads cleanly via `use BB, extensions: [BB.Perception.Dsl]`.
- [ ] `streams` DSL section with `stream` entity, `retention` (union-typed, multi-occurrence, max one per kind), `on_overrun`.
- [ ] Compile-time verifier for `streams` (paths resolve via persisted topology cache, retentions present and well-typed, no duplicate kinds, `on_overrun` valid).
- [ ] `BB.Perception.SampleStore.Server` per declared stream, supervised, with ETS-backed storage, `insert_new` collision avoidance, and pluggable retention.
- [ ] ETS objects use `{monotonic_time, %BB.Message{}}`; public reads return the canonical envelope rather than the storage tuple.
- [ ] Retention tests cover count, stream-time duration, and byte caps independently and in combination (`any` cap triggers oldest-unpinned eviction).
- [ ] Duration retention does not age a stopped stream by wall time, and pinned samples remain exempt from ordinary eviction.
- [ ] Robot-scoped store supervisors start before topology producers through a generic extension-child hook and stop with their robot.
- [ ] Read API: `latest`, `fetch`, `nearest`, `before`, `after`, `range`, `since`, `pin`, `unpin`, `pinned`, `stats`.
- [ ] `BB.Perception.publish/3` stores synchronously before PubSub and returns a structured `NoStore` error for undeclared paths.
- [ ] Stored and delivered envelopes are structurally identical, including collision-adjusted `monotonic_time` and `robot`.
- [ ] Store insertion or retention failure publishes nothing.
- [ ] Pin pressure that prevents retention compliance rolls back the new sample and returns `StoreOverrun`.
- [ ] PubSub failure leaves the canonical envelope stored, surfaces the failure, and is documented as non-idempotent on retry.
- [ ] Local refc binary handling verified — no per-subscriber copies for large payloads (test via `:erlang.process_info(pid, :binary)`).
- [ ] `BB.Perception.FrameData` protocol with `to_iodata/1`, `byte_size/1`, `residency/1`; byte-aligned `BitString` and typed `Nx.Tensor` implementations.
- [ ] `frame_data_fields/0` optional callback added to `BB.Message` in `bb`; perception payloads list every bulk field.
- [ ] SampleStore byte-cap retention reads payload sizes via `BB.Perception.FrameData.byte_size/1`.

### Phase 2: Estimator integration and DSL

- [ ] Core supports robot-level estimators using `BB.Estimator.Server`.
- [ ] Robot-level estimators run under `BB.TopologySupervisor` and participate in robot-wide process-name uniqueness validation.
- [ ] Core estimator outputs support a generic `publish_with: {module, function}` pair, defaulting to `{BB.PubSub, :publish}`.
- [ ] Estimator publishers treat only `:ok` as success, stop on `{:error, reason}`, preserve raise semantics, and emit output telemetry only after success.
- [ ] Core estimators support `max_input_age` independently of callback `latency_budget`.
- [ ] Successful dispatch advances recovery even when no `latency_budget` is configured.
- [ ] Multi-input lost detection is driven by the driver input or successful dispatch, not auxiliary input traffic.
- [ ] Estimator latency telemetry uses documented nanosecond units consistently.
- [ ] Core estimator verification resolves effective output path overrides for input lookup and cycle detection.
- [ ] Explicit multi-output estimators do not retain an undeclared implicit `:out` route.
- [ ] `perceptors` DSL section lowers every `perceptor` to robot-level estimator configuration; no `BB.Perception.Perceptor.Server` exists.
- [ ] `perceptor` retains core estimator input, output, timing, health, parameter, and callback semantics while adding perception paths and inline store declarations.
- [ ] Compile-time verifier for perceptors covers path resolution, input form, driver constraint, output collisions, store declarations, command references, and combined estimator/perceptor cycles.
- [ ] Single-output perceptors publish to `[:perception, perceptor_name]` from reply tuples tagged `:out`.
- [ ] Multi-output perceptors publish to `[:perception, perceptor_name, name]` per declared output.
- [ ] Store-backed outputs use `BB.Perception.publish/3`; non-store-backed outputs use `BB.PubSub.publish/3`.
- [ ] Missing, stale, and misaligned inputs use core estimator drop telemetry and health transitions.
- [ ] At least one reference perceptor adapter exists demonstrating the contract.

### Phase 3: Transition commands

- [ ] Transition mechanics inherited from `BB.Estimator` (Proposal 0018).
- [ ] Estimator transition reasons surface in command metadata: `:latency_overrun`, `:stale_input`, `:sync_miss`, `:lost`, and `:recovered`.
- [ ] Perceptors emit applicable core estimator input, output, synchronous latency, dropped, and transition telemetry without duplicate perception events.
- [ ] Sample stores independently emit `[:bb, :perception, :store_overrun]` telemetry.

### Phase 4: Chaining & polish

- [ ] Perceptor chaining resolves effective `[:perception, ...]` output paths.
- [ ] Cycle detection covers the combined estimator/perceptor graph.
- [ ] Documentation: README, tutorial covering camera → decode → detect, tutorial covering LIDAR + SLAM-style pinning, tutorial covering accumulator and async-inference patterns.
- [ ] `mix.usage_rules` documentation.

### Won't have (deferred)

- [ ] Concrete device-resident `FrameData` implementations (HAILO/CUDA zero-copy). V1 includes host binaries and `Nx.Tensor`; sibling packages may add owning, BEAM-lifetime-managed device carriers conforming to the protocol.
- [ ] Cross-node sample store lookups (consumers `:erpc` explicitly if needed).
- [ ] Hot reload of model weights without restart (perceptors can implement via `handle_call/3` if they want it).
- [ ] Recorded replay support (separate `bb_replay` proposal).

---

## Open questions

1. **Pin ownership.** Explicit `unpin/3` is simple but cannot recover automatically when a caller dies. Should pins be leases associated with owner processes, or should v1 retain manual ownership with telemetry for leaked capacity?

2. **Cross-perceptor transition coordination.** Two perceptors can invoke conflicting transition commands concurrently. Existing command `allowed_states` prevents invalid transitions, but the first accepted command wins; coordinated health policy remains a possible follow-up.

---

## References

- [Proposal 0018: BB.Estimator](https://github.com/beam-bots/proposals/blob/main/implemented/0018-bb-estimator.md) — defines the implemented behaviour, runtime, and transition-state-machine mechanics perception uses. Its older references to a perception-specific server are superseded by this revision.
- [BB.Sensor behaviour](https://github.com/beam-bots/bb/blob/main/lib/bb/sensor.ex) — pattern for behaviour-based driver modules.
- [BB.Message](https://github.com/beam-bots/bb/blob/main/lib/bb/message.ex) — envelope carries `monotonic_time`, `wall_time`, `node`, `frame_id`, `robot`.
- [BB.PubSub](https://github.com/beam-bots/bb/blob/main/lib/bb/pub_sub.ex) — local hierarchical pubsub the perception channel is built on.
- [BB.Safety](https://github.com/beam-bots/bb/blob/main/lib/bb/safety.ex) — the contract perception deliberately does not extend.
- [Proposal 0003: bb_dataset](0003-bb-dataset.md) — adjacent concern; recording sample streams ties into perception.
- [yolo_elixir](https://github.com/poeticoding/yolo_elixir) — reference behaviour-based wrapper; informed the original behaviour shape.
- [Bumblebee + Nx.Serving](https://github.com/elixir-nx/bumblebee) — the batching/distribution model the adapter for it will wrap.
- [Ortex](https://github.com/elixir-nx/ortex) — ONNX runtime wrapper, the underlying inference engine for yolo_elixir and a candidate for a generic ONNX adapter.
- [ROS sensor_msgs](http://wiki.ros.org/sensor_msgs) — payload-type precedent (`Image`, `LaserScan`, `PointCloud2`).
- [ROS vision_msgs](http://wiki.ros.org/vision_msgs) — payload-type precedent for `Detections`, `Masks`, `Keypoints`.
- [ROS message_filters ApproximateTime](http://wiki.ros.org/message_filters) — multi-input synchronisation precedent.
- [Iceoryx zero-copy IPC](https://iceoryx.io/) — the model behind the (deferred) device-handle approach.
