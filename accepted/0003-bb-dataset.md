<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal: bb_dataset

**Status:** Draft
**Author:** James Harton
**Created:** 2026-01-11
**Dependencies:** `bb_teleop` (for demonstration collection)
**Dependents:** `bb_policy` (consumes datasets for deployment), `bb_training` (future)

---

## Summary

`bb_dataset` provides infrastructure for recording, storing, and exporting robot demonstration data. It captures observations, actions, and metadata during teleoperation sessions, enabling the train-deploy workflow for learned policies.

---

## Motivation

### The Data Collection Problem

Training learned policies requires demonstration data—recordings of successful task executions. This creates several challenges:

1. **During recording:** Data must be captured reliably without dropping frames or introducing latency that affects teleoperation feel.

2. **Storage:** Demonstrations accumulate. A single training dataset might contain hundreds of episodes, each with thousands of timesteps of sensor data and video frames.

3. **Iteration:** Operators need to review, edit, and curate datasets. Bad demonstrations should be deletable. Episodes might need trimming.

4. **Export:** Training typically happens in Python (LeRobot, PyTorch). Data must be exportable to formats those tools understand.

5. **Metadata:** Each episode needs context—which robot, which task, success/failure, operator notes.

### Why a Separate Package?

- **Independent value:** Dataset tooling is useful even without `bb_policy` (analysis, playback, sharing)
- **Storage dependencies:** Will depend on database/file libraries
- **Export dependencies:** Will depend on Parquet/video encoding libraries
- **Evolving requirements:** Dataset formats and tools change frequently

### The Workflow

```
Teleoperation (bb_teleop)
         │
         ▼
    Recording (bb_dataset)
         │
         ▼
    Local Storage
         │
         ├──► Review & Curate (Livebook, CLI)
         │
         ▼
    Export (Parquet, video)
         │
         ▼
    Training (LeRobot / Python)
         │
         ▼
    Deploy (bb_policy)
```

---

## Design

### Core Concepts

**Episode:** A single demonstration recording—one task execution from start to finish.

**Step:** A single timestep within an episode—one observation-action pair.

**Dataset:** A collection of episodes, typically for one task or robot configuration.

**Frame:** A single sensor reading (joint positions, camera image, force reading).

### Data Model

```elixir
defmodule BB.Dataset.Episode do
  @moduledoc """
  A single demonstration episode.
  """

  @type t :: %__MODULE__{
          id: String.t(),
          dataset_id: String.t(),
          robot: module(),
          task: String.t() | nil,
          started_at: DateTime.t(),
          ended_at: DateTime.t() | nil,
          success: boolean() | nil,
          notes: String.t() | nil,
          metadata: map(),
          step_count: non_neg_integer()
        }

  defstruct [
    :id,
    :dataset_id,
    :robot,
    :task,
    :started_at,
    :ended_at,
    :success,
    :notes,
    :metadata,
    step_count: 0
  ]
end

defmodule BB.Dataset.Step do
  @moduledoc """
  A single timestep within an episode.
  """

  @type t :: %__MODULE__{
          episode_id: String.t(),
          index: non_neg_integer(),
          timestamp_ns: integer(),
          observation: map(),
          action: map() | nil,
          reward: float() | nil
        }

  defstruct [
    :episode_id,
    :index,
    :timestamp_ns,
    :observation,
    :action,
    :reward
  ]
end
```

### Recording

```elixir
defmodule BB.Dataset.Recorder do
  @moduledoc """
  Records demonstration episodes from robot teleoperation.

  Subscribes to robot PubSub and captures:
  - Joint positions and velocities
  - Actuator commands (as actions)
  - Sensor readings (cameras, force sensors, etc.)
  - Timestamps for synchronisation

  ## Usage

      # Start recording
      {:ok, recorder} = BB.Dataset.Recorder.start_link(
        robot: MyRobot,
        dataset: "mug_pickup",
        task: "pick_mug_from_table"
      )

      # ... perform teleoperation ...

      # Stop and save episode
      {:ok, episode} = BB.Dataset.Recorder.stop(recorder, success: true)

  """

  use GenServer

  @type observation_source ::
          {:joint_positions, [atom()]}
          | {:joint_velocities, [atom()]}
          | {:sensor, atom()}
          | {:camera, atom()}
          | {:custom, (map() -> term())}

  defstruct [
    :robot,
    :dataset_id,
    :episode,
    :storage,
    :observation_sources,
    :action_source,
    :buffer,
    :rate_hz
  ]

  def start_link(opts)
  def stop(recorder, opts \\ [])
  def mark_success(recorder, success)
  def add_note(recorder, note)
  def cancel(recorder)
end
```

### Storage Abstraction

Storage format is intentionally left as an open design question. The API abstracts over the underlying implementation:

```elixir
defmodule BB.Dataset.Storage do
  @moduledoc """
  Behaviour for dataset storage backends.

  The storage backend handles persistence of episodes and steps.
  Different backends may be appropriate for different use cases:
  - Development: simple file-based storage
  - Production: database-backed storage
  - Large datasets: optimised binary formats

  The specific backend choice is deferred to implementation.
  """

  @callback init(opts :: keyword()) :: {:ok, state :: term()} | {:error, term()}

  @callback create_episode(episode :: Episode.t(), state) ::
              {:ok, Episode.t(), state} | {:error, term()}

  @callback append_step(step :: Step.t(), state) ::
              {:ok, state} | {:error, term()}

  @callback finalise_episode(episode_id :: String.t(), metadata :: map(), state) ::
              {:ok, Episode.t(), state} | {:error, term()}

  @callback list_episodes(dataset_id :: String.t(), state) ::
              {:ok, [Episode.t()]} | {:error, term()}

  @callback get_episode(episode_id :: String.t(), state) ::
              {:ok, Episode.t()} | {:error, :not_found}

  @callback stream_steps(episode_id :: String.t(), state) ::
              {:ok, Enumerable.t()} | {:error, term()}

  @callback delete_episode(episode_id :: String.t(), state) ::
              :ok | {:error, term()}
end
```

### Export

```elixir
defmodule BB.Dataset.Export do
  @moduledoc """
  Export datasets to training-compatible formats.
  """

  @doc """
  Export a dataset to Parquet format (LeRobot compatible).

  Creates:
  - `data/` directory with Parquet files
  - `videos/` directory with MP4 files (if camera data present)
  - `meta/` directory with dataset metadata

  ## Options

  - `:output_dir` - Output directory (required)
  - `:video_codec` - Video codec for encoding (default: "libx264")
  - `:video_fps` - Frame rate for video (default: 30)
  - `:include_videos` - Whether to export videos (default: true)

  """
  @spec to_parquet(dataset_id :: String.t(), opts :: keyword()) ::
          {:ok, path :: String.t()} | {:error, term()}
  def to_parquet(dataset_id, opts)

  @doc """
  Export a dataset to a simple JSON format for inspection.
  """
  @spec to_json(dataset_id :: String.t(), opts :: keyword()) ::
          {:ok, path :: String.t()} | {:error, term()}
  def to_json(dataset_id, opts)
end
```

### Mix Tasks

```
# List datasets and episodes
mix bb.dataset.list
mix bb.dataset.list mug_pickup

# Show episode details
mix bb.dataset.show mug_pickup/episode_001

# Delete an episode
mix bb.dataset.delete mug_pickup/episode_003

# Export to Parquet
mix bb.dataset.export mug_pickup --format parquet --output ./export/

# Compute statistics
mix bb.dataset.stats mug_pickup

# Replay an episode (outputs commands that would be sent)
mix bb.dataset.replay mug_pickup/episode_001 --dry-run
```

### Livebook Integration

```elixir
# In Livebook, visualise an episode
alias BB.Dataset.{Episode, Step, Viz}

episode = BB.Dataset.get_episode!("mug_pickup/episode_001")

# Plot joint trajectories
Viz.plot_joints(episode, [:shoulder, :elbow, :wrist])

# Show video frames (if available)
Viz.show_frames(episode, :camera_wrist, sample_rate: 10)

# Interactive scrubber
Viz.episode_player(episode)
```

---

## Package Structure

```
bb_dataset/
├── lib/
│   └── bb/
│       └── dataset/
│           ├── dataset.ex          # Main API
│           ├── episode.ex          # Episode struct
│           ├── step.ex             # Step struct
│           ├── recorder.ex         # Recording GenServer
│           ├── storage.ex          # Storage behaviour
│           ├── storage/
│           │   └── (implementations TBD)
│           ├── export.ex           # Export functionality
│           ├── export/
│           │   ├── parquet.ex      # Parquet export
│           │   └── json.ex         # JSON export
│           ├── stats.ex            # Statistics computation
│           └── replay.ex           # Episode replay
├── lib/mix/
│   └── tasks/
│       └── bb/
│           └── dataset/
│               ├── list.ex
│               ├── show.ex
│               ├── delete.ex
│               ├── export.ex
│               ├── stats.ex
│               └── replay.ex
├── test/
├── mix.exs
├── README.md
└── CHANGELOG.md
```

### Dependencies

```elixir
# mix.exs
defp deps do
  [
    {:bb, "~> 0.13"},
    {:explorer, "~> 0.10"},      # Parquet I/O
    # Storage backend TBD - may include:
    # {:cubdb, "~> 2.0"},        # Embedded database option
    # Or other storage solutions
  ]
end
```

---

## Acceptance Criteria

### Must Have

- [ ] `BB.Dataset.Episode` and `BB.Dataset.Step` data structures
- [ ] `BB.Dataset.Recorder` that subscribes to robot PubSub
- [ ] `BB.Dataset.Storage` behaviour defining storage interface
- [ ] At least one storage backend implementation
- [ ] Recording captures: joint positions, joint velocities, timestamps
- [ ] Episode metadata: robot, task, start/end time, success flag
- [ ] `BB.Dataset.list_episodes/1` and `BB.Dataset.get_episode/1`
- [ ] `BB.Dataset.delete_episode/1`
- [ ] `mix bb.dataset.list` and `mix bb.dataset.show` tasks
- [ ] Basic documentation and usage examples
- [ ] Tests for recorder, storage, and data structures

### Should Have

- [ ] `BB.Dataset.Export.to_parquet/2` for LeRobot compatibility
- [ ] `mix bb.dataset.export` task
- [ ] Camera frame capture (stored as binary, not encoded)
- [ ] `BB.Dataset.Stats` for computing observation/action statistics
- [ ] Episode replay (stream steps with timing)
- [ ] Configurable observation sources

### Won't Have

- [ ] Cloud storage integration
- [ ] Dataset versioning/lineage
- [ ] Real-time streaming to remote
- [ ] Training loops (future `bb_training`)
- [ ] Automatic data augmentation

---

## Open Questions

### Storage Format (Discovery Required)

The internal storage format is intentionally unspecified. Questions to answer during implementation:

1. **Structured data (steps, metadata):**
   - DETS: Built-in, but 2GB limit, single writer
   - CubDB: Pure Elixir, concurrent, no size limit
   - SQLite: Proven, queryable, but external dependency
   - ETS + periodic flush: Fast, but crash-unsafe

2. **Binary data (camera frames):**
   - Separate files per frame?
   - Chunked binary files?
   - Embedded in structured storage?

3. **Separation of concerns:**
   - Same storage for metadata and frames?
   - Separate storage with references?

4. **Performance characteristics:**
   - Write latency during recording (must not affect teleop)
   - Read performance for replay
   - Export performance for large datasets

**Recommendation:** Prototype multiple approaches early. Profile with realistic data volumes.

### Camera Frame Handling

- What resolution/format to capture?
- Compress during recording or only on export?
- How to handle multiple cameras?
- Synchronisation between cameras and joint state?

### LeRobot Compatibility

- Which LeRobot dataset schema version to target?
- How to handle schema differences between LeRobot versions?
- Should we support import as well as export?

### Episode Boundaries

- How does recorder know when episode starts/ends?
- Automatic detection vs explicit start/stop?
- How to handle interrupted recordings?

### Multi-Robot Datasets

- Can one dataset contain episodes from different robots?
- How to handle different observation spaces?

---

## Prior Art

### LeRobot Dataset Format

LeRobot uses:
- Parquet files for tabular data (observations, actions)
- MP4 video files for camera data
- JSON metadata files
- HuggingFace Datasets integration

**Learnings:** Parquet is well-suited for ML training. Separating video from structured data is pragmatic.

### ROS Bag Files

ROS uses bag files:
- Single file containing all message streams
- Efficient binary format
- Built-in replay with timing

**Learnings:** Single-file format is convenient but harder to edit/inspect.

### D4RL / Minari

Gymnasium ecosystem uses:
- HDF5 for numerical data
- Standardised observation/action spaces
- Dataset versioning

**Learnings:** Standardised schemas enable dataset sharing.

---

## References

- [LeRobot Dataset Format](https://github.com/huggingface/lerobot/tree/main/lerobot/common/datasets)
- [HuggingFace Datasets](https://huggingface.co/docs/datasets/)
- [Apache Parquet](https://parquet.apache.org/)
- [Explorer (Elixir DataFrames)](https://github.com/elixir-explorer/explorer)
- [CubDB](https://github.com/lucaong/cubdb)
- [ROS Bag Files](http://wiki.ros.org/Bags)
- [Minari (Gymnasium Datasets)](https://minari.farama.org/)
