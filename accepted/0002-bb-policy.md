<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal: bb_policy

**Status:** Draft
**Author:** James Harton
**Created:** 2026-01-11
**Dependencies:** `bb_teleop` (for demonstration collection), `bb_dataset` (for training data)

---

## Summary

`bb_policy` enables Beam Bots robots to execute learned behaviours—neural networks or other models that map observations to actions. Rather than explicitly programming every motion, operators can train policies from demonstrations or simulation, then deploy them on real hardware with full safety system integration.

---

## Motivation

### The Limits of Explicit Programming

Some tasks resist explicit programming:

| Task Type | Why It's Hard to Program |
|-----------|-------------------------|
| **Dexterous manipulation** | Too many contact states, forces, edge cases |
| **Unstructured environments** | Can't enumerate all object positions/orientations |
| **Generalisation** | New objects, new arrangements, variations |
| **Compliant motion** | Requires real-time adaptation to forces |
| **Human-like motion** | Smooth, natural trajectories are hard to specify |

For these tasks, we want robots that *learn* from examples rather than follow explicit rules.

### Why Learned Policies?

**Imitation learning** has proven effective for manipulation:
- ALOHA, Mobile ALOHA — bimanual manipulation from demonstrations
- ACT (Action Chunking with Transformers) — learns from ~50 demonstrations
- Diffusion Policy — handles multimodal action distributions
- RT-1, RT-2, OpenVLA — vision-language-action models

These approaches work. The question is: how do we deploy them on Beam Bots?

### Why a Separate Package?

1. **Not everyone needs ML** — Many robots work fine with explicit programming
2. **Heavy dependencies** — Ortex, Axon, potentially Bumblebee add significant weight
3. **Can evolve independently** — ML ecosystem moves fast
4. **Clear boundary** — If it becomes ubiquitous, we can consider moving to core

### The BEAM Advantage

Why run policies on BEAM rather than Python?

| Concern | Python | Elixir/BEAM |
|---------|--------|-------------|
| **Fault tolerance** | Manual, error-prone | Supervision trees |
| **Concurrency** | GIL limitations | Lightweight processes |
| **Hot reload** | Restart required | Update policies live |
| **Integration** | Separate process, IPC | Same runtime as control |
| **Observability** | External tooling | Built-in telemetry |

A crashed policy shouldn't crash the robot. Slow inference shouldn't block sensors. These properties come free with BEAM.

---

## Terminology

### What is a "Policy"?

In machine learning, a **policy** is a function that maps observations to actions:

```
π: observation → action
```

Given what the robot sees (joint positions, camera images, force readings), the policy outputs what the robot should do (target positions, velocities, gripper commands).

**Alternative names considered:**
- "Skill" — intuitive but vague
- "Behaviour" — conflicts with Elixir `@behaviour`
- "Controller" — conflicts with `BB.Controller`
- "Model" — too generic

"Policy" is the standard term in robotics/ML literature. Users working with learned behaviours will encounter it.

---

## Design

### Core Behaviour

```elixir
defmodule BB.Policy do
  @moduledoc """
  Behaviour for learned policies that map observations to actions.

  A policy is stateful—some architectures (RNNs, transformers with context)
  maintain hidden state across timesteps. The behaviour accounts for this.
  """

  @type observation :: %{atom() => Nx.Tensor.t()}
  @type action :: %{atom() => Nx.Tensor.t()}
  @type state :: term()

  @doc """
  Initialise the policy.

  Called once when the policy is loaded. Should load model weights,
  set up any required state, and validate configuration.
  """
  @callback init(opts :: keyword()) :: {:ok, state()} | {:error, term()}

  @doc """
  Reset policy state.

  Called at episode boundaries. Clears any accumulated hidden state.
  For stateless policies, this is a no-op.
  """
  @callback reset(state()) :: state()

  @doc """
  Construct observation tensor from robot state and sensor data.

  Handles observation space definition, normalisation, and tensor construction.
  """
  @callback observe(
              robot_state :: map(),
              sensors :: %{atom() => term()},
              state()
            ) :: {observation(), state()}

  @doc """
  Run inference to produce an action.

  Given an observation, returns the action to execute.
  May update internal state (for recurrent policies).
  """
  @callback act(observation(), state()) :: {action(), state()}

  @doc """
  Convert action tensor to robot commands.

  Handles action space definition, denormalisation, and command construction.
  Returns commands suitable for BB.Actuator.
  """
  @callback action_to_commands(action(), robot :: module(), state()) ::
              {:ok, [BB.Actuator.command()]} | {:error, term()}

  @doc """
  Optional: Return policy metadata for introspection.
  """
  @callback info(state()) :: map()

  @optional_callbacks [info: 1]
end
```

### Policy Runner

```elixir
defmodule BB.Policy.Runner do
  @moduledoc """
  Executes a policy in a control loop.

  Handles:
  - Observation collection from robot state and sensors
  - Policy inference at specified frequency
  - Action application to actuators
  - Safety monitoring and intervention
  - Telemetry and logging
  """

  use GenServer

  defstruct [
    :robot,
    :policy_module,
    :policy_state,
    :rate_hz,
    :safety_config,
    :goal,
    :episode_step
  ]

  @type goal :: term()

  def start_link(opts) do
    robot = Keyword.fetch!(opts, :robot)
    GenServer.start_link(__MODULE__, opts, name: via(robot, :policy_runner))
  end

  @doc """
  Run a policy until completion or timeout.
  """
  @spec run(robot :: module(), policy :: module(), goal(), keyword()) ::
          {:ok, result :: term()} | {:error, term()}
  def run(robot, policy_module, goal, opts \\ [])
end
```

### Normalisation

Policies expect normalised inputs and produce normalised outputs:

```elixir
defmodule BB.Policy.Normalizer do
  @moduledoc """
  Observation and action normalisation.

  Supports multiple normalisation strategies:
  - Min-max scaling to [0, 1] or [-1, 1]
  - Z-score normalisation (mean=0, std=1)
  - No normalisation (passthrough)

  Statistics can be:
  - Hardcoded from training dataset
  - Loaded from file
  - Computed online (for simple cases)
  """

  defstruct [:observation_stats, :action_stats, :strategy]

  @type stats :: %{
          atom() => %{
            mean: float() | Nx.Tensor.t(),
            std: float() | Nx.Tensor.t(),
            min: float() | Nx.Tensor.t(),
            max: float() | Nx.Tensor.t()
          }
        }

  @spec normalize(Nx.Tensor.t(), stats(), atom()) :: Nx.Tensor.t()
  def normalize(tensor, stats, key)

  @spec denormalize(Nx.Tensor.t(), stats(), atom()) :: Nx.Tensor.t()
  def denormalize(tensor, stats, key)
end
```

### ONNX Implementation

The primary deployment path—load models trained elsewhere:

```elixir
defmodule BB.Policy.ONNX do
  @moduledoc """
  Policy implementation that loads ONNX models via Ortex.

  This is the recommended way to deploy policies trained in Python
  (PyTorch, JAX, TensorFlow) on Beam Bots.

  ## Usage

      {:ok, runner} = BB.Policy.Runner.start_link(
        robot: MyRobot,
        policy: BB.Policy.ONNX,
        policy_opts: [
          model: "path/to/policy.onnx",
          normalizer: "path/to/stats.json",
          observation_keys: [:joint_positions, :joint_velocities],
          action_keys: [:target_positions]
        ],
        rate_hz: 20
      )

  ## ONNX Export from LeRobot

  Policies trained with LeRobot can be exported to ONNX:

      # In Python
      torch.onnx.export(policy.model, dummy_input, "policy.onnx")

  """

  @behaviour BB.Policy

  defstruct [
    :model,
    :serving,
    :normalizer,
    :observation_keys,
    :action_keys,
    :hidden_state
  ]

  @impl BB.Policy
  def init(opts) do
    model_path = Keyword.fetch!(opts, :model)
    model = Ortex.load(model_path)

    serving =
      Nx.Serving.new(Ortex.Serving, model)
      |> Nx.Serving.client_preprocessing(&preprocess/1)

    normalizer = load_normalizer(opts[:normalizer])

    {:ok,
     %__MODULE__{
       model: model,
       serving: serving,
       normalizer: normalizer,
       observation_keys: Keyword.fetch!(opts, :observation_keys),
       action_keys: Keyword.fetch!(opts, :action_keys)
     }}
  end

  @impl BB.Policy
  def reset(state) do
    %{state | hidden_state: nil}
  end

  @impl BB.Policy
  def observe(robot_state, sensors, state) do
    obs =
      state.observation_keys
      |> Enum.map(fn key -> {key, get_observation(key, robot_state, sensors)} end)
      |> Map.new()
      |> normalize_observation(state.normalizer)

    {obs, state}
  end

  @impl BB.Policy
  def act(observation, state) do
    input = observation_to_tensor(observation, state.observation_keys)
    output = Nx.Serving.batched_run(state.serving, input)
    action = tensor_to_action(output, state.action_keys)
    {action, state}
  end

  @impl BB.Policy
  def action_to_commands(action, robot, state) do
    denormalized = denormalize_action(action, state.normalizer)
    commands = build_actuator_commands(denormalized, robot)
    {:ok, commands}
  end
end
```

### Integration with BB.Motion

```elixir
defmodule BB.Motion do
  # ... existing functions ...

  @doc """
  Execute a learned policy.

  Runs the policy in a control loop until:
  - The policy signals completion
  - A timeout is reached
  - The safety system intervenes
  - The caller cancels

  ## Options

  - `:rate_hz` - Control loop frequency (default: 20)
  - `:timeout` - Maximum execution time (default: 30_000 ms)
  - `:policy_opts` - Options passed to policy init
  - `:goal` - Goal specification passed to policy

  ## Example

      BB.Motion.run_policy(
        MyRobot,
        BB.Policy.ONNX,
        %{target: :mug_pickup},
        policy_opts: [model: "pick_mug.onnx"],
        timeout: :timer.seconds(30)
      )

  """
  @spec run_policy(robot :: module(), policy :: module(), goal :: term(), keyword()) ::
          {:ok, result :: term()} | {:error, term()}
  def run_policy(robot, policy_module, goal, opts \\ []) do
    BB.Policy.Runner.run(robot, policy_module, goal, opts)
  end
end
```

### Safety Integration

Policies execute within the existing safety framework:

```elixir
defmodule BB.Policy.Runner do
  # In the control loop:

  defp run_step(state) do
    # Check safety before each step
    case BB.Safety.check(state.robot) do
      :ok ->
        {obs, policy_state} = state.policy_module.observe(robot_state, sensors, state.policy_state)
        {action, policy_state} = state.policy_module.act(obs, policy_state)

        case state.policy_module.action_to_commands(action, state.robot, policy_state) do
          {:ok, commands} ->
            # Safety system validates commands before execution
            :ok = apply_commands_safely(state.robot, commands)
            {:continue, %{state | policy_state: policy_state}}

          {:error, reason} ->
            {:stop, {:error, {:action_conversion, reason}}}
        end

      {:error, reason} ->
        {:stop, {:error, {:safety, reason}}}
    end
  end
end
```

---

## Package Structure

```
bb_policy/
├── lib/
│   └── bb/
│       └── policy/
│           ├── policy.ex           # BB.Policy behaviour
│           ├── runner.ex           # Control loop GenServer
│           ├── normalizer.ex       # Observation/action normalisation
│           ├── onnx.ex             # ONNX implementation via Ortex
│           ├── command.ex          # BB.Command wrapper
│           ├── serving.ex          # Nx.Serving utilities
│           └── telemetry.ex        # Telemetry events
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
    {:ortex, "~> 0.1"},   # ONNX runtime
    {:nx, "~> 0.10"}      # Already a bb dependency
  ]
end
```

---

## User Experience

### Basic Usage: ONNX Model

```elixir
# Run a policy trained in Python/LeRobot
{:ok, result} = BB.Motion.run_policy(
  MyRobot,
  BB.Policy.ONNX,
  %{task: :pick_mug},
  policy_opts: [
    model: "/path/to/policy.onnx",
    normalizer: "/path/to/normalizer.json",
    observation_keys: [:joint_positions, :joint_velocities, :gripper],
    action_keys: [:target_positions, :target_gripper]
  ],
  rate_hz: 20,
  timeout: :timer.seconds(30)
)
```

### With BB.Command

```elixir
# Define a learned command
defmodule MyRobot.Commands.PickMug do
  use BB.Policy.Command,
    policy: BB.Policy.ONNX,
    policy_opts: [
      model: "priv/models/pick_mug.onnx",
      normalizer: "priv/models/pick_mug_stats.json",
      observation_keys: [:joint_positions, :camera_image],
      action_keys: [:target_positions]
    ]
end

# Execute via command system
BB.Command.execute(MyRobot, MyRobot.Commands.PickMug, %{mug_location: :table})
```

### In Reactor Workflow

```elixir
# Mix learned and programmed behaviours
defmodule MyRobot.Workflows.MakeCoffee do
  use BB.Reactor

  step :move_to_machine, BB.Command.MoveTo, target: :coffee_machine
  step :pick_mug, MyRobot.Commands.PickMug  # Learned policy
  step :place_under_spout, BB.Command.MoveTo, target: :spout
  step :press_button, MyRobot.Commands.PressButton  # Learned policy
  step :wait, BB.Command.Wait, duration: :timer.seconds(30)
  step :pick_up_mug, MyRobot.Commands.PickMug
  step :deliver, BB.Command.MoveTo, target: :delivery_point
end
```

---

## Acceptance Criteria

### Must Have

- [ ] `BB.Policy` behaviour with `init/1`, `reset/1`, `observe/3`, `act/2`, `action_to_commands/3`
- [ ] `BB.Policy.Runner` GenServer executing control loop at configurable rate
- [ ] `BB.Policy.Normalizer` for observation/action normalisation (min-max and z-score)
- [ ] `BB.Policy.ONNX` implementation loading models via Ortex
- [ ] Integration with `BB.Motion.run_policy/4`
- [ ] Safety system integration (joint limits, velocity limits enforced)
- [ ] Timeout handling (policies don't run forever)
- [ ] Basic telemetry (inference time, step count)
- [ ] Documentation with ONNX export examples
- [ ] Tests for behaviour contract, runner lifecycle, normalisation

### Should Have

- [ ] `BB.Policy.Command` wrapper for reactor integration
- [ ] Normalisation statistics loading from JSON file
- [ ] GPU acceleration configuration (CUDA via Ortex)
- [ ] Graceful degradation on inference failure
- [ ] Episode reset handling (clear hidden state)
- [ ] Example: simple ONNX policy running on simulated robot

### Won't Have

- [ ] Native Axon policies (separate `bb_policy_axon`)
- [ ] Diffusion policy implementation (separate `bb_policy_diffusion`)
- [ ] Training loops (separate `bb_training`)
- [ ] Dataset management (separate `bb_dataset`)
- [ ] Vision encoders (separate `bb_vision`)
- [ ] Python bridge (separate `bb_policy_pythonx`)

---

## Open Questions

1. **Observation sources:** Should `observe/3` receive raw sensor messages, or pre-processed robot state? How do we handle camera images efficiently?

2. **Action representation:** Joint positions? Velocities? Deltas from current position? Should this be configurable per-policy?

3. **Episode boundaries:** How does the runner know when an episode ends? Timeout only? Policy returns `:done`? External signal?

4. **Goal specification:** How are goals passed to policies? Separate input tensor? Part of observation? Policy-specific?

5. **Multi-step actions:** For action chunking (ACT), should the runner execute all predicted actions, or re-plan after each? What's the right abstraction?

6. **Vision input:** Camera images require preprocessing (resize, crop, normalise). Should this be in `observe/3`, or a separate vision pipeline?

7. **Recurrent policies:** How do we handle LSTM/transformer context? Store in policy state? How much history?

---

## References

- [LeRobot](https://github.com/huggingface/lerobot) — Hugging Face robotics library
- [ACT](https://tonyzhaozh.github.io/aloha/) — Action Chunking with Transformers
- [Diffusion Policy](https://diffusion-policy.cs.columbia.edu/) — Diffusion models for robotics
- [Ortex](https://github.com/elixir-nx/ortex) — ONNX Runtime for Elixir
- [Nx.Serving](https://hexdocs.pm/nx/Nx.Serving.html) — Model serving documentation
- [ONNX](https://onnx.ai/) — Open Neural Network Exchange format
