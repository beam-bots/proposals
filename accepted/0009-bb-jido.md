<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal: bb_jido

**Status:** Draft
**Author:** James Harton
**Created:** 2026-01-20
**Dependencies:** `bb_reactor` (optional, for workflow integration)

---

## Summary

`bb_jido` integrates the [Jido](https://github.com/agentjido/jido) autonomous agent framework with Beam Bots, enabling robots to make goal-directed decisions rather than just execute pre-defined sequences. Where `bb_reactor` answers "how do I execute this workflow?", Jido agents answer "what should I do next to achieve this goal?"

---

## Motivation

### The Limits of Explicit Workflows

`bb_reactor` excels at structured task sequences—pick-and-place, calibration, assembly. You declare the steps, dependencies, and compensation logic. The reactor executes them.

But some scenarios resist pre-declaration:

| Scenario | Why Reactors Are Awkward |
|----------|--------------------------|
| "Pick up the red block" | Need to locate, select grasp strategy, plan approach—decisions depend on perception |
| Multi-robot coordination | Robots must negotiate tasks dynamically, not follow fixed scripts |
| Recovery from unexpected states | When assumptions fail, you need adaptive replanning, not just compensation |
| Human-robot collaboration | Human actions are unpredictable; robot must adapt in real-time |
| Voice/LLM-directed tasks | Natural language goals decompose differently based on context |

These require a higher-level abstraction: **goal-directed agents that decide what to do**.

### Why Jido?

[Jido](https://agentjido.xyz/) (自動 - "automatic") is an Elixir framework for autonomous agent systems. Key properties:

1. **Event-driven, not tick-based** — Like `bb_reactor`, Jido uses signals and message passing. No polling loops. This aligns with our critique of behaviour trees.

2. **Pure agent logic** — Agents are immutable data structures. Side effects are described as directives, executed by a runtime. This matches Elixir idioms.

3. **Composable actions** — Actions are validated, introspectable units of work. Similar to reactor steps but designed for dynamic selection.

4. **AI-optional** — Core has zero LLM code. `jido_ai` is separate. You can use classical planning, decision trees, or LLMs—it's agnostic.

5. **Production-ready** — Built on OTP patterns, with supervision, fault tolerance, and telemetry.

### The Layered Architecture

Jido doesn't replace `bb_reactor`—it sits above it:

```
┌─────────────────────────────────────────────────┐
│  Jido Agent                                     │
│  "Achieve goal: assemble widget"                │
│  - Observes world state via BB sensors          │
│  - Selects strategy based on context            │
│  - Invokes workflows or commands                │
├─────────────────────────────────────────────────┤
│  bb_reactor Workflows                           │
│  PickAndPlace, Calibrate, ReturnHome            │
│  - Structured sequences with compensation       │
│  - Compile-time validated                       │
├─────────────────────────────────────────────────┤
│  BB Commands                                    │
│  move_to_pose, close_gripper, home              │
│  - Direct robot control                         │
└─────────────────────────────────────────────────┘
```

The agent decides "I need to pick up part A", then invokes `PickAndPlace` reactor with appropriate inputs. If perception reveals the part isn't where expected, the agent adapts—perhaps searching, or asking for help.

### Why a Separate Package?

1. **Not everyone needs agents** — Many robots work fine with explicit workflows
2. **Additional dependency** — Jido adds weight some projects don't need
3. **Can evolve independently** — Agent patterns are still emerging
4. **Clear integration boundary** — BB provides robotics primitives; Jido provides agent orchestration

---

## Design

### Core Integration: BB.Jido.Action

Wrap BB functionality as Jido Actions:

```elixir
defmodule BB.Jido.Action.Command do
  @moduledoc """
  Jido Action that executes a BB command.

  Bridges Jido's action system to BB's command infrastructure.
  """

  use Jido.Action,
    name: "bb_command",
    description: "Execute a Beam Bots command",
    schema: [
      robot: [type: :atom, required: true, doc: "Robot module"],
      command: [type: :atom, required: true, doc: "Command name"],
      goal: [type: :map, default: %{}, doc: "Command arguments"]
    ]

  @impl Jido.Action
  def run(%{robot: robot, command: command, goal: goal}, _context) do
    case apply(robot, command, [goal]) do
      {:ok, pid} ->
        ref = Process.monitor(pid)
        await_command(pid, ref)

      {:error, reason} ->
        {:error, reason}
    end
  end

  defp await_command(pid, ref) do
    receive do
      {:DOWN, ^ref, :process, ^pid, :normal} ->
        {:ok, %{status: :completed}}

      {:DOWN, ^ref, :process, ^pid, :disarmed} ->
        {:error, :safety_disarmed}

      {:DOWN, ^ref, :process, ^pid, reason} ->
        {:error, {:command_failed, reason}}
    end
  end
end
```

### Reactor as Action

Run entire reactor workflows as single Jido actions:

```elixir
defmodule BB.Jido.Action.Reactor do
  @moduledoc """
  Jido Action that runs a BB.Reactor workflow.

  Enables agents to invoke structured workflows as atomic operations.
  """

  use Jido.Action,
    name: "bb_reactor",
    description: "Execute a Beam Bots reactor workflow",
    schema: [
      robot: [type: :atom, required: true],
      reactor: [type: :atom, required: true, doc: "Reactor module"],
      inputs: [type: :map, default: %{}, doc: "Reactor inputs"]
    ]

  @impl Jido.Action
  def run(%{robot: robot, reactor: reactor, inputs: inputs}, _context) do
    context = %{private: %{bb_robot: robot}}

    case Reactor.run(reactor, inputs, context) do
      {:ok, result} ->
        {:ok, %{reactor: reactor, result: result}}

      {:error, errors} ->
        {:error, {:reactor_failed, errors}}

      {:halt, reason} ->
        {:error, {:reactor_halted, reason}}
    end
  end
end
```

### BB Sensor as Jido Sensor

Bridge BB's PubSub events to Jido signals:

```elixir
defmodule BB.Jido.Sensor do
  @moduledoc """
  Jido Sensor that bridges BB.PubSub events to Jido signals.

  Subscribes to BB event topics and emits corresponding Jido signals,
  allowing agents to react to robot state changes.
  """

  use Jido.Sensor,
    name: "bb_pubsub_sensor",
    description: "Bridges BB.PubSub to Jido signals",
    schema: [
      robot: [type: :atom, required: true],
      topics: [type: {:list, {:list, :atom}}, required: true, doc: "BB.PubSub paths to subscribe"]
    ]

  @impl Jido.Sensor
  def mount(opts) do
    robot = Keyword.fetch!(opts, :robot)
    topics = Keyword.fetch!(opts, :topics)

    for topic <- topics do
      BB.PubSub.subscribe(robot, topic)
    end

    {:ok, %{robot: robot, topics: topics}}
  end

  @impl Jido.Sensor
  def handle_info({:bb_pubsub, path, message}, state) do
    signal = Jido.Signal.new(
      "bb.#{Enum.join(path, ".")}",
      %{
        robot: state.robot,
        path: path,
        message: message,
        timestamp: DateTime.utc_now()
      }
    )

    {:noreply, emit(state, signal)}
  end
end
```

### Robot Agent Definition

Define agents that control robots:

```elixir
defmodule BB.Jido.Agent do
  @moduledoc """
  Behaviour for Jido agents that control Beam Bots robots.

  Extends Jido.Agent with robotics-specific conveniences:
  - Automatic BB sensor registration
  - Safety state awareness
  - Standard robot actions pre-registered
  """

  defmacro __using__(opts) do
    robot = Keyword.fetch!(opts, :robot)

    quote do
      use Jido.Agent,
        name: unquote(opts[:name] || "bb_agent"),
        description: unquote(opts[:description] || "Beam Bots robot agent"),
        actions: [
          BB.Jido.Action.Command,
          BB.Jido.Action.Reactor,
          BB.Jido.Action.WaitForState,
          BB.Jido.Action.GetJointState
          | unquote(opts[:actions] || [])
        ],
        schema: [
          robot: [type: :atom, default: unquote(robot)],
          safety_state: [type: :atom, default: :unknown]
          | unquote(opts[:schema] || [])
        ]

      @robot unquote(robot)

      def robot, do: @robot
    end
  end
end
```

### Example: Pick and Place Agent

```elixir
defmodule MyRobot.Agents.Manipulator do
  @moduledoc """
  Agent that performs manipulation tasks.

  Given a goal like "pick up the red block", the agent:
  1. Queries perception for object location
  2. Selects appropriate grasp strategy
  3. Executes pick-and-place workflow
  4. Verifies success or retries with different approach
  """

  use BB.Jido.Agent,
    robot: MyRobot,
    name: "manipulator",
    actions: [
      MyRobot.Actions.LocateObject,
      MyRobot.Actions.SelectGrasp,
      MyRobot.Actions.VerifyGrip
    ]

  def handle_instruction(%{action: :pick_object, target: target}, agent) do
    with {:ok, location} <- locate(agent, target),
         {:ok, grasp} <- select_grasp(agent, target, location),
         {:ok, _} <- execute_pick(agent, location, grasp),
         {:ok, _} <- verify_grip(agent) do
      {:ok, agent, [Jido.Directive.emit("object.picked", %{target: target})]}
    else
      {:error, :object_not_found} ->
        {:ok, agent, [Jido.Directive.emit("object.not_found", %{target: target})]}

      {:error, :grasp_failed} ->
        {:retry, agent, %{strategy: :alternative}}

      {:error, reason} ->
        {:error, reason}
    end
  end

  defp locate(agent, target) do
    Jido.Agent.run_action(agent, MyRobot.Actions.LocateObject, %{target: target})
  end

  defp select_grasp(agent, target, location) do
    Jido.Agent.run_action(agent, MyRobot.Actions.SelectGrasp, %{
      target: target,
      location: location
    })
  end

  defp execute_pick(agent, location, grasp) do
    Jido.Agent.run_action(agent, BB.Jido.Action.Reactor, %{
      robot: agent.state.robot,
      reactor: MyRobot.Reactor.PickAndPlace,
      inputs: %{pick_pose: location, grasp_strategy: grasp}
    })
  end

  defp verify_grip(agent) do
    Jido.Agent.run_action(agent, MyRobot.Actions.VerifyGrip, %{})
  end
end
```

### Multi-Robot Coordination

Jido signals enable robot-to-robot coordination:

```elixir
defmodule MyFleet.Agents.Coordinator do
  @moduledoc """
  Agent that coordinates multiple robots.

  Receives tasks, allocates to available robots, monitors progress.
  """

  use Jido.Agent,
    name: "fleet_coordinator",
    actions: [
      MyFleet.Actions.AllocateTask,
      MyFleet.Actions.CheckRobotStatus
    ]

  def handle_signal(%Jido.Signal{type: "robot.task.completed"} = signal, agent) do
    robot_id = signal.data.robot_id
    task_id = signal.data.task_id

    agent = update_task_status(agent, task_id, :completed)

    case get_next_task(agent, robot_id) do
      {:ok, next_task} ->
        {:ok, agent, [
          Jido.Directive.emit("robot.task.assigned", %{
            robot_id: robot_id,
            task: next_task
          })
        ]}

      :no_tasks ->
        {:ok, agent, []}
    end
  end

  def handle_signal(%Jido.Signal{type: "robot.error"} = signal, agent) do
    robot_id = signal.data.robot_id
    error = signal.data.error

    handle_robot_error(agent, robot_id, error)
  end
end
```

### Safety Integration

Agents must respect the BB safety system:

```elixir
defmodule BB.Jido.Action.SafetyAware do
  @moduledoc """
  Mixin for actions that should check safety state.
  """

  defmacro __using__(_opts) do
    quote do
      @before_compile BB.Jido.Action.SafetyAware
    end
  end

  defmacro __before_compile__(_env) do
    quote do
      defoverridable run: 2

      def run(params, context) do
        robot = Map.get(params, :robot) || Map.get(context, :robot)

        case BB.Safety.state(robot) do
          :armed ->
            super(params, context)

          state when state in [:disarmed, :error] ->
            {:error, {:safety_not_armed, state}}
        end
      end
    end
  end
end
```

---

## Package Structure

```
bb_jido/
├── lib/
│   └── bb/
│       └── jido/
│           ├── action/
│           │   ├── command.ex        # BB.Command wrapper
│           │   ├── reactor.ex        # BB.Reactor wrapper
│           │   ├── wait_for_state.ex # Wait for robot state
│           │   ├── get_joint_state.ex# Read joint positions
│           │   └── safety_aware.ex   # Safety checking mixin
│           ├── sensor.ex             # BB.PubSub → Jido signal bridge
│           ├── agent.ex              # BB.Jido.Agent macro
│           └── telemetry.ex          # Telemetry events
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
    {:jido, "~> 1.2"},
    # Optional: for AI-driven planning
    {:jido_ai, "~> 1.0", optional: true}
  ]
end
```

---

## User Experience

### Simple Agent

```elixir
# Define an agent for your robot
defmodule MyRobot.Agent do
  use BB.Jido.Agent, robot: MyRobot

  def handle_instruction(%{action: :home}, agent) do
    result = Jido.Agent.run_action(agent, BB.Jido.Action.Command, %{
      robot: MyRobot,
      command: :home,
      goal: %{}
    })

    case result do
      {:ok, _} -> {:ok, agent, []}
      {:error, reason} -> {:error, reason}
    end
  end
end

# Start and instruct
{:ok, agent} = MyRobot.Agent.start_link()
:ok = Jido.Agent.instruct(agent, %{action: :home})
```

### With AI Planning (jido_ai)

```elixir
# Agent that uses LLM for task decomposition
defmodule MyRobot.SmartAgent do
  use BB.Jido.Agent,
    robot: MyRobot,
    actions: [
      BB.Jido.Action.Command,
      BB.Jido.Action.Reactor,
      MyRobot.Actions.LocateObject,
      MyRobot.Actions.InspectObject
    ]

  def handle_instruction(%{natural_language: text}, agent) do
    case Jido.AI.plan(agent, text) do
      {:ok, plan} ->
        execute_plan(agent, plan)

      {:error, reason} ->
        {:error, {:planning_failed, reason}}
    end
  end
end

# Usage
Jido.Agent.instruct(agent, %{
  natural_language: "Pick up the red block and place it in the bin"
})
```

### Supervised in Application

```elixir
defmodule MyApp.Application do
  use Application

  def start(_type, _args) do
    children = [
      MyRobot,
      {MyRobot.Agent, robot: MyRobot}
    ]

    Supervisor.start_link(children, strategy: :one_for_one)
  end
end
```

---

## Acceptance Criteria

### Must Have

- [ ] `BB.Jido.Action.Command` - Execute BB commands as Jido actions
- [ ] `BB.Jido.Action.Reactor` - Run BB.Reactor workflows as Jido actions
- [ ] `BB.Jido.Sensor` - Bridge BB.PubSub events to Jido signals
- [ ] `BB.Jido.Agent` - Macro for defining robot-controlling agents
- [ ] Safety integration - Actions respect BB.Safety state
- [ ] Documentation with examples
- [ ] Tests for action execution and signal bridging

### Should Have

- [ ] `BB.Jido.Action.WaitForState` - Wait for robot state machine transition
- [ ] `BB.Jido.Action.GetJointState` - Read current joint positions
- [ ] `BB.Jido.Action.SafetyAware` - Mixin for safety-checking actions
- [ ] Example: simple agent controlling simulated robot
- [ ] Telemetry integration

### Won't Have

- [ ] LLM/AI integration (use `jido_ai` directly)
- [ ] Perception/vision (separate concern)
- [ ] Multi-robot discovery (application-level concern)
- [ ] Custom Jido action implementations beyond BB wrappers

---

## Open Questions

1. **Agent lifecycle:** Should agents be supervised per-robot, or can one agent control multiple robots? What's the right supervision strategy?

2. **Signal filtering:** BB.PubSub can be high-volume (joint states at 100Hz). How do we efficiently filter which events become Jido signals?

3. **State synchronisation:** How does the agent's internal state stay synchronised with actual robot state? Polling? Event-driven updates?

4. **Error boundaries:** When a Jido action fails, how does that interact with BB's error reporting and safety system?

5. **Reactor compensation:** If an agent runs a reactor and it fails mid-way, should compensation happen automatically, or should the agent decide?

6. **jido_ai integration:** For LLM-driven planning, what observation format works best? How do we expose robot capabilities to the planner?

7. **Testing:** How do we test agent behaviour in isolation? Mock BB commands? Simulated robot?

---

## References

- [Jido on Hex](https://hex.pm/packages/jido) — Autonomous agent framework
- [Agent Jido Website](https://agentjido.xyz/) — Documentation and guides
- [Jido GitHub](https://github.com/agentjido/jido) — Source code
- [jido_ai](https://hex.pm/packages/jido_ai) — LLM integration for Jido
- [Elixir Forum Discussion](https://elixirforum.com/t/jido-a-sdk-for-building-autonomous-agent-systems/68418) — Community feedback
- [BB.Reactor](https://hex.pm/packages/bb_reactor) — Workflow orchestration
- [Beyond Behaviour Trees](https://beambots.dev/blog/beyond-behaviour-trees/) — Our rationale for sagas over BTs
