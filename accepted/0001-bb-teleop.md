<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal: bb_teleop

**Status:** Draft
**Author:** James Harton
**Created:** 2026-01-11

---

## Summary

`bb_teleop` provides manual control of Beam Bots robots through various input devices. It enables operators to directly command robot motion for tasks like calibration verification, debugging, demonstration recording, and manual intervention.

---

## Motivation

### Why Teleoperation?

Manual robot control is fundamental to robotics development and operation:

1. **Calibration verification** — After calibrating a robot, operators need to verify joints move correctly by manually commanding motion.

2. **Debugging** — When a command fails or behaves unexpectedly, manual control helps isolate whether the issue is in the command logic, kinematics, or hardware.

3. **Demonstration recording** — For imitation learning (future `bb_policy` integration), human demonstrations are collected via teleoperation.

4. **Manual intervention** — Production robots sometimes need human takeover when autonomous behaviour fails or encounters unexpected situations.

5. **Teaching** — Showing a robot a path or pose by manually guiding it, rather than programming coordinates.

### Why a Separate Package?

Teleoperation is valuable independently:

- Not all robots need teleoperation (fixed automation)
- Input device dependencies vary by platform
- Can evolve independently of core framework
- Users can choose which input drivers to include

Existing alternatives (`BB.Command`, `bb_reactor`) can orchestrate scripted motion but don't provide real-time human-in-the-loop control.

---

## Design

### Core Abstraction

```elixir
defmodule BB.Teleop.Input do
  @moduledoc """
  Behaviour for input devices that produce teleop commands.
  """

  @type command :: %{
    optional(:joint_positions) => %{atom() => float()},
    optional(:joint_velocities) => %{atom() => float()},
    optional(:cartesian_position) => BB.Math.Transform.t(),
    optional(:cartesian_velocity) => BB.Math.Vec3.t(),
    optional(:gripper) => float()
  }

  @type state :: term()

  @doc "Initialise the input device."
  @callback init(opts :: keyword()) :: {:ok, state()} | {:error, term()}

  @doc "Read current input state and produce command."
  @callback read(state()) :: {:ok, command(), state()} | {:error, term(), state()}

  @doc "Check if deadman/enable switch is active."
  @callback enabled?(state()) :: boolean()

  @doc "Clean up resources."
  @callback terminate(state()) :: :ok
end
```

### Teleoperator Server

```elixir
defmodule BB.Teleop.Server do
  @moduledoc """
  GenServer that bridges input devices to robot control.

  Runs a control loop that:
  1. Reads input device state
  2. Applies safety limits (speed, workspace bounds)
  3. Sends commands to robot actuators
  4. Publishes telemetry for monitoring
  """

  use GenServer

  defstruct [
    :robot,
    :input,
    :input_state,
    :mode,
    :safety_config,
    :rate_hz
  ]

  # Control modes
  @type mode :: :joint_position | :joint_velocity | :cartesian_position | :cartesian_velocity
end
```

### Safety Configuration

```elixir
defmodule BB.Teleop.Safety do
  @moduledoc """
  Safety settings for teleoperation.
  """

  defstruct [
    # Deadman switch required to move
    deadman_required: true,

    # Maximum velocity scaling (0.0 - 1.0)
    max_velocity_scale: 0.5,

    # Workspace limits (optional bounding box)
    workspace_bounds: nil,

    # Ramp time for velocity changes (smooth acceleration)
    velocity_ramp_ms: 100,

    # Timeout: stop if no input for this long
    input_timeout_ms: 500
  ]
end
```

### Control Modes

| Mode | Input | Output | Use Case |
|------|-------|--------|----------|
| `joint_position` | Joint angles | `BB.Actuator.set_position` | Direct joint control |
| `joint_velocity` | Joint velocities | `BB.Actuator.set_velocity` | Continuous motion |
| `cartesian_position` | End-effector pose | IK → joint positions | Task-space control |
| `cartesian_velocity` | End-effector twist | IK → joint velocities | Smooth task-space motion |

### Input Device Implementations

**Phase 1: Included in `bb_teleop`**

```elixir
defmodule BB.Teleop.Input.Keyboard do
  @moduledoc """
  Keyboard input for basic teleoperation.

  Uses terminal raw mode to capture keypresses.
  Arrow keys, WASD, or configurable bindings.
  """
  @behaviour BB.Teleop.Input
end
```

**Phase 2: Separate packages**

- `bb_teleop_gamepad` — SDL2 or evdev-based gamepad input
- `bb_teleop_leader` — Read positions from a leader arm (same hardware as follower)
- `bb_teleop_spacemouse` — 3Dconnexion SpaceMouse support

### Integration Points

**With BB.PubSub:**
```elixir
# Teleop server publishes its state
BB.PubSub.broadcast(robot, [:teleop], %BB.Teleop.State{
  mode: :cartesian_velocity,
  enabled: true,
  input_device: :gamepad,
  velocity_scale: 0.5
})

# UI can subscribe to show teleop status
BB.PubSub.subscribe(robot, [:teleop])
```

**With BB.Safety:**
```elixir
# Teleop registers with safety system
BB.Safety.register_controller(robot, teleop_pid, :teleop)

# Safety system can disable teleop
BB.Safety.disable_controller(robot, :teleop)
```

**With BB.Command:**
```elixir
# Teleop can be wrapped as a command for integration with reactor
defmodule BB.Teleop.Command do
  use BB.Command

  @impl BB.Command
  def handle_command(%{input: input_mod, opts: opts}, context, state) do
    # Start teleop session, run until cancelled or timeout
  end
end
```

---

## Package Structure

```
bb_teleop/
├── lib/
│   └── bb/
│       └── teleop/
│           ├── input.ex          # Input behaviour
│           ├── server.ex         # Main GenServer
│           ├── safety.ex         # Safety configuration
│           ├── state.ex          # Teleop state struct
│           ├── command.ex        # BB.Command wrapper
│           └── input/
│               └── keyboard.ex   # Keyboard implementation
├── test/
│   └── bb/
│       └── teleop/
│           ├── server_test.exs
│           └── input/
│               └── keyboard_test.exs
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
    # No additional dependencies for core package
  ]
end
```

---

## User Experience

### Basic Usage

```elixir
# Start teleop with keyboard
{:ok, teleop} = BB.Teleop.Server.start_link(
  robot: MyRobot,
  input: BB.Teleop.Input.Keyboard,
  mode: :joint_velocity,
  rate_hz: 50
)

# With custom safety settings
{:ok, teleop} = BB.Teleop.Server.start_link(
  robot: MyRobot,
  input: BB.Teleop.Input.Keyboard,
  mode: :cartesian_velocity,
  safety: %BB.Teleop.Safety{
    max_velocity_scale: 0.3,
    deadman_required: true
  }
)

# Stop teleop
BB.Teleop.Server.stop(teleop)
```

### With External Input Package

```elixir
# Using gamepad from bb_teleop_gamepad
{:ok, teleop} = BB.Teleop.Server.start_link(
  robot: MyRobot,
  input: BB.Teleop.Input.Gamepad,
  input_opts: [device: "/dev/input/js0"],
  mode: :cartesian_velocity
)
```

### In Livebook (via bb_kino)

```elixir
# Visual teleop widget with sliders and buttons
Kino.BB.Teleop.new(robot, mode: :joint_position)
```

---

## Acceptance Criteria

### Must Have

- [ ] `BB.Teleop.Input` behaviour defined with `init/1`, `read/1`, `enabled?/1`, `terminate/1`
- [ ] `BB.Teleop.Server` GenServer that runs control loop at configurable rate
- [ ] `BB.Teleop.Safety` struct with deadman, velocity scaling, timeout settings
- [ ] `BB.Teleop.Input.Keyboard` implementation for basic testing
- [ ] Control modes: `joint_position` and `joint_velocity`
- [ ] Deadman switch support (no motion without enable signal)
- [ ] Velocity scaling (limit maximum speed)
- [ ] Input timeout (stop robot if input device disconnects)
- [ ] PubSub integration (publish teleop state)
- [ ] Safety system integration (register with BB.Safety)
- [ ] Documentation with usage examples
- [ ] Tests for server lifecycle and safety behaviour

### Should Have

- [ ] `cartesian_position` mode with IK integration
- [ ] `cartesian_velocity` mode with IK integration
- [ ] Velocity ramping (smooth acceleration/deceleration)
- [ ] `BB.Teleop.Command` wrapper for reactor integration
- [ ] Livebook example demonstrating teleop

### Won't Have

- [ ] Specific gamepad implementations (separate `bb_teleop_gamepad`)
- [ ] Leader-follower control (separate `bb_teleop_leader`)
- [ ] Phone/mobile input (separate `bb_teleop_phone`)
- [ ] Demonstration recording (handled by `bb_dataset`)
- [ ] Autonomous behaviour (handled by `bb_policy`)

---

## Open Questions

1. **Keyboard implementation:** Use Erlang's `:io` module in raw mode, or depend on a terminal library like `ex_termbox`?

2. **Rate limiting:** Should the control loop rate be enforced strictly (drop inputs if behind), or should it process all inputs (variable rate)?

3. **Cartesian IK:** For Cartesian modes, should teleop use the robot's configured IK solver, or accept one as a parameter?

4. **Gripper control:** How should gripper open/close be handled? Separate axis? Binary toggle?

5. **Multi-arm robots:** For bimanual robots, should one teleop server control both arms, or should there be separate instances?

---

## References

- [ROS teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) — Simple keyboard teleop
- [ROS joy](http://wiki.ros.org/joy) — Joystick input
- [LeRobot teleoperation](https://github.com/huggingface/lerobot) — Demonstration collection
- [ALOHA](https://tonyzhaozh.github.io/aloha/) — Leader-follower teleop for bimanual manipulation
