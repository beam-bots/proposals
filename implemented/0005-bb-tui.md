<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal: bb_tui

**Status:** Implemented
**Author:** James Harton
**Created:** 2026-01-11
**Implementation:** [`mcass19/bb_tui`](https://github.com/mcass19/bb_tui) —
community-owned, outside the `beam-bots` org

Implemented independently by [@mcass19](https://github.com/mcass19) and
published to Hex as [`bb_tui`](https://hex.pm/packages/bb_tui) (v0.3.1,
Apache-2.0). It goes well beyond what's specified here. See "As shipped" for
what differs and for what its existence means for the package name.

---

## Summary

`bb_tui` provides a terminal-based dashboard for Beam Bots robots, offering the same functionality as `bb_liveview` but running entirely in the terminal. This enables robot control and monitoring via SSH, on headless systems, or anywhere a web browser isn't available.

---

## Motivation

### Why a Terminal UI?

The `bb_liveview` dashboard is excellent for development and monitoring, but it requires:
- A web browser
- Phoenix application running
- Network access to the web server

Many robotics scenarios don't have these:

1. **SSH access** — Connecting to a robot over SSH, you want to see status and send commands without opening a browser
2. **Headless systems** — Embedded robots (Nerves devices) may not have displays or browsers
3. **Low-bandwidth** — Terminal UIs work over slow connections where web UIs struggle
4. **CI/CD pipelines** — Automated testing and deployment scripts benefit from terminal interfaces
5. **Developer preference** — Some developers prefer terminal tools over web UIs

### Feature Parity with bb_liveview

The terminal dashboard should provide the same core features:
- Safety controls (arm/disarm)
- Joint position display and control
- Event stream monitoring
- Command execution
- Parameter viewing and editing

The main difference is no 3D visualisation (though ASCII-based joint diagrams could be explored).

---

## Design

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Terminal                             │
│              (local or SSH session)                     │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│                  BB.TUI.Application                     │
│                                                         │
│  ┌─────────────────────────────────────────────────┐   │
│  │                   Layout                         │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────────────┐ │   │
│  │  │  Safety  │ │  State   │ │     Joints       │ │   │
│  │  │  Panel   │ │  Panel   │ │     Table        │ │   │
│  │  └──────────┘ └──────────┘ └──────────────────┘ │   │
│  │  ┌──────────────────────┐ ┌──────────────────┐  │   │
│  │  │    Event Stream      │ │   Commands/      │  │   │
│  │  │                      │ │   Parameters     │  │   │
│  │  └──────────────────────┘ └──────────────────┘  │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│                    BB.Robot                             │
│              (existing Beam Bots robot)                 │
└─────────────────────────────────────────────────────────┘
```

### Library Choice: Ratatouille

After evaluating options, Ratatouille is the recommended choice:

| Library | Pros | Cons |
|---------|------|------|
| **Ratatouille** | Mature, Elm Architecture, rich widgets | termbox dependency |
| Owl | Already in deps, good for simple UIs | Less suited for full-screen apps |
| TermUI | Newer, component-based | Less mature |

Ratatouille provides:
- Full-screen terminal applications
- The Elm Architecture (model-update-view)
- Built-in widgets (tables, panels, text)
- Keyboard event handling
- Colour and styling support

### Application Structure

```elixir
defmodule BB.TUI.Application do
  @moduledoc """
  Main TUI application using Ratatouille.

  Implements The Elm Architecture:
  - Model: Current state (robot data, UI state)
  - Update: Handle events (keyboard, robot messages)
  - View: Render UI from model
  """

  @behaviour Ratatouille.App

  alias BB.TUI.{Model, View, Update}

  def start(robot, opts \\ []) do
    Ratatouille.run(__MODULE__, robot: robot, opts: opts)
  end

  @impl true
  def init(%{robot: robot}) do
    # Subscribe to robot PubSub
    BB.PubSub.subscribe(robot, [])

    Model.new(robot)
  end

  @impl true
  def update(model, msg) do
    Update.handle(model, msg)
  end

  @impl true
  def render(model) do
    View.render(model)
  end
end
```

### Model

```elixir
defmodule BB.TUI.Model do
  @moduledoc """
  Application state for the TUI.
  """

  defstruct [
    :robot,
    :safety_state,
    :robot_state,
    :joints,
    :events,
    :parameters,
    :active_panel,
    :command_input,
    :scroll_offset
  ]

  @type panel :: :safety | :joints | :events | :commands | :parameters

  def new(robot) do
    %__MODULE__{
      robot: robot,
      safety_state: BB.Safety.state(robot),
      robot_state: BB.Robot.Runtime.state(robot),
      joints: load_joints(robot),
      events: [],
      parameters: load_parameters(robot),
      active_panel: :safety,
      command_input: "",
      scroll_offset: 0
    }
  end
end
```

### View Components

```elixir
defmodule BB.TUI.View do
  @moduledoc """
  Renders the TUI from the model.
  """

  import Ratatouille.View

  def render(model) do
    view do
      row do
        column size: 4 do
          safety_panel(model)
          state_panel(model)
        end

        column size: 8 do
          joints_panel(model)
        end
      end

      row do
        column size: 6 do
          events_panel(model)
        end

        column size: 6 do
          commands_panel(model)
        end
      end

      status_bar(model)
    end
  end

  defp safety_panel(model) do
    panel title: "Safety", height: :fill do
      label do
        case model.safety_state do
          :armed -> text(color: :green, content: "● ARMED")
          :disarmed -> text(color: :white, content: "○ DISARMED")
          :disarming -> text(color: :yellow, content: "◐ DISARMING...")
          :error -> text(color: :red, content: "✖ ERROR")
        end
      end

      label(content: "")
      label(content: "[a] Arm  [d] Disarm  [f] Force Disarm")
    end
  end

  defp joints_panel(model) do
    panel title: "Joints", height: :fill do
      table do
        table_row do
          table_cell(content: "Joint")
          table_cell(content: "Position")
          table_cell(content: "Min")
          table_cell(content: "Max")
          table_cell(content: "Status")
        end

        for joint <- model.joints do
          table_row do
            table_cell(content: joint.name)
            table_cell(content: format_angle(joint.position))
            table_cell(content: format_angle(joint.min))
            table_cell(content: format_angle(joint.max))
            table_cell(content: status_indicator(joint))
          end
        end
      end
    end
  end

  defp events_panel(model) do
    panel title: "Events (#{length(model.events)})", height: :fill do
      viewport offset_y: model.scroll_offset do
        for event <- Enum.take(model.events, 100) do
          label do
            text(color: :cyan, content: "[#{format_time(event.timestamp)}] ")
            text(content: "#{event.type}: #{inspect(event.payload)}")
          end
        end
      end
    end
  end

  defp commands_panel(model) do
    commands = BB.Robot.commands(model.robot)

    panel title: "Commands", height: :fill do
      for {cmd, idx} <- Enum.with_index(commands) do
        label do
          text(content: "[#{idx + 1}] #{cmd.name}")
          if cmd.description do
            text(color: :white, content: " - #{cmd.description}")
          end
        end
      end
    end
  end

  defp status_bar(model) do
    bar do
      label do
        text(content: "Robot: #{model.robot} | ")
        text(content: "State: #{model.robot_state} | ")
        text(content: "[q] Quit  [Tab] Switch Panel  [?] Help")
      end
    end
  end
end
```

### Update (Event Handling)

```elixir
defmodule BB.TUI.Update do
  @moduledoc """
  Handles events and updates the model.
  """

  alias BB.TUI.Model

  def handle(model, {:event, %{key: key}}) do
    case key do
      :tab -> cycle_panel(model)
      ?q -> {:halt, model}
      ?a -> arm_robot(model)
      ?d -> disarm_robot(model)
      ?f -> force_disarm_robot(model)
      ?j -> {:model, scroll(model, :down)}
      ?k -> {:model, scroll(model, :up)}
      _ -> {:model, model}
    end
  end

  def handle(model, {:bb, _path, %BB.Message{} = msg}) do
    model = add_event(model, msg)
    model = maybe_update_state(model, msg)
    {:model, model}
  end

  def handle(model, _msg) do
    {:model, model}
  end

  defp arm_robot(model) do
    case BB.Robot.Runtime.arm(model.robot) do
      :ok -> {:model, %{model | safety_state: :armed}}
      {:error, _} -> {:model, model}
    end
  end

  defp disarm_robot(model) do
    BB.Safety.disarm(model.robot)
    {:model, %{model | safety_state: :disarming}}
  end

  defp cycle_panel(model) do
    panels = [:safety, :joints, :events, :commands, :parameters]
    current_idx = Enum.find_index(panels, &(&1 == model.active_panel))
    next_idx = rem(current_idx + 1, length(panels))
    {:model, %{model | active_panel: Enum.at(panels, next_idx)}}
  end
end
```

### Mix Task for Quick Launch

```elixir
defmodule Mix.Tasks.Bb.Tui do
  @moduledoc """
  Launch the terminal UI for a robot.

      mix bb.tui MyRobot

  """

  use Mix.Task

  @impl Mix.Task
  def run([robot_module | _]) do
    Mix.Task.run("app.start")

    robot = Module.concat([robot_module])
    BB.TUI.Application.start(robot)
  end
end
```

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `q` | Quit |
| `Tab` | Cycle active panel |
| `a` | Arm robot |
| `d` | Disarm robot |
| `f` | Force disarm (with confirmation) |
| `j`/`↓` | Scroll down |
| `k`/`↑` | Scroll up |
| `1-9` | Execute command by number |
| `Enter` | Confirm action |
| `Esc` | Cancel/back |
| `?` | Show help |

---

## Package Structure

```
bb_tui/
├── lib/
│   └── bb/
│       └── tui/
│           ├── application.ex      # Main Ratatouille app
│           ├── model.ex            # Application state
│           ├── view.ex             # View rendering
│           ├── update.ex           # Event handling
│           ├── components/
│           │   ├── safety.ex       # Safety panel
│           │   ├── joints.ex       # Joints table
│           │   ├── events.ex       # Event stream
│           │   ├── commands.ex     # Command list/execution
│           │   └── parameters.ex   # Parameter editor
│           └── helpers.ex          # Formatting utilities
├── lib/mix/
│   └── tasks/
│       └── bb/
│           └── tui.ex              # mix bb.tui task
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
    {:ratatouille, "~> 0.5"}
  ]
end
```

---

## User Experience

### Starting the TUI

```bash
# Via mix task
mix bb.tui MyRobot

# Or programmatically
iex> BB.TUI.Application.start(MyRobot)
```

### Example Session

```
┌─ Safety ─────────┐┌─ State ──────────┐┌─ Joints ────────────────────────────┐
│ ● ARMED          ││ State: idle      ││ Joint          Pos    Min    Max    │
│                  ││ Executing: -     ││ shoulder_pan   45°   -110°   110°   │
│ [a] Arm          ││                  ││ shoulder_lift  -20°  -100°   100°   │
│ [d] Disarm       ││                  ││ elbow_flex     60°   -97°    97°    │
│ [f] Force Disarm ││                  ││ wrist_flex     0°    -95°    95°    │
└──────────────────┘└──────────────────┘│ wrist_roll     0°    -160°   160°   │
                                        │ gripper        50°    10°    100°   │
                                        └─────────────────────────────────────┘
┌─ Events (23) ─────────────────────────┐┌─ Commands ─────────────────────────┐
│ [12:34:56] StateChanged: idle         ││ [1] home - Move to home position   │
│ [12:34:55] JointPosition: shoulder... ││ [2] demo_circle - Demo circular... │
│ [12:34:54] JointPosition: elbow_fl... ││ [3] pick - Pick object at target   │
│ [12:34:53] SafetyState: armed         ││ [4] place - Place held object      │
│ [12:34:52] CommandStarted: arm        ││                                    │
│                                       ││                                    │
└───────────────────────────────────────┘└────────────────────────────────────┘
Robot: BB.Example.SO101.Robot | State: idle | [q] Quit  [Tab] Switch  [?] Help
```

### Over SSH

```bash
# SSH to robot and launch TUI
ssh robot@192.168.1.100
cd /opt/my_robot
mix bb.tui MyRobot
```

---

## Acceptance Criteria

### Must Have

- [ ] Ratatouille-based full-screen terminal application — built on ExRatatui
- [x] Safety panel with arm/disarm controls
- [x] State display (robot state machine state)
- [x] Joints table with current positions
- [x] Real-time updates via PubSub subscription
- [x] Keyboard navigation between panels
- [x] Basic keyboard shortcuts (quit, arm, disarm)
- [x] `mix bb.tui` task for launching
- [x] Documentation with usage examples
- [x] Tests for model and update logic

### Should Have

- [x] Event stream panel with scrolling
- [x] Command execution (select and run commands)
- [x] Parameter viewing
- [x] Parameter editing (simple types)
- [x] Help overlay (`?` key)
- [x] Colour-coded status indicators
- [x] Joint limit warnings (highlight when near limits)

### Could Have

- [x] ASCII art joint diagram (simple kinematic visualisation) — shipped as real 3D
- [ ] Command history
- [ ] Log file export
- [ ] Configuration file for keybindings
- [ ] Multiple robot support (switch between robots)
- [ ] Mouse support (if terminal supports it)

### Won't Have

- [x] 3D visualisation (terminal limitation) — shipped anyway
- [ ] Complex parameter types (use bb_liveview for those)
- [ ] Video/camera display
- [ ] Touch screen support

---

## As shipped

`mcass19/bb_tui` covers every must-have except the choice of rendering library,
and every should-have. It also shipped one thing this proposal ruled out as
impossible.

### It's built on ExRatatui, not Ratatouille

The whole Design section below assumes Ratatouille and its Elm-architecture
split — `BB.TUI.Application`, `.Model`, `.Update`, `.View`. None of those modules
exist. The implementation uses [ExRatatui](https://github.com/mcass19/ex_ratatui),
a Rust `ratatui` binding by the same author, and organises itself as
`BB.TUI.App`, a `BB.TUI.State` tree (`.Commands`, `.Events`, `.Joints`,
`.Parameters`, `.Power`, `.Safety`, `.Throttle`, `.Ui`, `.Viz`) and a
`BB.TUI.Panels.*` module per panel. Read the module names in this proposal as
illustrative only.

### 3D visualisation happened

"3D visualisation (terminal limitation)" is listed under Won't Have. It shipped:
a Visualization tab renders the live robot from its URDF topology and joint
positions with an orbitable, zoomable camera, using pixel graphics on capable
terminals (Ghostty, WezTerm, Kitty) and falling back to braille over SSH. The
terminal limitation turned out not to be one.

### It went further in several other directions

Not asked for here, but shipped: three transports (local, SSH with isolated
per-operator sessions, and Erlang distribution for attaching a thin renderer to
a TUI running on the robot node); an Igniter installer (`mix igniter.install
bb_tui`) with Nerves and SSH-daemon wiring; a `BB.TUI.Renderer` behaviour letting
consumers render their own PubSub payloads without `bb_tui` depending on their
structs; render coalescing and event-log debouncing so high-rate telemetry can't
stall the UI; and a `usage-rules.md`, per proposal 0007.

### The package name is taken

`bb_tui` on Hex is this package, owned by @mcass19. The `beam-bots` org cannot
publish under that name. This is only a problem if the org ever wants a
first-party terminal dashboard — and given the state of this one, it shouldn't.
The practical consequences are that the ecosystem's terminal UI lives outside
the org under an Apache-2.0 licence and a maintainer the org doesn't control,
and that `bb-sync` will never clone it into the workspace.

---

## Open Questions

1. **Library choice:** Ratatouille vs newer alternatives like TermUI? Ratatouille is more mature but TermUI is actively developed.

2. **Terminal compatibility:** How do we handle terminals that don't support colours or certain characters?

3. **Joint control:** Should we allow direct joint position control via keyboard, or just display?

4. **Screen size:** Minimum terminal size requirements? How to handle resize?

5. **Nerves integration:** Any special considerations for running on Nerves devices?

---

## Prior Art

### htop / btop

System monitoring tools with excellent terminal UIs:
- Real-time updates
- Keyboard navigation
- Colour-coded status

**Learnings:** Keep the UI responsive, show the most important info prominently.

### k9s (Kubernetes TUI)

Terminal UI for Kubernetes:
- Complex hierarchical data
- Command execution
- Vim-style navigation

**Learnings:** Vim keybindings are familiar to many developers.

### lazygit

Terminal UI for git:
- Panel-based layout
- Keyboard-driven
- Context-sensitive actions

**Learnings:** Good panel layouts make complex tools usable.

---

## References

- [Ratatouille](https://github.com/ndreynolds/ratatouille) — Declarative terminal UI for Elixir
- [The Elm Architecture](https://guide.elm-lang.org/architecture/) — Model-Update-View pattern
- [bb_liveview](https://github.com/beam-bots/bb_liveview) — Web-based dashboard (feature reference)
- [TermUI](https://elixirforum.com/t/termui-a-direct-mode-terminal-user-interface-framework-with-components/73464) — Alternative terminal UI framework
