<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal: bb_mcp

**Status:** Draft
**Author:** James Harton
**Created:** 2026-01-11

---

## Summary

`bb_mcp` provides a Model Context Protocol (MCP) server for Beam Bots robots, enabling AI assistants (Claude, GPT, etc.) to control robots through natural language. Robot commands become MCP tools, sensor data and parameters become resources, and the AI can discover and invoke capabilities dynamically.

---

## Motivation

### The AI-Robot Interface Problem

Large language models are increasingly capable of understanding intent and planning actions. But connecting them to physical robots requires custom integration code for each LLM-robot combination. This creates an N×M problem: N robots × M AI systems = many custom integrations.

### What is MCP?

The Model Context Protocol (MCP) is an open standard introduced by Anthropic for connecting AI applications to external systems. Think of it as "USB-C for AI"—a standardised interface that any MCP client can use to access any MCP server.

MCP servers expose three primitives:
- **Tools** — Executable functions the AI can invoke
- **Resources** — Data the AI can read (sensors, state, configuration)
- **Prompts** — Templates that guide AI interactions

### Why MCP for Robotics?

1. **Natural language control** — "Move the arm to pick up the mug" instead of programming waypoints
2. **Standardised interface** — Works with Claude Desktop, Cursor, custom agents, any MCP client
3. **Dynamic discovery** — AI discovers available commands, parameters, and sensors at runtime
4. **No robot code changes** — MCP server wraps existing BB commands and parameters
5. **Bidirectional** — AI can both control and monitor the robot

### Use Cases

**Development assistance:**
- "Show me the current joint positions"
- "Arm the robot and run the home command"
- "What commands are available?"

**Configuration:**
- "Set the maximum velocity to 50%"
- "What parameters can I adjust?"
- "Show me the current safety limits"

**Task execution:**
- "Pick up the red block and place it on the shelf"
- "Calibrate the gripper"
- "Move to the charging station"

**Debugging:**
- "Why did the last command fail?"
- "What's the error state?"
- "Show me the event log for the last 30 seconds"

---

## Design

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    MCP Client                           │
│            (Claude Desktop, Cursor, etc.)               │
└─────────────────────────────────────────────────────────┘
                          │
                    JSON-RPC 2.0
                    (stdio or SSE)
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│                    BB.MCP.Server                        │
│                                                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│  │    Tools    │  │  Resources  │  │   Prompts   │     │
│  │             │  │             │  │             │     │
│  │ - arm       │  │ - joints    │  │ - control   │     │
│  │ - disarm    │  │ - sensors   │  │ - debug     │     │
│  │ - commands  │  │ - state     │  │ - configure │     │
│  │ - get_param │  │ - params    │  │             │     │
│  │ - set_param │  │ - events    │  │             │     │
│  └─────────────┘  └─────────────┘  └─────────────┘     │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│                    BB.Robot                             │
│              (existing Beam Bots robot)                 │
└─────────────────────────────────────────────────────────┘
```

### MCP Server

```elixir
defmodule BB.MCP.Server do
  @moduledoc """
  MCP server exposing a Beam Bots robot to AI assistants.

  ## Usage

  Add to your supervision tree:

      children = [
        {BB.MCP.Server, robot: MyRobot, transport: :stdio}
      ]

  Or start manually:

      {:ok, _pid} = BB.MCP.Server.start_link(
        robot: MyRobot,
        transport: :sse,
        port: 8080
      )

  """

  use GenServer

  defstruct [:robot, :transport, :handler]

  @type transport :: :stdio | {:sse, keyword()}

  def start_link(opts)
  def handle_request(server, request)
end
```

### Tools (Robot Commands & Parameters)

Each robot command becomes an MCP tool, plus parameter management tools:

```elixir
defmodule BB.MCP.Tools do
  @moduledoc """
  Generates MCP tool definitions from robot commands and parameters.
  """

  @doc """
  List all available tools for a robot.
  """
  def list_tools(robot) do
    commands = BB.Robot.commands(robot)

    base_tools() ++ param_tools() ++ Enum.map(commands, &command_to_tool/1)
  end

  defp base_tools do
    [
      %{
        name: "arm",
        description: "Arm the robot, enabling motion commands",
        inputSchema: %{type: "object", properties: %{}}
      },
      %{
        name: "disarm",
        description: "Disarm the robot, stopping all motion",
        inputSchema: %{type: "object", properties: %{}}
      },
      %{
        name: "get_state",
        description: "Get current robot state (armed/disarmed/executing)",
        inputSchema: %{type: "object", properties: %{}}
      }
    ]
  end

  defp param_tools do
    [
      %{
        name: "list_parameters",
        description: "List all available robot parameters with their current values",
        inputSchema: %{type: "object", properties: %{}}
      },
      %{
        name: "get_parameter",
        description: "Get the current value of a specific parameter",
        inputSchema: %{
          type: "object",
          properties: %{
            path: %{
              type: "string",
              description: "Parameter path (e.g., 'velocity_scale' or 'servo.max_torque')"
            }
          },
          required: ["path"]
        }
      },
      %{
        name: "set_parameter",
        description: "Set a parameter value. Use list_parameters to see available parameters and their types.",
        inputSchema: %{
          type: "object",
          properties: %{
            path: %{
              type: "string",
              description: "Parameter path (e.g., 'velocity_scale')"
            },
            value: %{
              description: "New value for the parameter (type depends on parameter)"
            }
          },
          required: ["path", "value"]
        }
      }
    ]
  end

  defp command_to_tool(command) do
    %{
      name: command.name,
      description: command.description || "Execute #{command.name} command",
      inputSchema: build_schema(command.arguments)
    }
  end

  @doc """
  Execute a tool by name.
  """
  def call_tool(robot, "list_parameters", _args) do
    params = BB.Parameter.list(robot)
    {:ok, format_parameters(params)}
  end

  def call_tool(robot, "get_parameter", %{"path" => path}) do
    case BB.Parameter.get(robot, parse_path(path)) do
      {:ok, value} -> {:ok, %{path: path, value: value}}
      {:error, reason} -> {:error, reason}
    end
  end

  def call_tool(robot, "set_parameter", %{"path" => path, "value" => value}) do
    case BB.Parameter.set(robot, parse_path(path), value) do
      :ok -> {:ok, %{path: path, value: value, status: "updated"}}
      {:error, reason} -> {:error, reason}
    end
  end

  def call_tool(robot, "arm", _args) do
    case BB.Robot.Runtime.arm(robot) do
      :ok -> {:ok, %{status: "armed"}}
      {:error, reason} -> {:error, reason}
    end
  end

  # ... other tool implementations
end
```

### Resources (Robot State, Sensors & Parameters)

Robot state, sensors, and parameters become MCP resources:

```elixir
defmodule BB.MCP.Resources do
  @moduledoc """
  Generates MCP resource definitions from robot state.
  """

  def list_resources(robot) do
    [
      %{
        uri: "robot://#{robot}/state",
        name: "Robot State",
        description: "Current robot state machine state",
        mimeType: "application/json"
      },
      %{
        uri: "robot://#{robot}/joints",
        name: "Joint Positions",
        description: "Current position of all joints",
        mimeType: "application/json"
      },
      %{
        uri: "robot://#{robot}/safety",
        name: "Safety State",
        description: "Current safety system state",
        mimeType: "application/json"
      },
      %{
        uri: "robot://#{robot}/parameters",
        name: "Parameters",
        description: "All robot parameters and their current values",
        mimeType: "application/json"
      }
    ] ++ sensor_resources(robot) ++ parameter_resources(robot)
  end

  defp parameter_resources(robot) do
    BB.Parameter.list(robot)
    |> Enum.map(fn {path, _value, meta} ->
      %{
        uri: "robot://#{robot}/parameter/#{Enum.join(path, ".")}",
        name: "Parameter: #{Enum.join(path, ".")}",
        description: meta[:description] || "Robot parameter",
        mimeType: "application/json"
      }
    end)
  end

  def read_resource(robot, uri) do
    case parse_uri(uri) do
      {:state, _} -> read_state(robot)
      {:joints, _} -> read_joints(robot)
      {:safety, _} -> read_safety(robot)
      {:parameters, _} -> read_all_parameters(robot)
      {:parameter, path} -> read_parameter(robot, path)
      {:sensor, name} -> read_sensor(robot, name)
      _ -> {:error, :not_found}
    end
  end

  defp read_all_parameters(robot) do
    params = BB.Parameter.list(robot)
    {:ok, format_parameters(params)}
  end

  defp read_parameter(robot, path) do
    case BB.Parameter.get(robot, path) do
      {:ok, value} -> {:ok, %{path: path, value: value}}
      error -> error
    end
  end
end
```

### Prompts (Interaction Templates)

Prompts guide the AI's interaction with the robot:

```elixir
defmodule BB.MCP.Prompts do
  @moduledoc """
  MCP prompts for robot interaction.
  """

  def list_prompts(_robot) do
    [
      %{
        name: "robot_control",
        description: "Control the robot with natural language commands",
        arguments: [
          %{name: "task", description: "What you want the robot to do", required: true}
        ]
      },
      %{
        name: "robot_status",
        description: "Get a summary of the robot's current state",
        arguments: []
      },
      %{
        name: "robot_configure",
        description: "Adjust robot parameters and configuration",
        arguments: [
          %{name: "goal", description: "What configuration change you want", required: true}
        ]
      },
      %{
        name: "robot_debug",
        description: "Debug robot issues",
        arguments: [
          %{name: "problem", description: "Description of the problem", required: true}
        ]
      }
    ]
  end

  def get_prompt(:robot_configure, %{goal: goal}, robot) do
    params = BB.Parameter.list(robot)
    param_list = format_param_list(params)

    """
    You are configuring a #{robot} robot. Available parameters:

    #{param_list}

    The user wants: #{goal}

    Use list_parameters to see current values, then set_parameter to make changes.
    Validate parameter values are within acceptable ranges before setting.
    Explain what each change will do before making it.
    """
  end

  def get_prompt(:robot_control, %{task: task}, robot) do
    commands = BB.Robot.commands(robot)
    command_list = Enum.map_join(commands, "\n", &"- #{&1.name}: #{&1.description}")

    """
    You are controlling a #{robot} robot. Available commands:

    #{command_list}

    The user wants: #{task}

    First check the robot state, then execute appropriate commands.
    Always check if the robot is armed before sending motion commands.
    """
  end
end
```

### Transport Layer

Support both stdio (local) and SSE (remote) transports:

```elixir
defmodule BB.MCP.Transport.Stdio do
  @moduledoc """
  Stdio transport for local MCP connections.

  Reads JSON-RPC requests from stdin, writes responses to stdout.
  Used by Claude Desktop and similar local clients.
  """

  use GenServer

  def start_link(opts)
  def send_response(transport, response)
end

defmodule BB.MCP.Transport.SSE do
  @moduledoc """
  Server-Sent Events transport for remote MCP connections.

  Runs an HTTP server with SSE endpoint for real-time updates.
  Useful for remote access or web-based clients.
  """

  use Plug.Router

  def start_link(opts)
end
```

### Integration with BB.PubSub

Subscribe to robot events for real-time updates:

```elixir
defmodule BB.MCP.EventStream do
  @moduledoc """
  Streams robot events to MCP clients.
  """

  def subscribe(robot) do
    BB.PubSub.subscribe(robot, [])  # Subscribe to all messages
  end

  def format_event(%BB.Message{} = msg) do
    %{
      type: "robot_event",
      timestamp: msg.timestamp,
      path: msg.path,
      payload: msg.payload
    }
  end
end
```

---

## Package Structure

```
bb_mcp/
├── lib/
│   └── bb/
│       └── mcp/
│           ├── server.ex           # Main MCP server GenServer
│           ├── handler.ex          # JSON-RPC request handler
│           ├── tools.ex            # Tool generation from commands
│           ├── resources.ex        # Resource generation from state
│           ├── prompts.ex          # Prompt templates
│           ├── transport/
│           │   ├── stdio.ex        # Stdio transport
│           │   └── sse.ex          # SSE transport (Plug-based)
│           └── event_stream.ex     # PubSub event streaming
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
    {:jason, "~> 1.4"},           # JSON encoding
    {:plug, "~> 1.14"},           # HTTP for SSE transport
    {:bandit, "~> 1.0"}           # HTTP server for SSE
  ]
end
```

---

## User Experience

### With Claude Desktop

Add to Claude Desktop's `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "my-robot": {
      "command": "mix",
      "args": ["run", "--no-halt", "-e", "BB.MCP.Server.start_link(robot: MyRobot)"],
      "cwd": "/path/to/my_robot_app"
    }
  }
}
```

Then in Claude:
- "What's the robot's current state?"
- "Arm the robot and move to the home position"
- "Show me all available commands"
- "Set the velocity scale to 0.5"
- "What parameters can I adjust?"

### Programmatic Usage

```elixir
# Start MCP server in supervision tree
children = [
  MyRobot,
  {BB.MCP.Server, robot: MyRobot, transport: :stdio}
]

Supervisor.start_link(children, strategy: :one_for_one)
```

### Remote Access via SSE

```elixir
# Start with SSE transport for remote access
{:ok, _} = BB.MCP.Server.start_link(
  robot: MyRobot,
  transport: {:sse, port: 8080}
)
```

---

## Acceptance Criteria

### Must Have

- [ ] `BB.MCP.Server` GenServer handling JSON-RPC 2.0 protocol
- [ ] MCP initialization handshake (protocol version negotiation)
- [ ] Tool discovery via `tools/list`
- [ ] Tool invocation via `tools/call`
- [ ] Base tools: `arm`, `disarm`, `get_state`
- [ ] Parameter tools: `list_parameters`, `get_parameter`, `set_parameter`
- [ ] Dynamic tool generation from robot commands
- [ ] Resource discovery via `resources/list`
- [ ] Resource reading via `resources/read`
- [ ] Resources: state, joints, safety, parameters
- [ ] Stdio transport for local connections
- [ ] Documentation with Claude Desktop setup example
- [ ] Tests for protocol handling and tool execution

### Should Have

- [ ] Prompt discovery via `prompts/list`
- [ ] Prompt templates for control, configure, and debug
- [ ] SSE transport for remote connections
- [ ] PubSub event streaming to clients
- [ ] Sensor resources (dynamic from robot definition)
- [ ] Individual parameter resources
- [ ] Error handling with structured error responses

### Won't Have

- [ ] Authentication/authorisation (use network-level security)
- [ ] Multi-robot support in single server (run multiple servers)
- [ ] WebSocket transport (SSE is sufficient)
- [ ] Custom tool definitions beyond robot commands

---

## Open Questions

1. **Command arguments:** How should complex command arguments (nested structs, units) be represented in MCP tool schemas?

2. **Streaming results:** For long-running commands, should we stream progress updates or just return final result?

3. **Safety considerations:** Should certain commands (force_disarm) or parameters be excluded from MCP access by default?

4. **Resource subscriptions:** MCP supports resource subscriptions. Should we implement real-time joint position and parameter updates?

5. **Multi-robot:** Should one MCP server support multiple robots, or recommend running separate servers?

6. **Parameter validation:** Should we expose parameter metadata (min/max, units, type) so the AI can validate before setting?

---

## Prior Art

### ROS MCP Server

The `ros-mcp-server` project connects LLMs to ROS robots:
- Uses rosbridge for ROS communication
- Exposes topics, services, and actions as tools
- Works with Claude, GPT, and Gemini

**Learnings:** Rosbridge-style abstraction works well. Topic subscriptions as resources are useful.

### Hermes MCP (Elixir)

Cloudwalk's Hermes provides full MCP client/server in Elixir:
- Phoenix integration
- Both client and server functionality
- Production-ready implementation

**Learnings:** Can leverage Hermes for protocol handling, or use as reference implementation.

---

## References

- [Model Context Protocol](https://modelcontextprotocol.io/) — Official MCP documentation
- [MCP Specification](https://spec.modelcontextprotocol.io/) — Protocol specification
- [Hermes MCP](https://github.com/cloudwalk/hermes-mcp) — Elixir MCP implementation
- [ROS MCP Server](https://github.com/robotmcp/ros-mcp-server) — MCP for ROS robots
- [Anthropic MCP Announcement](https://www.anthropic.com/news/model-context-protocol)
