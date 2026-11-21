<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal 0008: bb_mavlink

## Summary

A MAVLINK bridge package for Beam Bots that enables communication with autopilot systems (ArduPilot, PX4, etc.) for parameter exchange and sensor data ingestion. Provides an Elixir implementation of the MAVLINK v2 protocol with integration into BB's message system.

## Motivation

Many robotics projects incorporate flight controllers or autopilot hardware that use MAVLINK as their communication protocol. These systems provide:

- **Sensor fusion**: Autopilots combine IMU, GPS, barometer, and magnetometer data into filtered attitude and position estimates
- **Parameter management**: Hundreds of tunable parameters for control loops, sensor calibration, and system configuration
- **Hardware abstraction**: Standard interface to diverse sensor hardware
- **Proven reliability**: Battle-tested in thousands of deployed systems

### Use Cases

1. **Mobile robots with flight controller IMU** - Use a Pixhawk/ArduPilot as a sensor hub, reading fused attitude and position data
2. **Hybrid ground/air vehicles** - Coordinate between BB-controlled manipulators and autopilot-controlled locomotion
3. **Sensor integration** - Access GPS, compass, barometer, and rangefinder data through a single interface
4. **Remote parameter tuning** - Adjust autopilot PID gains and configuration from BB applications
5. **Multi-vehicle systems** - Coordinate multiple MAVLINK-enabled vehicles from a central BB application

### Why Not Just Use Serial/I2C Directly?

MAVLINK provides:
- Reliable parameter read/write with acknowledgment and retry
- Efficient binary encoding optimised for bandwidth-constrained links
- Standard message definitions across autopilot vendors
- Built-in system/component addressing for multi-device networks
- Heartbeat-based connection monitoring

## Design

### Architecture Overview

```
                                    ┌─────────────────────────────────┐
                                    │      BB Application             │
                                    └─────────────────────────────────┘
                                                   │
                    ┌──────────────────────────────┼──────────────────────────────┐
                    │                              │                              │
                    ▼                              ▼                              ▼
         ┌─────────────────────┐       ┌─────────────────────┐       ┌─────────────────────┐
         │ BB.Mavlink.Parameter│       │ BB.Mavlink.Telemetry│       │ BB.Mavlink.Command  │
         │     (read/write)    │       │   (sensor streams)  │       │   (vehicle cmds)    │
         └─────────────────────┘       └─────────────────────┘       └─────────────────────┘
                    │                              │                              │
                    └──────────────────────────────┼──────────────────────────────┘
                                                   │
                                                   ▼
                                    ┌─────────────────────────────────┐
                                    │      BB.Mavlink.Connection      │
                                    │   (framing, routing, heartbeat) │
                                    └─────────────────────────────────┘
                                                   │
                              ┌────────────────────┼────────────────────┐
                              ▼                    ▼                    ▼
                       ┌───────────┐        ┌───────────┐        ┌───────────┐
                       │   Serial  │        │    UDP    │        │    TCP    │
                       └───────────┘        └───────────┘        └───────────┘
```

### Core Components

#### 1. Connection Management

```elixir
defmodule BB.Mavlink.Connection do
  use GenServer

  @type transport :: {:serial, path :: String.t(), opts :: keyword()}
                   | {:udp, port :: pos_integer(), opts :: keyword()}
                   | {:tcp, host :: String.t(), port :: pos_integer(), opts :: keyword()}

  @type t :: %__MODULE__{
    transport: transport(),
    system_id: 1..255,
    component_id: 1..255,
    target_system: 1..255,
    target_component: 1..255,
    sequence: 0..255,
    subscribers: %{message_id :: pos_integer() => [pid()]},
    last_heartbeat: DateTime.t() | nil,
    connected: boolean()
  }

  @doc "Start a MAVLINK connection"
  def start_link(transport, opts \\ [])

  @doc "Subscribe to specific message types"
  def subscribe(conn, message_ids)

  @doc "Unsubscribe from message types"
  def unsubscribe(conn, message_ids)

  @doc "Send a raw MAVLINK message"
  def send_message(conn, message)

  @doc "Check if connection is alive (recent heartbeat)"
  def connected?(conn)

  @doc "Get connection statistics"
  def stats(conn)
end
```

The connection GenServer handles:
- Transport abstraction (serial, UDP, TCP)
- Message framing and CRC validation
- Sequence number management
- Heartbeat transmission (1Hz)
- Connection state monitoring
- Message routing to subscribers

#### 2. Message Codec

```elixir
defmodule BB.Mavlink.Codec do
  @moduledoc """
  MAVLINK v2 message encoding and decoding.

  Message modules are generated at compile time from XML dialect files.
  By default, common.xml is included. Additional dialects can be added
  via configuration:

      config :bb_mavlink, :dialects, [
        "priv/dialects/common.xml",
        "priv/dialects/ardupilotmega.xml"
      ]
  """

  @type message :: %{
    __struct__: module(),
    # message-specific fields
  }

  @doc "Encode a message struct to binary"
  def encode(message, sequence, system_id, component_id)

  @doc "Decode binary to message struct"
  def decode(binary)

  @doc "Calculate CRC for message"
  def crc(payload, message_id)
end
```

Message structs are generated at compile time from MAVLINK XML dialect files. The generator parses the XML and creates a module per message with appropriate struct fields, encode/decode functions, and CRC extra byte:

```elixir
defmodule BB.Mavlink.Message.Heartbeat do
  @message_id 0
  @crc_extra 50

  defstruct [
    :type,           # MAV_TYPE enum
    :autopilot,      # MAV_AUTOPILOT enum
    :base_mode,      # MAV_MODE_FLAG bitmask
    :custom_mode,    # autopilot-specific
    :system_status,  # MAV_STATE enum
    :mavlink_version
  ]
end

defmodule BB.Mavlink.Message.Attitude do
  @message_id 30
  @crc_extra 39

  defstruct [
    :time_boot_ms,
    :roll,          # radians
    :pitch,         # radians
    :yaw,           # radians
    :rollspeed,     # rad/s
    :pitchspeed,    # rad/s
    :yawspeed       # rad/s
  ]
end
```

#### 3. Parameter Protocol

```elixir
defmodule BB.Mavlink.Parameter do
  @moduledoc """
  MAVLINK parameter protocol implementation.

  Provides reliable read/write of autopilot parameters with
  acknowledgment and retry logic.
  """

  @type param_value :: float() | integer()
  @type param_type :: :float | :int8 | :uint8 | :int16 | :uint16 | :int32 | :uint32

  @type param :: %{
    id: String.t(),
    value: param_value(),
    type: param_type(),
    index: non_neg_integer()
  }

  @type error ::
    :timeout |
    :connection_lost |
    {:invalid_param, String.t()} |
    {:type_mismatch, expected :: param_type(), got :: param_type()}

  @doc "Read all parameters from the autopilot"
  @spec read_all(Connection.t(), keyword()) :: {:ok, [param()]} | {:error, error()}
  def read_all(conn, opts \\ [])

  @doc "Read a single parameter by name"
  @spec read(Connection.t(), String.t(), keyword()) :: {:ok, param()} | {:error, error()}
  def read(conn, param_id, opts \\ [])

  @doc "Read a single parameter by index"
  @spec read_by_index(Connection.t(), non_neg_integer(), keyword()) :: {:ok, param()} | {:error, error()}
  def read_by_index(conn, index, opts \\ [])

  @doc "Write a parameter value"
  @spec write(Connection.t(), String.t(), param_value(), keyword()) :: {:ok, param()} | {:error, error()}
  def write(conn, param_id, value, opts \\ [])

  @doc "Write multiple parameters atomically (best effort)"
  @spec write_many(Connection.t(), [{String.t(), param_value()}], keyword()) ::
    {:ok, [param()]} | {:error, error(), succeeded :: [param()]}
  def write_many(conn, params, opts \\ [])
end
```

Options for parameter operations:
- `:timeout` - Operation timeout in milliseconds (default: 5000)
- `:retries` - Number of retry attempts (default: 3)
- `:retry_delay` - Delay between retries in milliseconds (default: 100)

#### 4. Telemetry Streaming

```elixir
defmodule BB.Mavlink.Telemetry do
  @moduledoc """
  Subscribe to autopilot telemetry streams and convert to BB messages.
  """

  @type stream :: :attitude | :position | :imu | :gps | :battery | :system_status

  @doc "Start streaming telemetry, publishing to BB message bus"
  def start_stream(conn, stream, opts \\ [])

  @doc "Stop a telemetry stream"
  def stop_stream(conn, stream)

  @doc "Request specific message rate from autopilot"
  def set_message_rate(conn, message_id, rate_hz)
end
```

Telemetry is published as BB messages:

```elixir
defmodule BB.Message.Mavlink.Attitude do
  @moduledoc "Attitude estimate from MAVLINK autopilot"

  defstruct [
    :timestamp,
    :roll,        # BB.Math.Angle in radians
    :pitch,       # BB.Math.Angle in radians
    :yaw,         # BB.Math.Angle in radians
    :roll_rate,   # rad/s
    :pitch_rate,  # rad/s
    :yaw_rate,    # rad/s
    :source       # {:mavlink, system_id, component_id}
  ]
end

defmodule BB.Message.Mavlink.GlobalPosition do
  @moduledoc "Global position estimate from MAVLINK autopilot"

  defstruct [
    :timestamp,
    :latitude,       # degrees
    :longitude,      # degrees
    :altitude_msl,   # metres above mean sea level
    :altitude_rel,   # metres above ground
    :velocity,       # BB.Math.Vec3 in m/s (NED frame)
    :heading,        # BB.Math.Angle in radians
    :source
  ]
end

defmodule BB.Message.Mavlink.RawImu do
  @moduledoc "Raw IMU data from MAVLINK autopilot"

  defstruct [
    :timestamp,
    :accelerometer,  # BB.Math.Vec3 in m/s^2
    :gyroscope,      # BB.Math.Vec3 in rad/s
    :magnetometer,   # BB.Math.Vec3 in gauss
    :temperature,    # degrees C (if available)
    :source
  ]
end

defmodule BB.Message.Mavlink.Battery do
  @moduledoc "Battery status from MAVLINK autopilot"

  defstruct [
    :timestamp,
    :voltage,           # volts
    :current,           # amps (negative = discharging)
    :remaining_percent, # 0-100
    :remaining_mah,     # milliamp-hours remaining
    :source
  ]
end
```

#### 5. Request/Response Patterns

For operations requiring acknowledgment:

```elixir
defmodule BB.Mavlink.Request do
  @moduledoc """
  Handles request/response patterns with timeout and retry.
  """

  @doc "Send request and wait for matching response"
  def request(conn, request_message, response_matcher, opts \\ [])

  @doc "Send request with automatic retry on timeout"
  def request_with_retry(conn, request_message, response_matcher, opts \\ [])
end
```

#### 6. Vehicle Commands

```elixir
defmodule BB.Mavlink.Command do
  @moduledoc """
  Send commands to MAVLINK vehicles.

  Uses MAV_CMD long command protocol with acknowledgment.
  """

  @type command_result ::
    :accepted |
    :temporarily_rejected |
    :denied |
    :unsupported |
    :failed |
    :in_progress |
    :cancelled

  @doc "Arm/disarm the vehicle"
  def arm(conn, arm? \\ true, opts \\ [])

  @doc "Set flight mode"
  def set_mode(conn, mode, opts \\ [])

  @doc "Request specific data stream"
  def request_data_stream(conn, stream_id, rate_hz, opts \\ [])

  @doc "Send arbitrary MAV_CMD"
  def send_command(conn, command, params, opts \\ [])
end
```

#### 7. DSL Integration

For declaring MAVLINK connections in BB robot definitions:

```elixir
defmodule MyRobot do
  use BB.Robot

  mavlink :autopilot do
    transport {:serial, "/dev/ttyACM0", baud: 115200}
    system_id 1
    component_id 1

    # Subscribe to telemetry streams
    stream :attitude, rate: 50
    stream :position, rate: 10
    stream :battery, rate: 1
  end

  # Use autopilot IMU for robot state estimation
  sensor :imu, source: {:mavlink, :autopilot, :attitude}
end
```

#### 8. Dialect Code Generation

```elixir
defmodule BB.Mavlink.Dialect.Generator do
  @moduledoc """
  Compile-time code generator for MAVLINK dialects.

  Parses MAVLINK XML dialect files and generates:
  - Message struct modules with encode/decode
  - Enum modules with value mappings
  - CRC extra bytes for message validation

  Used internally during compilation. Not called at runtime.
  """

  @doc "Parse dialect XML and return AST for message modules"
  def generate(xml_path)

  @doc "Generate enum module from dialect enums"
  def generate_enums(xml_path)
end
```

The generator is invoked at compile time via a `use` macro:

```elixir
defmodule BB.Mavlink.Messages do
  use BB.Mavlink.Dialect,
    dialects: [
      "priv/dialects/common.xml",
      "priv/dialects/ardupilotmega.xml"
    ]

  # Generates BB.Mavlink.Message.Heartbeat, BB.Mavlink.Message.Attitude, etc.
end
```

### Error Types

```elixir
defmodule BB.Error.Mavlink do
  defmodule ConnectionLost do
    use BB.Error, class: :communication, fields: [:last_heartbeat, :transport]
  end

  defmodule Timeout do
    use BB.Error, class: :communication, fields: [:operation, :timeout_ms]
  end

  defmodule InvalidMessage do
    use BB.Error, class: :communication, fields: [:reason, :raw_bytes]
  end

  defmodule ParameterNotFound do
    use BB.Error, class: :configuration, fields: [:param_id]
  end

  defmodule CommandRejected do
    use BB.Error, class: :command, fields: [:command, :result, :progress]
  end
end
```

### Transport Implementations

#### Serial

```elixir
defmodule BB.Mavlink.Transport.Serial do
  @moduledoc """
  Serial port transport for MAVLINK.

  Uses Circuits.UART for cross-platform serial communication.
  """

  @default_opts [
    baud: 115_200,
    data_bits: 8,
    stop_bits: 1,
    parity: :none,
    flow_control: :none
  ]
end
```

#### UDP

```elixir
defmodule BB.Mavlink.Transport.UDP do
  @moduledoc """
  UDP transport for MAVLINK.

  Supports both listening (for connections from autopilot) and
  connecting (to known autopilot address).
  """

  @default_opts [
    mode: :listen,  # :listen or :connect
    bind_port: 14550,
    target_host: nil,
    target_port: 14550
  ]
end
```

### Example Usage

```elixir
# Connect to autopilot over serial
{:ok, conn} = BB.Mavlink.Connection.start_link(
  {:serial, "/dev/ttyACM0", baud: 115200},
  system_id: 255,
  component_id: 190
)

# Wait for connection
:ok = BB.Mavlink.Connection.await_connected(conn, timeout: 5000)

# Read all parameters
{:ok, params} = BB.Mavlink.Parameter.read_all(conn)
IO.puts("Loaded #{length(params)} parameters")

# Read specific parameter
{:ok, %{value: value}} = BB.Mavlink.Parameter.read(conn, "ARMING_CHECK")
IO.puts("ARMING_CHECK = #{value}")

# Write parameter
{:ok, _} = BB.Mavlink.Parameter.write(conn, "ARMING_CHECK", 0)

# Subscribe to attitude updates
BB.Mavlink.Telemetry.start_stream(conn, :attitude, rate: 50)

# Attitude messages now published to BB message bus
BB.PubSub.subscribe(BB.Message.Mavlink.Attitude)

receive do
  %BB.Message.Mavlink.Attitude{roll: roll, pitch: pitch, yaw: yaw} ->
    IO.puts("Attitude: roll=#{roll}, pitch=#{pitch}, yaw=#{yaw}")
end
```

### Dependencies

- `bb` - Core Beam Bots framework
- `circuits_uart` - Serial port communication
- `saxy` - XML parsing for dialect generation (compile-time only)

### Prerequisites

- **BB.Process multi-instance support**: For multi-vehicle scenarios, BB core needs to support running multiple instances of a robot module simultaneously. This is out of scope for bb_mavlink but required for the "one robot module = one MAVLINK system" architecture to scale to swarms.

## Acceptance Criteria

### Phase 1: Core Protocol

- [ ] MAVLINK v2 packet framing and CRC validation
- [ ] Message encoding/decoding for common.xml dialect
- [ ] Serial transport with Circuits.UART
- [ ] Connection GenServer with heartbeat handling
- [ ] Message subscription/routing
- [ ] Connection state monitoring (connected/disconnected)
- [ ] Basic test coverage with mock transport

### Phase 2: Parameter Protocol

- [ ] `read_all/2` - Read all parameters with progress tracking
- [ ] `read/3` - Read single parameter by name
- [ ] `read_by_index/3` - Read single parameter by index
- [ ] `write/4` - Write parameter with acknowledgment
- [ ] Timeout and retry handling
- [ ] Parameter caching option

### Phase 3: Telemetry

- [ ] Attitude stream (`ATTITUDE` message)
- [ ] Position stream (`GLOBAL_POSITION_INT` message)
- [ ] Raw IMU stream (`RAW_IMU`, `SCALED_IMU` messages)
- [ ] Battery stream (`BATTERY_STATUS` message)
- [ ] System status stream (`SYS_STATUS` message)
- [ ] Message rate configuration (`REQUEST_DATA_STREAM`)
- [ ] BB message integration

### Phase 4: Additional Transports

- [ ] UDP transport (listen and connect modes)
- [ ] TCP transport

### Phase 5: DSL & Commands

- [ ] DSL integration for robot definitions
- [ ] Vehicle commands (arm, mode, etc.)
- [ ] Additional dialect support (ardupilotmega.xml)
- [ ] Message signing (MAVLINK v2 security)

## Design Decisions

### Compile-Time Dialect Generation

Message structs are generated at compile time from MAVLINK XML dialect files rather than shipping pre-generated modules. This:
- Allows users to include only the dialects they need
- Supports custom/vendor-specific dialects without changes to bb_mavlink
- Keeps runtime footprint minimal on constrained devices
- Trades slower compilation for faster execution

### Direct BB.PubSub Integration

Telemetry messages are published directly to `BB.PubSub`. High-rate streams (50Hz+ attitude data) will generate significant message volume, but this is consistent with how other BB sensors work and allows standard subscription patterns. Consumers who don't need the data simply don't subscribe.

### One Connection Per System

Each `BB.Mavlink.Connection` represents a single MAVLINK system (autopilot). Multi-vehicle scenarios are handled by instantiating multiple robot modules, each with its own MAVLINK connection. This:
- Aligns with BB's "one robot module = one robot" model
- Keeps the MAVLINK bridge simple (no system ID routing)
- Pushes multi-vehicle coordination to the orchestration layer where it belongs

**Prerequisite**: BB.Process will need modification to support multiple instances of a robot module running simultaneously. This is a BB core change that benefits other multi-robot use cases beyond MAVLINK.

### Raw Passthrough (No Fusion)

bb_mavlink passes through sensor readings and position estimates exactly as received from the autopilot. No timestamp conversion, no coordinate transformation, no sensor fusion. The autopilot's internal EKF has already fused its sensors; we simply expose that data to BB.

If a system needs to combine MAVLINK data with other sensors (e.g. local IMU on a manipulator), that's the responsibility of a dedicated state estimation package (see: future `bb_ekf`).

## Open Questions

1. **Coordinate frame handling**: MAVLINK uses NED (North-East-Down) coordinates while BB uses a configurable frame. Should coordinate conversion happen in the bridge or be left to consumers?

## References

- [MAVLINK Protocol Overview](https://mavlink.io/en/about/overview.html)
- [MAVLINK Parameter Protocol](https://mavlink.io/en/services/parameter.html)
- [MAVLINK Common Message Set](https://mavlink.io/en/messages/common.html)
- [ArduPilot MAVLINK Interface](https://ardupilot.org/dev/docs/mavlink-commands.html)
- [PX4 MAVLINK Interface](https://docs.px4.io/main/en/middleware/mavlink.html)
- [pymavlink (Python reference implementation)](https://github.com/ArduPilot/pymavlink)
- [MAVSDK](https://mavsdk.mavlink.io/main/en/)
