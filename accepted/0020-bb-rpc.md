<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal: bb_rpc

**Status:** Draft
**Author:** James Harton
**Created:** 2026-06-01

---

## Summary

`bb_rpc` is the Beam Bots RPC protocol: a small, extensible, CBOR-based protocol
that carries request/response calls, server-streaming responses, and
fire-and-forget notifications over any reliable-or-lossy byte stream (UART, TCP,
TLS, and future datagram transports). It ships as a normative wire specification
plus reference implementations in Elixir, C/C++, and Rust, sharing one set of
conformance vectors. The protocol core carries no dependency on BB types — the
wire format is reusable on its own — but it is built for and homed in Beam Bots,
so the Elixir package also ships the `BB.Bridge` and `BB.PubSub` integration that
motivates it. (It is named `bb_rpc` rather than a generic `cbor-rpc`, which
already denotes several unrelated projects.)

The envelope is six single-byte opcodes carried as CBOR arrays. Streaming is
modelled as a response that "keeps the call id open": one call can produce many
partial responses followed by a terminal `ok`/`error`. This is the mechanism
that lets a single `pubsub.subscribe` call become a live stream of messages,
correlated by the original call id, with a per-stream sequence number so a peer
can detect drops on a lossy link.

---

## Motivation

### The problem

Beam Bots already defines the *shape* of remote integration but has no wire
protocol underneath it:

- `BB.Bridge` (in `bb`) is a behaviour with inbound callbacks (`list_remote/1`,
  `get_remote/2`, `set_remote/3`, `subscribe_remote/2`) and an outbound
  `handle_change/3`. Today every bridge author has to invent their own transport
  and serialisation.
- `BB.Parameter.{list,get,set,subscribe}_remote` expose that behaviour as a
  clean Elixir API, but the bytes on the wire are undefined.
- `BB.PubSub` routes `BB.Message` structs by hierarchical path. Forwarding those
  to a remote GCS, microcontroller, or web client needs a streaming transport
  that none of the existing pieces provide.

Separately, the ecosystem keeps hitting the same lower-level need: a framed,
typed, bidirectional protocol that works equally well over a 57600-baud UART to
a microcontroller, a TCP socket to a desktop GCS, and a TLS socket across a
network. `feetech` and `robotis` each hand-roll a framed binary protocol;
`bb_mavlink` (proposal 0008) needs one; a future microcontroller co-processor
link needs one. They should share a protocol rather than each reinventing
framing, correlation, and error handling.

### Why a new protocol rather than an existing one

| Option | Why not |
|---|---|
| **gRPC** | HTTP/2 + protobuf codegen is far too heavy for a UART link to an AVR, and adds a cross-compile/codegen burden on Nerves. |
| **JSON-RPC 2.0** | Right semantics, but text encoding and repeated string keys are wasteful on constrained links, and it has no native streaming. |
| **MessagePack-RPC** | The right *semantics* (request/response/notification, id correlation) in a compact positional array — but MessagePack's `ext` extensibility is a single uncoordinated 8-bit code, and the project is largely dormant. |
| **CoAP (RFC 7252) + Observe** | The only standard with a real subscribe/notify model, and CBOR-friendly — but UDP-centric with heavy option encoding; more than we want for a stream transport. |

The design borrows MessagePack-RPC's positional-array **semantics**, encodes
them in **CBOR** (RFC 8949 — an IETF Internet Standard with a proper IANA tag
registry for extensibility and graceful pass-through of unknown tags), and adds
CoAP-Observe-style **streaming correlated to the call id**. We get a proven RPC
model, first-class extensibility for third-party message types, and server-push
streaming, without inheriting a heavyweight stack.

---

## Design

### Layering

```
┌──────────────────────────────────────────────────────────┐
│ Application methods   param.get · pubsub.subscribe · …     │  ← BB & users define these
├──────────────────────────────────────────────────────────┤
│ Typed payloads        CBOR tag → named type + value        │  ← extensible type registry
├──────────────────────────────────────────────────────────┤
│ RPC envelope          6 opcodes, CBOR arrays, id-correlated │  ← this spec (normative)
├──────────────────────────────────────────────────────────┤
│ Framing               COBS (UART) │ length-prefix (TCP/TLS) │  ← per-transport
├──────────────────────────────────────────────────────────┤
│ Transport             circuits_uart │ gen_tcp │ ssl │ …     │  ← pluggable behaviour
└──────────────────────────────────────────────────────────┘
```

The envelope and typed-payload layers are language-neutral and identical across
all three reference implementations. The framing and transport layers are
per-language but interoperable on the wire.

### The envelope

Every message is a CBOR array whose first element is a single-byte opcode
(values 0–23 encode in one CBOR byte). Each opcode carries only the fields it
needs — there are no perpetually-`nil` slots.

```
0  call          [0, id, method, params]    request; expects a response
1  ok            [1, id, result]            terminal success — closes id
2  error         [2, id, error]             terminal failure — closes id
3  partial       [3, id, seq, result]       streamed item; more may follow; id stays open
4  cancel        [4, id]                     caller closes an open stream
5  notification  [5, method, params]        fire-and-forget; no id, no reply
```

- **`id`** — unsigned integer chosen by the caller, unique among that caller's
  in-flight calls. Correlates responses to calls. An id is *open* from the call
  until a terminal `ok`/`error`; it must not be reused while open. The id space
  is **per-originator** (see Peer symmetry): a `call` with id 5 sent A→B and a
  `call` with id 5 sent B→A are unrelated.
- **`method`** — text string. Dotted namespacing by convention
  (`param.get`, `pubsub.subscribe`, `myapp.do_thing`). The `$` namespace is
  reserved for protocol-level methods (see Handshake).
- **`params`** / **`result`** — any CBOR value; typically an array of positional
  arguments for `params`, and a single typed value for `result`.
- **`error`** — a 3-element array `[code, message, data]`: `code` an integer
  (see Error model), `message` a short text string, `data` any CBOR value or
  `null`.
- **`seq`** — see Streaming.

#### Design rationale (captured for the record)

- **Distinct opcodes instead of flags.** An earlier shape used
  `[1, id, final?, error, result]`. CBOR is self-describing and we dispatch on
  the opcode anyway, so a boolean flag costs a guaranteed byte (`0xf4`/`0xf5`)
  and a perpetually-`nil` slot costs another. Splitting "more follows / done"
  and "ok / error" into separate opcodes removes both and maps the wire opcode
  directly onto Elixir's `{:ok, _} | {:error, _}` — which is also exactly the
  return contract `BB.Bridge` callbacks already use.
- **A normal call is the degenerate stream.** A non-streaming call simply skips
  straight to a terminal `ok`/`error` with zero `partial`s. There is no separate
  "this is a streaming method" concept; the server decides per-response whether
  to emit partials.

### Peer symmetry

The protocol has **no fixed client/server roles**. Both ends of a link may act
as client and server *simultaneously* over the same connection: either end can
issue `call`s and `notification`s, and either end can serve them. This is what
lets a robot both expose its parameters to a GCS *and* read the GCS's
parameters, or a microcontroller both answer queries *and* stream sensor data,
over a single UART.

Mechanically this needs nothing new in the envelope, only a discipline about the
two correlation tables each peer keeps:

- **Outgoing calls** — ids *I* allocated, awaiting responses. Inbound `ok` /
  `error` / `partial` are matched here; I send `cancel` for these.
- **Inbound calls** — ids the *remote* allocated that I am serving. I echo the
  id on my `ok` / `error` / `partial`; inbound `cancel` is matched here.

Because the two tables are keyed by *originator*, equal id values flowing in
opposite directions never collide, and a peer always knows which table an
incoming message belongs to from its opcode (`call`/`cancel` are inbound work;
`ok`/`error`/`partial` answer my outgoing work).

**A peer is not required to serve anything.** It is perfectly valid for one end
to reject every inbound `call` with `error -32601` (method not found) while
still issuing its own calls — i.e. to be effectively call-only. Likewise a peer
may serve only a subset of methods and "nope" the rest. The optional `$.hello`
handshake lets a peer advertise an empty (or partial) method set up front, but
nothing depends on it: the wire contract is simply that any call may be answered
with an error.

### Streaming

Streaming reuses the call id as the stream handle (as CoAP Observe reuses the
request Token), rather than minting a separate subscription id (as Ethereum
`eth_subscribe` / LSP progress tokens do). The caller already holds the id, so
demultiplexing is free, two subscriptions are two independent ids, and
cancellation is "cancel this id".

A subscribe-and-stream sequence:

```
client → [0, 123, "pubsub.subscribe", [["sensor", "imu1"]]]
server → [3, 123, 0, "ok"]              # subscribed; ack is the first partial (seq 0)
server → [3, 123, 1, <message>]         # each forwarded message
server → [3, 123, 2, <message>]
client → [4, 123]                       # cancel
server → [1, 123, null]                 # terminal — stream closed
```

**Sequence numbers.** `seq` appears only on `partial`, because partials are the
only message class with no other loss-detection mechanism: calls and terminal
responses are protected by the caller's request timeout, but a one-way streamed
item that vanishes is otherwise silent. `seq` is a per-stream counter (resets
per id, since ids are not reused while open), starting at 0 on the ack and
incrementing once per partial. A gap (`0,1,3`) signals a drop; out-of-order
(`0,1,3,2`) signals reordering — moot on UART/TCP (both in-order) but correct
insurance for any future datagram transport.

`seq` **wraps at 2^8**. CBOR varint-encodes integers (0–23 → 1 byte, 24–255 → 2
bytes, 256+ → 3 bytes), so a monotonic counter would silently inflate every
frame on a long-lived high-rate stream. Wrapping bounds it to ≤2 bytes.
Detection only aliases if exactly a multiple of 256 consecutive partials are
lost, which on a CRC-framed link is an outage the transport surfaces
independently.

**Backpressure is out of scope for the protocol.** The wire contract is only
that `seq` increments per partial and gaps mean missed data; the protocol
mandates no buffer accounting or flow-control policy. A constrained C
implementation on an AVR does pure send-and-forget with a single per-stream
counter; a rich Elixir server may decimate or apply credit-based throttling.
The one rule that keeps detection meaningful: **a deliberately dropped
(decimated) message still consumes a `seq`**, so a gap is truthful whether the
loss was on the wire or a server-side drop — without the client needing to know
which. An implementation that never drops never has to think about this.

### Error model

`error` codes follow JSON-RPC 2.0's reserved ranges so the meaning is familiar:

| Code | Meaning |
|---|---|
| `-32700` | Parse error (malformed CBOR / frame) |
| `-32600` | Invalid request (well-formed CBOR, invalid envelope) |
| `-32601` | Method not found |
| `-32602` | Invalid params |
| `-32603` | Internal error |
| `≥ 0` | Application-defined |

`data` carries structured detail. The Elixir implementation maps `BB.Error`
(Splode) exceptions into `data` the same way `bb_mcp` already does for JSON-RPC,
so a remote peer sees the real reason (e.g. "Robot is in state `:armed`")
instead of a generic failure.

### Typed payloads & extensibility

The third extensibility requirement — users adding their own message types and
RPC calls — is served by a registered CBOR **tag** for "typed value".
A typed value is:

```
tag(T, [type_name, value])
```

where `type_name` is a stable text string (e.g. `"bb.sensor.imu"`,
`"geometry.pose"`) and `value` is the CBOR encoding of the type's contents — usually a map of
named fields, but a type may define a more compact form (the fixed math types
below use a packed byte string). The tag number
`T` is allocated from CBOR's First-Come-First-Served range (≥ 32768) — see Open
Questions.

Why a tag and a string name rather than a numeric type id or a bare map:

- A generic CBOR decoder that doesn't recognise the tag still parses the value
  and can pass it through untouched — unknown third-party types degrade
  gracefully instead of failing the parse.
- A *string* name is language-neutral. C and Rust have no atoms or module names;
  each implementation keeps a registry mapping `type_name` ⇄ a native
  type/struct (or codec functions). Unknown names decode to a generic
  `{type_name, map}` rather than being forced into a type.

**Atoms travel under a dedicated tag.** A second reserved tag — the *atom tag* —
encodes an Elixir atom as `tag(A, "name")` where the content is a CBOR text
string. This preserves atom-ness losslessly across Elixir↔Elixir round-trips —
a `frame_id`, a PubSub path segment, or an atom-valued field stays an atom —
without the decoder needing schema knowledge of which fields are atoms. Other
languages have no atoms: they read the tag transparently as a plain string and
never emit it.

**Atom safety (Elixir).** The decoder resolves an atom-tagged string with
`String.to_existing_atom/1`, falling back to the plain string if no such atom
exists; it never calls `String.to_atom/1` on wire data, so a hostile or buggy
peer cannot exhaust the atom table. `type_name`s are likewise mapped to modules
only via the explicit registry. The field-name keys of a typed payload's map
are sent as plain strings and resolved to field atoms via the type's own
schema, so they need no per-key tag.

**Math types are fixed-shape, not general tensors.** Every Beam Bots math type
is a known-size `f64` value: `geometry.vec3` (3), `geometry.quaternion` (4,
WXYZ), `geometry.transform` (16, 4×4 row-major), `geometry.covariance3` (9),
`geometry.covariance6` (36). The type name fixes the element count, dtype, and
layout, so the wire carries only the floats: a **packed little-endian `f64`
byte string** of the known length (a CBOR byte string of `8 × count` bytes).
This is a trivial `Nx.to_binary` / `memcpy` on both ends, smaller than a CBOR
array of doubles, and carries no `dtype`/`shape` metadata. Element order is part
of the type contract — quaternion is WXYZ, matrices are 4×4 (and 3×3 / 6×6)
row-major — and is locked by conformance vectors.

Variable-length numeric payloads (e.g. `LaserScan`'s `ranges`/`intensities`)
are plain CBOR arrays of floats. Binary payloads (`Image`) are a CBOR byte
string plus their scalar dimension fields (`height`/`width`/`encoding`/`step`).

There is deliberately **no general `[dtype, shape, data]` tensor encoding** in
the core spec — nothing in `bb` needs one. A future package that genuinely
carries variable-shaped numeric arrays (e.g. `bb_perception` point clouds or
feature maps) can register its own typed payload as an extension; it does not
belong in the core.

### Method naming & introspection

Methods are strings; the protocol defines none of its own application methods.
A reserved `$` namespace carries protocol-level methods:

- `$.hello` — handshake / capability negotiation (below).
- `$.methods` — optional introspection returning the list of methods a peer
  serves, so clients can discover capabilities at runtime.

### Handshake

On connect, each side sends `[0, id, "$.hello", [capabilities]]` where
`capabilities` is a map: `{version, max_frame, tags}` — protocol version,
maximum frame size it will accept, and the set of typed-payload tags/type-names
it understands. The peer replies `ok` with its own capabilities. This lets the
two ends agree a frame-size cap and detect unsupported payload types before they
are used. The handshake is optional for closed point-to-point links (e.g. a
fixed AVR co-processor) where both ends are known at build time.

### Framing

The envelope is encoded to a CBOR byte string, then framed for the transport.

- **TCP / TLS** — 4-byte big-endian length prefix followed by the CBOR bytes.
  The stream is already reliable and in-order, so message boundaries are all
  that's needed. The length is bounded by the negotiated `max_frame`.
- **UART (and other lossy/frameless links)** — **COBS** (Consistent Overhead
  Byte Stuffing): the CBOR bytes (optionally with a trailing CRC-16/CCITT
  appended first) are COBS-encoded and terminated by a `0x00` delimiter. COBS
  has bounded overhead (≤1 byte per 254) and unambiguous resync: after
  corruption, scan to the next `0x00`. A CRC inside the frame lets the receiver
  discard a corrupted frame (which then shows up as a `seq` gap on any stream it
  carried).

Both framers expose the same interface to the layer above: feed bytes, get back
zero or more complete CBOR messages plus a remainder — which pairs naturally
with the `cbor` decoder's `{:ok, item, rest}` return in Elixir, and with
incremental decoders in Rust/C.

### Transport abstraction

There is no ecosystem-wide transport behaviour, but the established pattern
(Thousand Island's `Transport`, Ranch's `ranch_transport`) is a thin behaviour
plus a protocol state machine. The Elixir implementation defines:

```elixir
defmodule BB.RPC.Transport do
  @moduledoc "Pluggable byte-stream transport; the connection owns framing."
  @callback write(state :: term(), iodata()) :: :ok | {:error, term()}
  @callback framing() :: module()   # BB.RPC.Framing.{Cobs,LengthPrefix}
  # Inbound bytes arrive at the owning connection as
  # `{:bb_rpc_bytes, binary()}` messages.
end
```

with concrete implementations `BB.RPC.Transport.{Tcp,Tls,Uart}` — each a thin
byte pipe (`:gen_tcp`, `:ssl`, `circuits_uart`) that declares its framing
(`LengthPrefix` for TCP/TLS, `Cobs` for UART). The connection is a `:gen_statem`
that owns the correlation table (open ids → awaiting caller or stream handler),
the inbound framer buffer, request timeouts, and reconnection. Three real
transports justify the abstraction; it is not speculative.

### Beam Bots integration (the consumer)

BB does not change the protocol. The BB-facing modules ship in the same `bb_rpc`
package, layered on the BB-agnostic protocol core.

**`BB.Bridge` over the wire.** A generic `BB.RPC.Bridge` implements the existing
`BB.Bridge` behaviour by translating its callbacks into calls:

| `BB.Bridge` callback | Method |
|---|---|
| `list_remote/1` | `param.list` |
| `get_remote/2` | `param.get` |
| `set_remote/3` | `param.set` |
| `subscribe_remote/2` | `param.subscribe` (streaming) |
| `handle_change/3` (outbound) | `notification` `param.changed`, or a partial on an inbound subscription |

Because the wire opcodes already map to `{:ok, _} | {:error, _}`, the bridge
callbacks pass results through with no translation layer. `BB.Parameter`'s
public API is unchanged.

**PubSub forwarding.** A `pubsub.subscribe` call carries a path (and optional
message-type filter); the serving side calls `BB.PubSub.subscribe/3` on the
caller's behalf and turns each delivered `{:bb, path, %BB.Message{}}` into a
`partial` on the call's id. On the receiving side, each partial is re-published
into the *local* `BB.PubSub`, closing the loop: remote `BB.PubSub` → wire stream
→ local `BB.PubSub` → local subscribers. The `BB.Message` envelope and its
payload struct are carried as typed payloads. Path segments arrive atom-tagged
from Elixir peers, or as plain strings from others; both resolve via
`String.to_existing_atom/1` (see Atom safety), never `String.to_atom/1`.

---

## Package Structure

The Elixir package is the canonical home for the spec and vectors and holds both
the BB-agnostic protocol core and the BB integration (the core has no `bb`
dependency; the bridge and pubsub modules do). Rust and C live in their own
repositories.

```
beam-bots/bb_rpc/             # Elixir package + canonical spec home (namespace BB.RPC)
├── SPEC.md                   # normative wire specification
├── vectors/                  # language-neutral conformance vectors
│   ├── envelope/*.cbor + .json
│   ├── framing/*.bin
│   └── typed/*.cbor + .json
├── lib/bb/rpc/
│   ├── codec.ex              # envelope encode/decode        ┐
│   ├── typed.ex              # typed-payload + atom tags     │ BB-agnostic
│   ├── framing/{cobs.ex,length_prefix.ex}                   │ protocol core
│   ├── transport/{tcp.ex,tls.ex,uart.ex}                    │ (no bb dep)
│   ├── connection.ex         # :gen_statem; correlation      │
│   ├── server.ex             # method dispatch + streaming    │
│   ├── client.ex                                             ┘
│   ├── bridge.ex             # BB.Bridge over the wire     ┐ BB integration
│   └── pubsub.ex             # BB.PubSub forwarding        ┘ (depends on bb)
├── test/                     # incl. a runner over vectors/
├── mix.exs
└── README.md

beam-bots/bb-rpc-rs/          # Rust reference implementation (crate: bb-rpc)
beam-bots/bb-rpc-c/           # C/C++ reference implementation (no-malloc capable)
```

### Candidate dependencies (to be confirmed during implementation)

| Language | CBOR library | Notes |
|---|---|---|
| Elixir | `cbor` (scalpel-software) | Pure Elixir (no NIF → no ARM cross-compile pain), MIT, actively maintained, `CBOR.Tag` for the typed-payload tag. |
| Rust | `minicbor` (preferred) or `ciborium` | `minicbor` is `no_std`, derive-based, embedded-friendly; `ciborium` for `std`/serde hosts. |
| C/C++ | `TinyCBOR` or `QCBOR` | Both run no-malloc / streaming, suitable for AVR-class targets. |
| UART (Elixir) | `circuits_uart` | `Circuits.UART.Framing` behaviour hosts the COBS framer. |
| TCP/TLS (Elixir) | `:gen_tcp` / `:ssl`, optionally Thousand Island | Acceptor side. |

The specific Rust and C library choices are **not yet verified** and are listed
as candidates (see Open Questions).

---

## User Experience

### Elixir — serving methods

```elixir
defmodule MyServer do
  use BB.RPC.Server

  # Unary call
  def handle_call("param.get", [path], _ctx) do
    {:ok, MyStore.fetch(path)}
  end

  # Streaming call: emit partials, return :stream to keep the id open
  def handle_call("pubsub.subscribe", [path], ctx) do
    BB.PubSub.subscribe(ctx.robot, path)
    {:stream, ctx}
  end

  def handle_info({:bb, path, %BB.Message{} = msg}, ctx) do
    BB.RPC.Server.partial(ctx, [path, msg])
    {:noreply, ctx}
  end
end

{:ok, conn} = BB.RPC.Client.connect(transport: {BB.RPC.Transport.Uart,
  device: "/dev/ttyACM0", speed: 57_600})
{:ok, value} = BB.RPC.Client.call(conn, "param.get", [["motion", "max_speed"]])
{:ok, _ref}  = BB.RPC.Client.stream(conn, "pubsub.subscribe", [["sensor", "imu1"]],
  into: self())
```

### Beam Bots — unchanged API

```elixir
parameters do
  bridge :gcs, {BB.RPC.Bridge, transport: {:tcp, host: "gcs.local", port: 4840}}
end

# Existing BB.Parameter API works over the wire with no changes:
{:ok, params} = BB.Parameter.list_remote(MyRobot, :gcs)
:ok = BB.Parameter.set_remote(MyRobot, :gcs, "PITCH_RATE_P", 0.15)
:ok = BB.Parameter.subscribe_remote(MyRobot, :gcs, "PITCH_RATE_P")
```

### Rust — client

```rust
let mut conn = bb_rpc::Client::connect(Tcp::new("gcs.local:4840")?)?;
let v: f64 = conn.call("param.get", &[&["motion", "max_speed"]])?;
let mut sub = conn.stream("pubsub.subscribe", &[&["sensor", "imu1"]])?;
while let Some(msg) = sub.next()? { /* … */ }
```

### C (embedded, no-malloc sketch)

```c
bbrpc_conn conn;
bbrpc_init(&conn, &uart_io);                 /* caller supplies byte I/O */
bbrpc_call(&conn, "param.get", path_args);   /* writes one framed call  */
/* in the receive loop: feed bytes, get decoded messages via callback   */
bbrpc_feed(&conn, buf, n);                   /* invokes on_message(...)  */
```

---

## Acceptance Criteria

### Must Have

- [ ] `SPEC.md`: normative wire spec for the 6 opcodes, error model, typed-payload and atom tags, framing (COBS + length-prefix), and handshake.
- [ ] Language-neutral conformance vectors covering envelope, framing, and typed payloads (including the fixed math types and the `Image`/`LaserScan` payloads).
- [ ] Elixir implementation: codec, COBS + length-prefix framing, TCP/TLS/UART transports, `:gen_statem` connection with id correlation and request timeouts, server method dispatch, and streaming (partial/cancel) — passing all vectors.
- [ ] Rust implementation: codec + framing + a TCP transport, passing the shared vectors.
- [ ] C/C++ implementation: codec + COBS framing + a UART byte-I/O hook, no-malloc capable, passing the shared vectors.
- [ ] `bb_rpc`: `BB.RPC.Bridge` implementing `BB.Bridge` (`param.{list,get,set,subscribe}`) and `BB.PubSub` forwarding via streaming.
- [ ] `seq` drop/reorder detection on partials, wrapping at 2^8.
- [ ] Symmetric peers: each connection can issue *and* serve calls simultaneously (dual correlation tables, per-originator id spaces); a peer that serves no methods replies `error -32601` to all inbound calls.
- [ ] Atom tag: atoms travel as `tag(A, "name")`, decode via `String.to_existing_atom/1` (fallback to string), never `String.to_atom/1` on wire data.
- [ ] Tests and documentation for each implementation.

### Should Have

- [ ] `$.hello` handshake with version + `max_frame` + supported-tags negotiation.
- [ ] `$.methods` introspection.
- [ ] CRC-16 inside UART frames for corruption detection.
- [ ] Reconnection handling in the Elixir connection state machine.
- [ ] C++ wrapper over the C core.
- [ ] Benchmarks: encode/decode and round-trip latency on an embedded target.

### Won't Have

- [ ] A mandated backpressure/flow-control policy (implementation-specific by design).
- [ ] Stream resumption / replay after disconnect (detect-only, no recovery).
- [ ] Built-in authentication (use TLS / network-level security).
- [ ] Wire compatibility with MessagePack-RPC (the model descends from it, but the opcode set diverges — split ok/error responses, streaming partials, and cancel).
- [ ] A datagram (UDP) transport in the first release (the design leaves room for one).

---

## Open Questions

1. **CBOR tag numbers** for the typed-payload tag and the atom tag. Pick two in
   the FCFS range (≥ 32768); decide whether to register them with IANA or
   document them as private.
2. **Rust and C library choices.** Verify current maintenance/versions of
   `minicbor` vs `ciborium`, and `TinyCBOR` vs `QCBOR`, including no-malloc
   behaviour on AVR-class targets.
3. **Timeouts & in-flight calls across reconnect.** Default request timeout;
   whether open stream ids survive a transparent reconnect or are torn down.
4. **CRC choice & placement** for UART frames (CRC-16/CCITT vs CRC-32), and
   whether it is mandatory or negotiated.
5. **Max frame size** default and whether oversize frames are an error or are
   chunked at the framing layer (kept separate from streaming `partial`s, which
   are whole values, not fragments).
6. **Repository layout.** One repo hosting spec + vectors + Elixir impl, or a
   dedicated spec/vectors repo that all three implementations vendor.

---

## Prior art

Several CBOR-based RPC systems already exist. Surveying them validates the shape
of this design — the same capabilities recur independently — while showing where
the embedded/lossy-link target leads somewhere none of them go.

| Axis | **`bb_rpc`** | [L-Briand/CBOR-RPC](https://github.com/L-Briand/CBOR-RPC) | [cbor-rpc-py](https://github.com/mesudip/cbor-rpc-py) | [Smithy RPC v2 CBOR](https://smithy.io/2.0/additional-specs/protocols/smithy-rpc-v2-cbor.html) | [go-coordinate cborrpc](https://pkg.go.dev/github.com/swiftlobste/go-coordinate/cborrpc) |
|---|---|---|---|---|---|
| Envelope | positional array, 6 single-byte opcodes | sequential header list `[vrpc,vapi,id,event,name]` + message + raw payload | undocumented (CBOR, optional JSON) | HTTP POST + CBOR body | map `{id,method,params}` |
| Correlation | explicit id, per-originator | explicit id; same id usable both ways | bidirectional | implicit (HTTP request/response pair) | explicit id, sequential/non-unique |
| Symmetric / bidirectional | yes | yes | yes | no (client→server) | pipelined, client→server |
| Streaming / push | id-correlated `partial`s + `seq` | separate raw payload field; notifications | log/progress streaming, topic events | Amazon event-stream (chunked) | none |
| Drop detection | per-stream `seq` (wrap 2⁸) | none | none | n/a (reliable HTTP) | none |
| Error model | JSON-RPC codes + `data` | none (custom event+name) | — | `__type` Shape-ID + `message` | `error.message` |
| Extensibility | typed-payload + atom CBOR tags | custom event types | — | Smithy model shapes | ad-hoc CBOR tags |
| Byte-stream framing | COBS (UART) / length-prefix (TCP/TLS) | 3-part concat (assumes msg-preserving channel) | buffering transformer | HTTP / event-stream | CBOR-in-CBOR (tag 24) |
| Target | embedded UART → TCP/TLS | WebSocket / BT / TCP | TCP / SSH / stdio | HTTP services (AWS) | daemon socket |

**What recurs (and so validates the design).** Bidirectional symmetry
(cbor-rpc-py, L-Briand), per-call cancellation and topic/progress streaming
(cbor-rpc-py), pluggable transport with a buffering framer (both), and explicit
ids reused in both directions all appear independently. L-Briand's "both sides
may use the same id" is precisely our per-originator id space.

**What is distinctive here.** Two properties none of the four have, both falling
out of the embedded target the others don't share: **per-stream `seq` drop
detection** and **COBS framing**. Every surveyed system rides a
message-preserving transport (HTTP, WebSocket, a reliable socket), so none needs
to detect dropped frames or delimit a frameless UART. This proposal treats the
AVR-over-UART case as first-class, which is what forces both. The single-byte
positional opcodes are also the most compact envelope of the group (versus maps,
named keys, or a protocol version on every message), and the atom tag is unique
to the Elixir-fidelity requirement.

**Ideas considered.** L-Briand carries large raw blobs in a payload field
*outside* the structured CBOR; we embed them (e.g. `Image` data as a CBOR byte
string), which carries no nesting penalty since CBOR byte strings are
length-prefixed with no escaping. Smithy encodes timestamps as CBOR tag 1; our
`BB.Message` times are nanosecond integers, so no dedicated time tag is needed.

**Smithy RPC v2 CBOR** is the useful contrast at the other end of the spectrum:
schema-first, HTTP-bound, codegen-driven, correlation implicit in the HTTP pair.
It is the right design for large-scale web services (AWS uses it) and the wrong
one for a UART link to a microcontroller — which is exactly why this protocol
exists alongside it rather than adopting it.

---

## References

- [RFC 8949 — Concise Binary Object Representation (CBOR)](https://www.rfc-editor.org/rfc/rfc8949.html)
- [IANA CBOR Tags registry](https://www.iana.org/assignments/cbor-tags)
- [JSON-RPC 2.0 Specification](https://www.jsonrpc.org/specification) — error codes, request/response/notification model
- [MessagePack-RPC spec](https://github.com/msgpack-rpc/msgpack-rpc/blob/master/spec.md) — positional-array envelope semantics
- [RFC 7641 — Observing Resources in CoAP](https://www.rfc-editor.org/rfc/rfc7641.html) — id/token-correlated streaming prior art
- [Consistent Overhead Byte Stuffing (COBS)](http://stuartcheshire.org/papers/COBSforSIGCOMM/)
- [`cbor` (Elixir)](https://hexdocs.pm/cbor/readme.html) · [`minicbor`](https://docs.rs/minicbor) · [`ciborium`](https://docs.rs/ciborium) · [TinyCBOR](https://github.com/intel/tinycbor) · [QCBOR](https://github.com/laurencelundblade/QCBOR)
- [`Circuits.UART.Framing` behaviour](https://github.com/elixir-circuits/circuits_uart) · [Thousand Island transport](https://hexdocs.pm/thousand_island/)
- `bb/lib/bb/bridge.ex`, `bb/lib/bb/parameter.ex`, `bb/lib/bb/pub_sub.ex`, `bb/lib/bb/message.ex` — the BB primitives this protocol serves
- `feetech/lib/feetech/protocol.ex` — in-house framed-binary-protocol prior art
