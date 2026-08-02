<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal 0020: bb_rpc

**Status:** Rejected
**Author:** James Harton
**Created:** 2026-06-01
**Rejected:** 2026-06-03 — shelved pending a concrete use case

---

## Summary

A small, extensible CBOR-based RPC protocol carrying request/response calls,
server-streaming responses, and notifications over any byte stream (UART, TCP,
TLS). Proposed as a normative wire specification plus reference implementations
in Elixir, C/C++ and Rust sharing a set of conformance vectors. The protocol core
would have no dependency on BB types; the Elixir package would additionally
provide the `BB.Bridge` and `BB.PubSub` integration that motivated it.

Named `bb_rpc` rather than a generic `cbor-rpc`, which already denotes at least
three unrelated projects.

The design covered six single-byte opcodes (`call`, `ok`, `error`, `partial`,
`cancel`, `notification`) encoded as CBOR arrays and mapping onto the existing
`BB.Bridge` return contract; server-streaming correlated to the call id with a
per-stream sequence number for drop and reorder detection; peer symmetry, so
both ends can issue and serve calls over one connection; two reserved CBOR tags
for typed payloads and for lossless atoms; COBS framing for UART and 4-byte
length-prefix framing for TCP/TLS over a pluggable byte-stream transport
behaviour; and fixed-shape packed little-endian `f64` for math types, with no
general tensor encoding in the core.

---

## Why it was rejected

Shelved rather than refuted. The design work stands; what it lacked was a
concrete problem pulling it into existence.

An RPC protocol is expensive to get wrong. Its choices — opcode set, framing,
extensibility tags, streaming semantics, backpressure — are the kind that
harden on first release and are painful to revise afterwards, because every
implementation in every language has to move together. Speculating about the
right answers without a real bridge needing them risks paying that cost for a
shape nobody turns out to want. A single real use case would settle several of
the open questions immediately.

The same reasoning that deferred ownership and arbitration in proposal 0021
applies here: wait for the conflict rather than design against an imagined one.

A spike was built and archived at
[`beam-bots/bb_rpc`](https://github.com/beam-bots/bb_rpc) — Elixir and Rust
reference implementations. It is the starting point if this is revived, not a
maintained package; `bb-sync` skips it, so it is not cloned into the workspace.

Reviving this means opening a new proposal with the motivating use case stated
up front, reusing whatever of the design below still fits.

---

## References

- [`beam-bots/bb_rpc`](https://github.com/beam-bots/bb_rpc) — archived spike
- Prior art surveyed in the original proposal: L-Briand/CBOR-RPC, cbor-rpc-py,
  Smithy RPC v2 CBOR, go-coordinate cborrpc
