<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Proposal 0023: bb_geo

**Status:** Draft
**Author:** James Harton
**Created:** 2026-08-04
**Dependencies:** `bb`

---

## Summary

A sibling package providing the geodetic maths and shared message vocabulary that
every GNSS driver needs and no arm should ever compile: the WGS 84 ellipsoid,
conversions between geodetic coordinates, ECEF, and local ENU/NED frames,
great-circle and ellipsoidal distance and bearing, and the `NavSatFix` family of
payloads that receiver drivers publish.

It deliberately introduces **no new behaviour**. `BB.Sensor` plus a shared message
already makes GNSS drivers interchangeable on the read path, and `BB.Bridge`
already abstracts their configuration systems. What was missing was a common
vocabulary and the maths to make a fix useful.

---

## Motivation

### A latitude is not a position

A robot cannot do anything with degrees. To compare a GNSS fix against a link
pose, feed it to a Kalman filter, or drive a floating joint (Proposal 0022), the
fix has to become metres in a local Cartesian frame. That conversion needs the
reference ellipsoid, a datum, a chosen local origin, and a decision about axis
convention.

`BB.Math` currently has `Vec3`, `Quaternion`, `Transform`, `Covariance3`, and
`Covariance6` — and nothing geodetic at all. `bb_ublox` publishes latitude and
longitude and stops there, because there is nowhere for the next step to live.

### Every driver would otherwise reinvent it, subtly wrongly

Geodesy is a field where plausible-looking code is quietly wrong:

- Treating the earth as a sphere rather than an ellipsoid: up to ~0.5% distance
  error, which is 5 m per kilometre.
- Confusing height above the ellipsoid with height above mean sea level: tens of
  metres of vertical offset, varying by location.
- Confusing ENU with NED: two sign errors and a swapped axis pair, producing
  results that look almost right near the origin and diverge with distance.
- Using 32-bit floats for ECEF coordinates: ECEF magnitudes are around
  6.4 × 10⁶ m, so `f32`'s ~7 significant digits give roughly **half-metre**
  resolution before any arithmetic. This one is especially insidious because it
  produces plausible numbers.

These are exactly the errors a shared, tested library exists to prevent. Every
one of them is silent.

### Why a package and not core

Not every robot needs this. `bb` has so far been used exclusively for fixed-base
arms, none of which will ever want an ellipsoid. Core should stay lean, and a
geodesy library has no claim on it — unlike Proposal 0022, which touches
kinematics and therefore has nowhere else to go.

The two proposals divide cleanly along that line: 0022 is core because forward
kinematics is core; `bb_geo` is a package because coordinate maths is
self-contained.

The message payloads follow the maths, for the same reason and one more. It would
be easy to argue from precedent that they belong in core, since
`BB.Message.Sensor.Imu`, `LaserScan`, `Range`, `BatteryState`, `PowerState`, and
`Image` all live there today. But that precedent is weaker than it looks:

| Payload in core | Referenced by core | Referenced by any package |
|---|---|---|
| `JointState` | 5 files | yes |
| `Imu` | docstrings only | `bb_sensor_bmi323` |
| `PowerState` | no | `bb_sensor_ina219` |
| `LaserScan` | no | **nothing** |
| `Range` | no | **nothing** |
| `BatteryState` | no | **nothing** |
| `Image` | no | **nothing** |

`JointState` is genuinely load-bearing — transmissions, the actuator command
pipeline, and `BB.Sensor.publish_joint_state/3` all use it. Everything below it is
vocabulary core never touches, and four of the seven are used by *nothing in the
ecosystem at all*: payloads added ahead of drivers that never arrived.

So the direction of travel is core **shedding** driver-facing payloads rather than
accumulating more. Adding `NavSatFix` to core would be moving against that, and
`bb_geo` is where a payload that only GNSS drivers publish and only GNSS consumers
read belongs. Unbundling the existing ones is a separate question, out of scope
here.

### Why no behaviour

Worth stating explicitly, because "a common behaviour for GNSS modules" is the
intuitive design and it turns out to be unnecessary:

- **The read path already has an interface: the message type.** Nothing calls
  `GNSS.read_position(driver)`. A `BB.Sensor` publishes and subscribers
  pattern-match, so swapping `{BB.UBlox.Sensor, …}` for `{BB.Unicore.Sensor, …}`
  in the DSL leaves every consumer untouched — *provided they publish the same
  message*. Pubsub supplies the polymorphism; a behaviour would add callbacks
  nobody would call.
- **The configuration path already has `BB.Bridge`.** `list_remote/1`,
  `get_remote/2`, `set_remote/3`, and `subscribe_remote/2` with string parameter
  ids is precisely "a common interface to different vendors' configuration
  systems". `bb_ublox` implements it over a 32-bit keyed binary store; a Unicore
  driver would implement it over `CONFIG` text commands; a MediaTek driver over
  `$PMTK` sentences. Wildly different underneath, identical contract.

A `BB.GNSS` behaviour would sit between two behaviours that already cover both
halves. So the shared thing is a *message plus a maths library*, not a callback
contract.

### No estimator lives here

A `BB.Estimator` fusing a fix with AHRS orientation into a pose — the thing that
would drive Proposal 0022's floating joint — does **not** belong in this package.

The test is whether any of it is *geo-specific*. It isn't. The coordinate
conversion is geo-specific, but that is a pure function this package already
provides. Everything else — fusing two input streams, gating on input validity,
handling stale inputs, publishing derived state — is exactly what `BB.Estimator`
(Proposal 0018) already abstracts, and is identical whether the input is GNSS,
wheel odometry, or visual odometry.

So the split is: `bb_geo` turns degrees into metres; a generic estimator turns
metres into a pose. Putting the estimator here would tie generic fusion machinery
to one sensor modality.

### Configuration is where receivers actually differ

The observation that motivated this proposal: across vendors, sensor output is
essentially identical — position, accuracy, velocity, fix status — while
configuration is radically different. Compare what a driver must speak to set a
measurement rate:

| Vendor | Configuration mechanism |
|---|---|
| u-blox (protocol 27+) | `UBX-CFG-VALSET`, 32-bit typed key store |
| u-blox (pre-27) | Per-topic binary structs, read-modify-write |
| Unicore UM980 | `CONFIG` / `MODE` text commands |
| Quectel LC29H | `$PQTM` proprietary sentences |
| MediaTek / Airoha | `$PMTK` numbered packets |

That divergence stays in the drivers, behind `BB.Bridge`. Only the output
vocabulary is shared, and that is what this package holds.

---

## Design

### Coordinates and datums

```elixir
defmodule BB.Geo.Ellipsoid do
  @moduledoc "Reference ellipsoid parameters."
  defstruct [:semi_major_axis, :flattening, :name]

  def wgs84, do: %__MODULE__{semi_major_axis: 6_378_137.0,
                             flattening: 1 / 298.257223563, name: :wgs84}
  def grs80, do: ...
end
```

```elixir
defmodule BB.Geo.LLA do
  @moduledoc """
  A geodetic coordinate.

  `latitude` and `longitude` are degrees. `height` is metres **above the
  reference ellipsoid** — not above mean sea level, and not above terrain.

  The distinction matters and is the commonest source of vertical error in
  GNSS work: the geoid departs from the WGS 84 ellipsoid by roughly -105 m to
  +85 m depending on where you are. A receiver reports both, and confusing them
  puts you tens of metres out with no other symptom.
  """

  defstruct [:latitude, :longitude, :height, :ellipsoid]
end
```

**`height` and `altitude` are two different words for two different things,
consistently across the ecosystem.** `height` is always above the ellipsoid;
`altitude` is always above mean sea level. `BB.Geo.Message.NavSatFix` already
carries both under exactly those names, so `LLA.height` matches rather than
inventing a third convention. Nothing anywhere should use "altitude" loosely.

### Conversions

```elixir
BB.Geo.lla_to_ecef(%LLA{}) :: Vec3.t()
BB.Geo.ecef_to_lla(Vec3.t(), Ellipsoid.t()) :: LLA.t()
```

ECEF is a `BB.Math.Vec3` in metres, so it composes with everything else in
`BB.Math` for free.

Local frames are the ergonomic centre of the package. Converting one fix in
isolation is rare; converting a stream against a fixed origin is the norm, and
the rotation matrix for that origin can be computed once:

```elixir
defmodule BB.Geo.LocalFrame do
  @moduledoc """
  A local tangent-plane frame anchored at a geodetic origin.

  Holds the origin's ECEF position and the precomputed rotation from ECEF into
  the local frame, so converting a stream of fixes costs one subtraction and one
  matrix multiply each.
  """
  defstruct [:origin, :origin_ecef, :rotation, :convention]

  @spec new(LLA.t(), convention: :enu | :ned) :: t()
end
```

```elixir
frame = BB.Geo.LocalFrame.new(origin_lla, convention: :ned)

BB.Geo.LocalFrame.to_local(frame, %LLA{}) :: Vec3.t()
BB.Geo.LocalFrame.from_local(frame, Vec3.t()) :: LLA.t()
```

`LocalFrame` provides the **mechanism** for anchoring a local frame. It does not
decide the **policy** — whether the origin is the first usable fix, a constant
configured for the site, or something that migrates as the robot travels. That is
a robot-configuration concern and belongs in the DSL, most likely as part of the
future motion-planning extension (Proposal 0006) rather than here. This package
takes an origin and converts against it.

**Both ENU and NED are first-class and must be named at construction.** There is
no default. Robotics and ROS conventionally use ENU for map frames; aerospace,
MAVLink, and u-blox's own `NAV-PVT` velocity fields use NED. Silently picking one
is how sign errors happen, so the API refuses to.

### Distance and bearing

```elixir
BB.Geo.distance(from, to, method: :haversine)  # spherical, fast
BB.Geo.distance(from, to, method: :karney)     # ellipsoidal, ~15nm accurate
BB.Geo.initial_bearing(from, to)
BB.Geo.final_bearing(from, to)
BB.Geo.destination(from, bearing, distance)
```

Two distance methods because the tradeoff is real: haversine is cheap and wrong
by up to 0.5%; Karney's algorithm is accurate and iterative. Naming the method at
the call site makes the choice explicit rather than hidden in a default.

### Implemented with Nx

The coordinate transforms are written as `defn`, not plain Elixir float
arithmetic. Four reasons, in ascending order of importance:

1. **Composition.** `BB.Math.Vec3`, `Transform`, and `Covariance3` are all
   tensor-backed. Geo maths that speaks the same language chains with them
   without marshalling at every boundary.
2. **Consistency.** `bb`'s kinematics is already `defn` (`BB.Robot.Kinematics.Defn`,
   `BB.Math.Defn`), so this is the established idiom rather than a new one.
3. **Batching for free.** The same code converts one fix or a hundred thousand.
   A robot rarely needs that; offline log analysis, map building, and trajectory
   post-processing do, and they get it without a second implementation.
4. **Differentiability.** This is the one that decides it. A Kalman or factor-graph
   estimator fusing GNSS needs the Jacobian of its measurement model — which is
   the Jacobian of the geodetic-to-local transform. Written as `defn`, that comes
   from `Nx.Defn.grad/1`. Written as plain floats, somebody hand-derives it, gets
   it subtly wrong, and the filter diverges in a way that looks like bad GPS.

Given `bb` already has `BB.Estimator` and this package exists partly to feed it,
hand-derived Jacobians would be a trap laid for our own future selves.

#### One carve-out: Karney's geodesic

`defn` suits closed-form and fixed-iteration maths. It suits data-dependent
iteration poorly, because the loop count has to be static to compile and
vectorise.

- **`lla_to_ecef/1`** is closed-form. `defn`.
- **`ecef_to_lla/2`** has no exact closed form for latitude, so it uses Bowring's
  method. A single iteration is accurate to roughly a micrometre on the ellipsoid,
  so implementations run a *fixed* one or two passes rather than looping to
  convergence — which makes it straight-line arithmetic, not a loop. `defn`.
- **Local frame conversions** are a subtraction and a matrix multiply. `defn`,
  and the batching benefit is real here since converting a whole track is one
  call.
- **Haversine** is closed-form. `defn`.
- **Karney's geodesic** is series expansions with data-dependent convergence.
  Plain Elixir. Forcing it into `defn` would mean a worst-case fixed iteration
  count on every call and would not vectorise usefully anyway.

#### Precision, and a backend trap

All geodetic arithmetic is `:f64`. This is a correctness requirement, not a
preference — see the `f32` ECEF resolution problem in the Motivation. `BB.Math`'s
existing types already use `:f64` (`Covariance3` casts with
`Nx.as_type(tensor, :f64)`), so this is consistent with core.

Two ways to lose it, both silent:

- An innocuous `Nx.tensor([...])` without an explicit type defaults to `:f32`,
  giving half-metre errors that look like GNSS noise.
- **Choosing `defn` makes the backend a correctness concern, not just a
  performance one.** Consumer GPUs are dramatically slower at `f64` than `f32`,
  and some accelerator backends silently downcast rather than refusing. A user
  who sets an EXLA GPU backend to speed up kinematics could quietly degrade their
  position solution to half-metre resolution.

So the package must assert `:f64` on its outputs in tests rather than assuming
it, and its usage rules must warn about the backend interaction. This is a real
cost of the `defn` decision and worth paying with eyes open.

### Messages

The payloads currently defined in `bb_ublox` move here, renamed for their new
home:

```elixir
BB.Geo.Message.NavSatFix     # position, velocity, accuracy, fix status
BB.Geo.Message.Satellites    # per-satellite constellation/CNO diagnostics
```

`NavSatFix` follows the shape `bb_ublox` already ships and validated: **lowest
common denominator plus nil-able extras**. Position, accuracy, NED velocity, and
fix status are required, because every receiver reports them. RTK carrier
solution status, full position covariance, and heading are optional and `nil`
when unavailable.

A strict lowest-common-denominator message would be a mistake. A ZED-F9P has
genuinely more to say than a NEO-M8N — carrier solution state, a real covariance
matrix — and flattening to the cheapest receiver's capability throws away exactly
what the expensive one was bought for.

Absent values are `nil`, never zero. An unavailable covariance and a zero
covariance are different facts, and the latter reads as a perfect fix.

### Shared vocabulary

The enumerated atoms must be identical across drivers, or a downstream `case`
breaks when the driver is swapped. They live with the message:

```elixir
@type fix_type :: :none | :dead_reckoning | :fix_2d | :fix_3d
                  | :gnss_dead_reckoning | :time_only
@type carrier_solution :: :none | :float | :fixed
@type constellation :: :gps | :sbas | :galileo | :beidou | :imes
                       | :qzss | :glonass | :navic
```

`constellation` covers the full set because it is a property of the satellites,
not the receiver — BeiDou is as constellation-agnostic a concept as GPS, and a
u-blox M8, a Unicore UM980, and an NMEA-only MediaTek module all report it.

---

### Verifying correctness

Choosing `defn` makes correctness harder to eyeball, so the test strategy matters
more than usual. Three tiers, in descending order of authority:

**1. Authoritative reference data.** Karney's **`GeodTest.dat`** is 500,000
geodesic cases computed to 15 nm accuracy by construction. It is what other
implementations are themselves validated against, so it beats cross-checking
against any of them. Published test vectors serve the same role for ECEF↔LLA.

**2. Cross-checks against existing Elixir packages**, as test-only dependencies.
Useful as characterisation, weaker than tier 1 — if a package disagrees with us,
that alone doesn't say which is wrong.

| Package | Covers | Notes |
|---|---|---|
| [`sidereon`](https://hex.pm/packages/sidereon) | `Coordinates.geodetic_to_itrs/1`, `to_geodetic/1`, `Geodesic.inverse/4`, `direct/4` | Near-exact surface match. MIT, Rust NIF. **Its geodetic struct uses kilometres** where ours uses metres — a 1000× error waiting to happen in the harness. ITRS rather than ECEF: same frame for our purposes, differing by centimetres. |
| [`geocalc`](https://hex.pm/packages/geocalc) | Distance, bearing, destination | 1.3M downloads, the mature option — but **spherical** (movable-type formulas). Characterises `:haversine` only; using it for `:karney` would validate against the wrong answer. |
| [`coord`](https://hex.pm/packages/coord) | UTM | Apache-2.0. Validated against a reference implementation over hundreds of thousands of coordinates. |

Prefer `sidereon` over [`orbis`](https://hex.pm/packages/orbis) despite the
latter's "0 ULP Skyfield parity" claim: they are the same author, `orbis`'s
repository now 404s, and `sidereon` has the more recent release.

**3. Round-trips and properties.** `lla_to_ecef |> ecef_to_lla` returning the
input to sub-millimetre, ENU and NED agreeing after axis permutation, and so on.
Necessary but **not sufficient** — a systematically wrong implementation round-trips
perfectly. These catch typos, not misconceptions.

Additionally, because the transforms are `defn`, the **Jacobians should be checked
against finite differences** of the forward transform. That validates the
differentiability the design depends on, rather than assuming `grad` produces
something sensible.

---

## Package Structure

```
bb_geo/
├── lib/bb/geo/
│   ├── ellipsoid.ex           # WGS 84, GRS 80
│   ├── lla.ex                 # geodetic coordinate struct
│   ├── local_frame.ex         # ENU/NED tangent-plane frames
│   ├── conversions.ex         # LLA <-> ECEF
│   ├── distance.ex            # haversine, Karney, bearings
│   ├── message/
│   │   ├── nav_sat_fix.ex
│   │   └── satellites.ex
│   └── utm.ex                 # Should Have
├── test/
└── usage-rules.md
```

### Dependencies

- `bb` — for `BB.Math.Vec3`, `BB.Math.Covariance3`, and `BB.Message`
- Nx, transitively via `bb`

No new external dependencies. The conversions are closed-form or simple
iterations and do not warrant pulling in a geodesy library.

### Ecosystem shape

```
bb  ──────────────┐
                  ├── bb_geo ──┬── bb_ublox
                  │            ├── bb_gpsd
                  │            └── bb_unicore …
                  └── bb_ik_dls, bb_sensor_*, …
```

`bb_ublox` gains a `bb_geo` dependency and deletes its local `NavSatFix` and
`Satellites`. Since nothing depends on `bb_ublox` yet and it has never been
released, this costs no migration.

---

## User Experience

Converting a fix stream into a local metric frame — the core use case:

```elixir
defmodule Rover.Localisation do
  use GenServer

  alias BB.Geo.LocalFrame
  alias BB.Geo.Message.NavSatFix

  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)
    BB.subscribe(robot, [:sensor, :chassis, :gps])
    {:ok, %{robot: robot, frame: nil}}
  end

  # The first usable fix anchors the local frame.
  def handle_info({:bb, _path, %BB.Message{payload: %NavSatFix{fix_ok: true} = fix}},
                  %{frame: nil} = state) do
    origin = %BB.Geo.LLA{latitude: fix.latitude,
                         longitude: fix.longitude,
                         height: fix.height}

    {:noreply, %{state | frame: LocalFrame.new(origin, convention: :ned)}}
  end

  def handle_info({:bb, _path, %BB.Message{payload: %NavSatFix{fix_ok: true} = fix}}, state) do
    position = LocalFrame.to_local(state.frame, fix_to_lla(fix))
    # position is a BB.Math.Vec3 in metres, NED, relative to the origin
    {:noreply, state}
  end

  # Fixes the receiver doesn't vouch for are dropped, not used.
  def handle_info({:bb, _path, _message}, state), do: {:noreply, state}
end
```

Distance and bearing to a waypoint:

```elixir
iex> here = %BB.Geo.LLA{latitude: -41.2865, longitude: 174.7762, height: 0.0}
iex> there = %BB.Geo.LLA{latitude: -41.2900, longitude: 174.7800, height: 0.0}
iex> BB.Geo.distance(here, there, method: :karney)
505.6
iex> BB.Geo.initial_bearing(here, there)
141.2
```

Driver-agnostic consumption — the point of a shared message:

```elixir
# Either driver, same subscriber code.
sensor :gps, {BB.UBlox.Sensor, port: "/dev/ttyAMA0"}
sensor :gps, {BB.GPSD.Sensor, host: "localhost"}
```

---

## Acceptance Criteria

### Must Have

- [ ] `BB.Geo.Ellipsoid` with WGS 84 parameters
- [ ] `BB.Geo.LLA` struct, documented as height above ellipsoid
- [ ] Coordinate transforms implemented as `defn`, with Karney's geodesic the
      documented exception
- [ ] `lla_to_ecef/1` and `ecef_to_lla/2`, verified against published reference
      values rather than only round-tripped
- [ ] `Nx.Defn.grad/1` produces correct Jacobians for the transforms, checked
      against finite differences of the forward function
- [ ] Those Jacobians are **finite and correct across the operating envelope,
      including near-polar latitudes**. `ecef_to_lla/2` is built from `atan2` and
      `sqrt`, which are singular at the poles and at the geocentre; a gradient that
      goes `NaN` rather than merely large would fill a downstream filter with `NaN`
      and give no indication why
- [ ] `BB.Geo.LocalFrame` supporting both `:enu` and `:ned`, with no default
- [ ] `to_local/2` and `from_local/2` round-tripping to sub-millimetre
- [ ] Haversine and ellipsoidal distance, initial and final bearing, the latter
      verified against Karney's `GeodTest` dataset
- [ ] All arithmetic in `:f64`, asserted by test on outputs rather than assumed
- [ ] Usage rules warn that an accelerator backend may downcast or be slow at
      `:f64`, making backend choice a correctness concern for this package
- [ ] `BB.Geo.Message.NavSatFix` and `BB.Geo.Message.Satellites`
- [ ] Shared `fix_type` / `carrier_solution` / `constellation` vocabulary
- [ ] `bb_ublox` migrated to depend on `bb_geo` and its local payloads deleted
- [ ] `usage-rules.md`, per Proposal 0007

### Should Have

- [ ] `BB.Geo.destination/3` — project a point along a bearing
- [ ] UTM, and possibly MGRS
- [ ] Geoid separation helpers, so height above mean sea level is derivable
- [ ] Cross-check tests against `sidereon` and `geocalc` as test-only
      dependencies, on top of the reference data

### Won't Have

- [ ] Any `BB.GNSS` behaviour — see the Design rationale
- [ ] Vendor-specific configuration abstractions; those stay in drivers behind
      `BB.Bridge`
- [ ] Kinematic or joint changes — Proposal 0022
- [ ] Map tiles, geofencing, or route planning
- [ ] Coordinate transforms beyond WGS 84-family datums; no full PROJ
- [ ] A fix-to-pose estimator. Nothing about that fusion is geo-specific, so it
      belongs with generic estimation
- [ ] Local-frame origin *policy* — first-fix, configured, or migrating. This
      package provides the mechanism; the policy is DSL configuration, most likely
      in the motion-planning extension
- [ ] Usability without `bb`. This is BB-specific and splitting a
      dependency-free core would be speculative

---

## Open Questions

1. **Which of `sidereon`'s geodesic implementations backs `inverse/4`?** It exposes
   the standard `direct`/`inverse` geodesic problems, but those names are used by
   both Vincenty and Karney. It matters: Vincenty fails to converge for
   near-antipodal points and Karney does not, which is exactly the case a
   cross-check should exercise. Determine before relying on it as a reference.

---

## References

- [WGS 84 / NGA TR8350.2](https://earth-info.nga.mil/) — ellipsoid parameters
- Karney, C.F.F. (2013), *Algorithms for geodesics*, J. Geodesy 87(1) —
  ellipsoidal distance, plus the `GeodTest` reference dataset
- [ROS `sensor_msgs/NavSatFix`](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)
  — prior art for the payload shape
- [gpsd JSON `TPV`](https://gpsd.gitlab.io/gpsd/gpsd_json.html) — evidence that
  the output vocabulary genuinely is receiver-independent
- Proposal 0022 (Multi-DoF Joints) — the kinematic half of mobile-robot support
- Proposal 0018 (`BB.Estimator`) — how a fix would become a pose
- Proposal 0007 (Usage Rules) — the `usage-rules.md` requirement
