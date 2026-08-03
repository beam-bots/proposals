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
  @moduledoc "A geodetic coordinate: degrees, degrees, metres above the ellipsoid."
  defstruct [:latitude, :longitude, :altitude, :ellipsoid]
end
```

`altitude` is unambiguously **height above the ellipsoid**, not above mean sea
level. Geoid separation is a distinct concern and receivers report both; keeping
the struct's meaning fixed prevents the commonest vertical error.

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

### Numeric precision

All geodetic arithmetic is `:f64`. This is a correctness requirement, not a
preference — see the `f32` ECEF resolution problem in the Motivation. `BB.Math`'s
existing types already use `:f64` (`Covariance3` casts with
`Nx.as_type(tensor, :f64)`), so this is consistent with core rather than novel.

The package should state this in its usage rules and assert it in tests, because
an innocuous `Nx.tensor([...])` without an explicit type will default to `:f32`
and produce half-metre errors that look like GNSS noise.

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
                         altitude: fix.height}

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
iex> here = %BB.Geo.LLA{latitude: -41.2865, longitude: 174.7762, altitude: 0.0}
iex> there = %BB.Geo.LLA{latitude: -41.2900, longitude: 174.7800, altitude: 0.0}
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
- [ ] `lla_to_ecef/1` and `ecef_to_lla/2`, verified against published reference
      values rather than only round-tripped
- [ ] `BB.Geo.LocalFrame` supporting both `:enu` and `:ned`, with no default
- [ ] `to_local/2` and `from_local/2` round-tripping to sub-millimetre
- [ ] Haversine and ellipsoidal distance, initial and final bearing
- [ ] All arithmetic in `:f64`, asserted by test
- [ ] `BB.Geo.Message.NavSatFix` and `BB.Geo.Message.Satellites`
- [ ] Shared `fix_type` / `carrier_solution` / `constellation` vocabulary
- [ ] `bb_ublox` migrated to depend on `bb_geo` and its local payloads deleted
- [ ] `usage-rules.md`, per Proposal 0007

### Should Have

- [ ] `BB.Geo.destination/3` — project a point along a bearing
- [ ] UTM, and possibly MGRS
- [ ] Geoid separation helpers, so height above mean sea level is derivable
- [ ] Test vectors from a published dataset (e.g. Karney's `GeodTest`) rather
      than only hand-computed cases

### Won't Have

- [ ] Any `BB.GNSS` behaviour — see the Design rationale
- [ ] Vendor-specific configuration abstractions; those stay in drivers behind
      `BB.Bridge`
- [ ] Kinematic or joint changes — Proposal 0022
- [ ] Map tiles, geofencing, or route planning
- [ ] Coordinate transforms beyond WGS 84-family datums; no full PROJ

---

## Open Questions

1. **Should the messages live here or in `bb` core?** Arguments both ways. Core
   precedent is strong — `BB.Message.Sensor.Imu`, `LaserScan`, and `BatteryState`
   are all in core, and `bb_sensor_bmi323` doesn't depend on a `bb_imu` package to
   get `Imu`. Messages are vocabulary rather than capability, so a struct in core
   costs nothing unless something publishes it. Against: every GNSS driver takes
   the `bb_geo` dependency anyway, so putting them here keeps core leaner.

   The case that tips it is a *surface* rather than a driver: `bb_liveview`
   wanting to render a fix on a map would need a geodesy dependency purely to
   pattern-match a message. This proposal assumes `bb_geo`; worth settling in
   review.

2. **Does a fix-to-floating-joint estimator belong here?** Proposal 0022 makes a
   floating joint real but leaves it unpopulated. A `BB.Estimator` fusing
   `NavSatFix` with AHRS orientation into a pose is the obvious join between the
   two proposals. It may want its own package, since it depends on both and on
   `bb_estimator_ahrs`.

3. **Which local-frame origin policy?** First-fix, configured constant, and
   moving-origin are all reasonable. The example above uses first-fix, but a
   robot that reboots mid-mission wants a stable configured origin. Possibly a
   caller concern rather than a library one.

4. **`altitude` naming.** `LLA.altitude` meaning height above the ellipsoid is
   correct but invites misreading, since colloquial "altitude" means above sea
   level. `height` — which is what `bb_ublox`'s `NavSatFix` calls it — may be
   clearer despite being less standard for the acronym.

5. **Should `bb_geo` be usable without `bb`?** The conversions are independent of
   the framework; only the messages need `BB.Message`. Splitting a dependency-free
   `geo` library from `bb_geo` is possible but is speculative generality unless
   someone actually wants it.

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
