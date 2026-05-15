# expanse-sim

Deterministic, data-driven, 6-DOF interplanetary simulator built as a backend
for ROS 2 autonomy stacks. The simulator never integrates planetary physics —
it queries an ephemeris (NAIF SPICE when present, deterministic Keplerian
propagator otherwise). It supports two propulsion plants — conventional
chemical/ion and Expanse-style brachistochrone (high-thrust, high-Isp,
time-variant mass/inertia) — and a web dashboard for visualisation and
manual control.

## Workspace layout

```
sim_core/        Rust library — ECS, RK4 dynamics, propulsion, ephemeris, IPC
sim_server/      Rust binary  — axum HTTP/WebSocket bridge that drives sim_core
web_dashboard/   Next.js 16 + React 19 + Three.js bird's-eye dashboard
.vscode/         tasks.json + launch.json for one-click run/debug/test
```

## Quick start

```bash
# Run the core test suite (33 tests across unit + integration)
cargo test -p sim_core --no-default-features --features thermodynamics

# Run the simulation server (port 8080, time-warp 60×, telemetry every 1 s)
cargo run --release -p sim_server -- \
    --bind 127.0.0.1:8080 \
    --dt 0.05 \
    --warp 60 \
    --telemetry-stride 20

# In another terminal — start the dashboard
cd web_dashboard && npm install && npm run dev
# open http://localhost:3000
```

The dashboard's WebSocket URL defaults to `ws://127.0.0.1:8080/ws` and can be
overridden in the settings overlay (persisted to localStorage).

## Architecture

```
┌────────────── sim_server (tokio) ──────────────┐
│                                                │
│   ┌──── ExpanseSim (bevy_ecs World) ────────┐ │
│   │  ephemeris_refresh → propulsion         │ │
│   │     → dynamics (RK4) → lockstep         │ │
│   │              ↘ thermal (optional)       │ │
│   └─────────────────────────────────────────┘ │
│       ▲                       │                │
│  mpsc │ ControlCommand        │ TelemetryFrame │
│       │                       ▼                │
│   /ws WebSocket  ────► broadcast channel       │
└────────┬─────────────────┬─────────────────────┘
         │                 │
   Dashboard           ROS 2 / autonomy
   (React/Three)       (planned: ZeroMQ REQ/REP
                        lockstep bridge in
                        sim_core::ipc, feature-
                        gated as `zmq_bridge`)
```

### Determinism

* `SimTime.dt` is constant. Time-warp only changes how many ticks the driver
  retires per wall-second; the integration step never changes.
* The schedule is a deterministic Bevy `Schedule` with explicit ordering:
  `ephemeris_refresh → propulsion → dynamics → lockstep`.
* RK4 for translational state, Euler-equation step + quaternion exp-map for
  rotation. Free-rotation quaternion norm holds to machine precision over
  10⁵ ticks (covered by tests).

### Data-driven environment

The ephemeris cache stores a snapshot of every body's state, refreshed each
tick by either:

* **Keplerian propagator (default)** — closed-form Kepler solve, J2000
  ecliptic elements for Sun + 8 planets. Builds and tests with zero external
  dependencies.
* **NAIF SPICE (`--features spice`)** — wraps `spice::spkez` for
  arc-second-accurate states. Needs CSPICE installed and `CSPICE_DIR` set.

`EphemerisCache::gravity_at(position, t)` sums Newtonian gravity from every
body in the cache and is what `dynamics_system` consumes during RK4
substeps.

### Propulsion

Both drive types live in the same `PropulsionDrive` component, differentiated
by a `PropulsionType` enum. The propulsion system clamps the commanded thrust
to `max_thrust`, burns propellant per Tsiolkovsky, and (for Brachistochrone)
rescales the inertia tensor as mass drops. The actual force application is
done by `dynamics_system` so RK4 stays authoritative for f = ma. Body-frame
thrust is rotated through the rigid body's attitude before integration.

### Feature flags

| flag                 | what it does                                              |
|----------------------|-----------------------------------------------------------|
| `thermodynamics`     | enables `Thermodynamics` component + Stefan-Boltzmann sink |
| `synthetic_sensors`  | reserved for Iceoryx2 zero-copy sensor publishing          |
| `spice`              | swaps the Keplerian propagator for NAIF SPICE              |
| `zmq_bridge`         | lockstep ZeroMQ REP socket for ROS 2 autonomy nodes        |
| `iceoryx`            | shared-memory transport for high-bandwidth sensor frames   |
| `full`               | all of the above                                           |

## Testing

```bash
cargo test -p sim_core                                                # 26 unit + 7 integration
cargo test -p sim_core --features thermodynamics                      # +2 thermal
cargo test -p sim_server --test ws_smoke -- --ignored                  # E2E WebSocket
```

The integration tests cover:

* clock determinism under warp,
* uniform-motion sanity (RK4 with zero force),
* analytic kinematics under constant force,
* circular-orbit energy conservation,
* quaternion-norm preservation under free rotation,
* Tsiolkovsky Δv,
* Keplerian planet periods and orbital bands,
* end-to-end orbital drift (≤ 0.5 % over 1 sim-day).

The E2E test launches the binary, opens a WebSocket, asserts that telemetry
arrives, that `set_warp` is reflected in subsequent frames, and that
`set_thrust` actually burns propellant.

## VS Code integration

`.vscode/tasks.json` exposes one-click commands:

* **sim: cargo build (workspace)** — default build task
* **sim: cargo test (all features minus spice)** — default test task
* **sim: run server (debug)** / **(release)**
* **sim: full stack (server + dashboard)** — runs both in parallel
* **dashboard: dev server / build / tsc --noEmit**
* **sim: smoke test snapshot** — curl-based health check

`.vscode/launch.json` adds LLDB launch configs for the server binary and
for both unit and integration test runners, so breakpoints work directly
in the IDE.

## Debugging tips

* High-warp run looks "stuck"? Check `--telemetry-stride` — the dashboard
  drops frames silently when stride is small relative to warp. A stride of
  `warp / 10` is a sensible starting point.
* Spacecraft seems to teleport at high warp — the wall-clock budget cap in
  `sim_runner.rs` ticks at most 5000 sim steps per loop iteration to keep the
  executor responsive. Raise `loop_interval_us` or the cap if you need more.
* Propulsion goes silent — confirm `propellant_mass > 0`. The drive emits no
  force after the tank empties even if `thrust_command` is non-zero.
* Test failures around energy drift — RK4 is stable for the default `dt =
  0.01` at solar distances; if you change `dt` to something coarse, expect
  some tests to need adjusted tolerances.
