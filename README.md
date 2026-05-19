# expanse-sim

> A deterministic, 6-DOF interplanetary spacecraft simulator with a
> ZEM/ZEV rendezvous autopilot, an onboard EKF nav filter, a synthetic
> sensor model, and a live solar-system web dashboard — a digital twin
> of the solar system built for navigation, localization, and autonomy
> R&D, runnable entirely in the browser via WebAssembly.

The simulator never integrates planetary motion itself — it queries an
ephemeris (NAIF SPICE when present, a deterministic Keplerian propagator
otherwise). It supports two propulsion plants: conventional chemical /
ion and Expanse-style brachistochrone (high-thrust, high-Isp, time-variant
mass / inertia). External autonomy stacks (ROS 2, MPC, EKF, etc.) drive
it through a WebSocket or a lockstep IPC bridge; the bundled web
dashboard talks to the same wire format and can either connect to a
running Rust server or run the whole simulation in-browser via WASM.

## Table of contents

- [Highlights](#highlights)
- [Repository layout](#repository-layout)
- [Quick start](#quick-start)
- [Architecture](#architecture)
- [Determinism contract](#determinism-contract)
- [Physics & dynamics](#physics--dynamics)
- [Autopilot (ZEM/ZEV)](#autopilot-zemzev)
- [Onboard navigation (EKF)](#onboard-navigation-ekf)
- [Synthetic sensor model](#synthetic-sensor-model)
- [Mission, source & target](#mission-source--target)
- [Operating modes (Sandbox vs Mission)](#operating-modes-sandbox-vs-mission)
- [Web dashboard](#web-dashboard)
- [Transports: WASM in-browser vs WebSocket server](#transports-wasm-in-browser-vs-websocket-server)
- [Feature flags](#feature-flags)
- [Testing](#testing)
- [VS Code integration](#vs-code-integration)
- [GitHub Pages deployment](#github-pages-deployment)
- [Debugging tips](#debugging-tips)
- [License](#license)

## Highlights

- **Real RK4** translational integration with quaternion-aware rotation.
  Free-rotation quaternion norm holds to machine precision over 10⁵ ticks.
- **ZEM/ZEV rendezvous autopilot** (Battin §13 fixed-time guidance) that
  nulls both relative position and velocity at the target. Saturation
  aware: `t_go` is iteratively grown until the commanded acceleration
  fits the budget. Ship arrives **in target orbit**, not at body centre.
- **Onboard EKF navigation filter** — 6-state position+velocity estimator
  consuming the noisy sensor pack with measurement updates from each
  catalogued body. Position uncertainty visualised as a wireframe ball
  around a "ghost ship" alongside the ground-truth ship.
- **Synthetic sensors** — range, range-rate (with optional **light-time
  delay**), inertial bearings, IMU (gravity-free specific force + gyro),
  star-tracker attitude. Deterministic Gaussian noise seeded from
  `(sim_time, body_id)` so EKF replays see identical samples.
- **Data-driven environment** — ephemeris cache pre-loaded with Sun + 8
  planets + Luna + Phobos + Deimos. Swap in NAIF SPICE BSP kernels via
  `--features spice` for arc-second-accurate states.
- **Time-warp without breaking determinism** — `SimTime.dt` is constant;
  warp only changes how many ticks the driver retires per wall-second.
- **Two transports** — the dashboard can run the simulator
  **entirely in the browser (WASM)** or talk to a Rust **WebSocket
  server**. Identical JSON wire format; one toggle in the header.
- **Live web dashboard** — Three.js solar-system view with orbit traces,
  predicted-trajectory forecast, fading breadcrumb trail, body labels,
  velocity / thrust arrows, mission progress + Δv budget, EKF
  ghost-ship, click-to-focus on any body.
- **Solar Radiation Pressure** modelled as a small radial force when the
  spacecraft carries a `RadiationModel` component — physically correct,
  scales as `(AU/r)²`.
- **73 unit + integration tests** + 4 ignored E2E (WebSocket smoke).

## Repository layout

```
sim_core/        Rust library — ECS, RK4 dynamics, propulsion, ephemeris,
                 autopilot, sensors, EKF nav, wire protocol
sim_server/      Rust binary — axum HTTP/WebSocket server, thin shim over
                 sim_core::protocol
sim_wasm/        WASM wrapper crate — wasm-bindgen module that runs the same
                 sim in a browser Web Worker
web_dashboard/   Next.js 16 + React 19 + Three.js dashboard (static export)
.github/         CODEOWNERS, workflows (GitHub Pages deploy with WASM build),
                 issue & PR templates
.vscode/         tasks.json + launch.json + settings.json for one-click
                 build / run / debug / test from the IDE
```

`sim_core` is a single library that everyone — Rust server, WASM
front-end, integration tests — uses. The crate root keeps a small
ergonomic surface (`SimTime`, `SimClock`, `RigidBody`, `ExpanseSim`,
`SimConfig`); for everything else, `use sim_core::prelude::*;` or reach
into the module path directly (`sim_core::autopilot::Autopilot`).

## Quick start

Prerequisites: **Rust ≥ 1.85** (edition 2024) and **Node.js ≥ 20**.

```bash
# Tests — 73 unit + integration, no external deps needed
cargo test --workspace --no-default-features --features sim_core/thermodynamics

# Either run the dashboard standalone (uses in-browser WASM sim) …
cd web_dashboard
# one-time: build the WASM module the dashboard loads
wasm-pack build ../sim_wasm --release --target web --out-dir ../web_dashboard/public/wasm
npm install
npm run dev
# open http://localhost:3000 — the simulator runs in a Web Worker

# … or run the WebSocket server backend and flip the dashboard's
# transport toggle to "Server" in the header.
cargo run --release -p sim_server -- \
    --bind 127.0.0.1:8080 \
    --dt 0.05 \
    --warp 60 \
    --telemetry-stride 20
```

In VS Code: `Tasks: Run Task` → **`run: full stack (server + dashboard)`**
for the WebSocket path, or **`build: sim_wasm`** + **`run: dashboard (dev)`**
for the WASM-only path.

## Architecture

```
            ┌──────── ExpanseSim (bevy_ecs World + Schedule) ────────┐
            │  ephemeris_refresh → autopilot → propulsion →          │
            │     dynamics (RK4) → sensors → nav (EKF) →             │
            │     ↘ thermal (optional)  → lockstep                   │
            └────────────────────────────────────────────────────────┘
                              ▲             │
              ControlCommand  │             │ TelemetryFrame
                              │             ▼
                ┌─────────────┴─────────────────────────────┐
                │  sim_core::protocol  (single source of    │
                │  truth: types + snapshot + apply_command) │
                └─────────────┬─────────────────────────────┘
                              │
              ┌───────────────┼───────────────────┐
              ▼               ▼                   ▼
      sim_server          sim_wasm           autonomy stack
      (axum + tokio)      (wasm-bindgen,     (ROS 2 + planned
                          Web Worker)         ZMQ bridge in
                                              sim_core::ipc)
              ▼               ▼
       WebSocket client   in-browser
              └──────┬────────┘
                     ▼
            web_dashboard (Next.js)
```

`sim_core::protocol` is the only module that knows about the wire
format. Submodules:

- `types.rs`    — `TelemetryFrame`, `ControlCommand`, snapshot structs
- `snapshot.rs` — read the ECS world into a `TelemetryFrame`
- `command.rs`  — `apply_command` + `stage_at_source`
- `setup.rs`    — `build_default_sim` + `spawn_default_spacecraft`

### Determinism

- `SimTime.dt` is constant. Warp only changes the tick-retire rate.
- The schedule is a deterministic Bevy `Schedule` with explicit ordering.
- Sensor noise is seeded off `(sim_time, body_id)`, never wall clock.
- The WASM build runs the same schedule as the native server bit-for-bit.

## Physics & dynamics

- **Translation** — classical 4th-order Runge-Kutta over a constant `dt`.
- **Rotation** — Euler's rigid-body equation with quaternion exponential
  map for attitude.
- **Gravity** — N-body sum from every catalogued body. Interior gravity
  uses the uniform-density shell theorem (linear in r below the surface)
  so the integrator never hits the 1/r² singularity.
- **Solar Radiation Pressure** — radial outward force scaling as
  `(AU/r)²`, applied when the spacecraft has a `RadiationModel`.
- **Body-frame thrust** is rotated through the rigid body's attitude
  before integration.

## Autopilot (ZEM/ZEV)

```
a_cmd =  6 · ZEM / t_go²  −  2 · ZEV / t_go
ZEM   =  (r_target − r_ship) + (v_target − v_ship) · t_go
ZEV   =  v_target − v_ship
```

The closed-form optimal fixed-time rendezvous law (Battin §13). The
naive form demands unbounded acceleration when `v_rel` is poorly aligned
with `r_rel`, so the controller iteratively grows `t_go` until the
commanded acceleration fits the spacecraft's `accel_g` budget.

**Target = parking orbit, not body centre.** When the target body has
non-zero μ, the autopilot's effective target is a circular parking
orbit around it. The ship arrives in orbit instead of being driven into
the body's centre.

Phases reported on the wire: `Idle / Boost / Brake / Arrived / Hold`.

## Onboard navigation (EKF)

`sim_core::nav` runs a 6-state position+velocity EKF that consumes the
synthetic sensor pack and produces an estimated state plus 6×6
covariance. Process model: Sun-only gravity (cruise-style). Observations:
per-body range with sequential updates. Initialises from ground truth on
engage with operator-set uncertainty; thereafter operates purely on
sensor data.

The dashboard renders the estimate as a violet "ghost ship" with a
wireframe 1-σ ball around it, plus a violet error line drawn back to
the ground-truth ship — instant visual diagnostic for nav filter
performance.

## Synthetic sensor model

Per-tick `SensorPack` contains:

- **Per-body range** + range-rate + inertial-frame bearing unit vector,
  with optional **one-way light-time delay** (`r/c` correction).
- **IMU** — body-frame specific force (gravity-free, as strapdown reports)
  and gyro rate.
- **Star tracker** — attitude quaternion with isotropic pointing noise.

Configurable noise sigmas. Four dashboard presets: **Off / DSN-class /
Spacecraft / Stressed**. Noise is deterministic per `(sim_time, body_id)`.

## Mission, source & target

A `Mission { source_body, target_body }` resource lives in the ECS world
and is broadcast in every telemetry frame. The dashboard exposes
source / target dropdowns, a transit-warp picker, and a one-shot
**Plan & Run** button that:

1. Sets the mission endpoints
2. Switches to Mission mode
3. Stages the ship in a circular orbit around the source body
4. Engages the autopilot
5. Bumps warp so the transit finishes in seconds of wall time

## Operating modes (Sandbox vs Mission)

- **Sandbox** — the ship coasts under gravity (and SRP); autopilot is
  silent. Useful for orbital-mechanics study and nav-filter benchmarking
  without a thrust confound.
- **Mission** — autopilot has authority; operator thrust commands are
  ignored so the planned trajectory plays out cleanly.

Header toggle (chip pair) flips between them. Switching back to Sandbox
cuts any in-flight burn.

## Web dashboard

- **Three.js solar-system map** with bodies sized for legibility,
  ecliptic grid, configurable star background. Moons (Luna, Phobos,
  Deimos) hide their labels at solar-system zoom to avoid colliding
  with their planet's label; click any body sphere to focus the camera
  on it.
- **Spacecraft hull** oriented along velocity, cyan selection ring,
  thrust plume when burning, green velocity arrow, orange thrust arrow.
- **Trajectory visuals** — predicted forecast (cyan dashed), past
  breadcrumb trail (warm orange), mission transit line (pink dashed),
  body orbit rings, all drawn with drei's `<Line>` so width actually
  shows up.
- **Ghost ship + 1-σ ball** when the nav filter is engaged.
- **Side panels** — Telemetry, Mission (with source / target picker,
  Plan & Run, transit estimates), Autopilot (engage, accel slider,
  phase badge, range / closing / ETA, progress bar, Δv budget),
  Nav (engage filter, uncertainty, estimator error vs ground truth),
  Sensors (live radar / IMU / star tracker, noise presets, light-time
  toggle), View (orbit / label / grid / star toggles, trajectory
  horizon, camera focus).
- **Keyboard shortcuts** — `Space` pause, `[` / `]` warp down / up,
  `x` cut engine, `f` focus on ship, `r` reset.

## Transports: WASM in-browser vs WebSocket server

The dashboard ships **both** transports and picks via a header toggle
(persisted in `localStorage`).

| Mode       | Use it for                                                                            |
|------------|---------------------------------------------------------------------------------------|
| **WASM**   | Hosted demo / no backend needed / works offline / scales to N users for free          |
| **Server** | High-fidelity local dev with full Rust performance, or a hosted backend with `wss://` |

The wire format is identical — same `TelemetryFrame`, same
`ControlCommand`, same `sim_core::protocol::{snapshot, apply_command}`.
The WASM module is built via:

```bash
wasm-pack build sim_wasm --release --target web \
    --out-dir ../web_dashboard/public/wasm
```

and ends up as a ~720 KB optimised `.wasm` plus a 12 KB JS shim that
Next.js bundles into the static export.

## Feature flags

Compose with `cargo build -p sim_core --features "<a>,<b>"`:

| flag                | what it does                                               |
|---------------------|------------------------------------------------------------|
| `thermodynamics`    | enables `Thermodynamics` component + Stefan-Boltzmann sink |
| `synthetic_sensors` | reserved for Iceoryx2 zero-copy sensor publishing          |
| `spice`             | swaps the Keplerian propagator for NAIF SPICE              |
| `zmq_bridge`        | lockstep ZeroMQ REP socket for ROS 2 autonomy nodes        |
| `iceoryx`           | shared-memory transport for high-bandwidth sensor frames   |
| `full`              | all of the above                                           |

Native-dependency features (`spice`, `zmq_bridge`, `iceoryx`) are opt-in
on purpose — the core library builds and tests cleanly on a bare machine
and compiles to WebAssembly without modification.

## Testing

```bash
# 73 unit + integration tests, no external deps
cargo test --workspace --no-default-features --features sim_core/thermodynamics

# End-to-end WebSocket smoke tests (need a built binary; ignored by default)
cargo build --release -p sim_server
CARGO_BIN_EXE_sim_server=target/release/sim_server \
    cargo test -p sim_server -- --ignored --test-threads=1 --nocapture

# Dashboard
cd web_dashboard
npx tsc --noEmit
npm run build
```

Coverage spans: clock determinism under warp, RK4 conservation, autopilot
arrival at static and moving targets, parking-orbit staging, light-time
delay correction, EKF init + measurement update, body-frame transforms,
SRP scaling laws, ephemeris periods + lunar return-to-start, and
end-to-end WebSocket telemetry + commands.

## VS Code integration

`.vscode/tasks.json` exposes 33+ tasks grouped by **build / run / stop /
clean / test / lint / smoke / meta**, including
**`build: sim_wasm (release, into dashboard public/)`** for the
WASM artifact.

## GitHub Pages deployment

`.github/workflows/deploy-pages.yml` builds the WASM module + the
Next.js static export on every push to `develop`/`main` and publishes
to GitHub Pages. One-time setup: **Settings → Pages → Source = "GitHub
Actions"**.

Live: <https://l0g1c-80m8.github.io/expanse-sim/>

## Debugging tips

- **Plan & Run barely moves the ship** — bump the transit-warp picker in
  the mission panel (default 10k×). The server respects it via the
  `warp` field on `start_mission`.
- **High-warp run flagged amber** — the `warp eff` row in the telemetry
  panel is < 90% of the requested warp. The throughput cap is biting;
  raise `--dt` or lower the requested warp.
- **Spacecraft staged "inside" Earth visually** — that's a scale
  artefact, not a bug. Planet visual radii are non-physical for
  legibility; the ship is in a real parking orbit (1000+ km altitude).
  Click Earth's sphere to focus and zoom in.
- **Autopilot oscillating** — usually means the target is a fast-orbiting
  synthetic body. Real bodies (μ > 0) drop into a parking-orbit target
  automatically.
- **WASM mode shows "Loading…" forever** — check the browser console.
  Most likely the WASM artifact isn't in `web_dashboard/public/wasm/`
  (run `wasm-pack build sim_wasm --release --target web --out-dir
  ../web_dashboard/public/wasm`).

## License

[MIT](LICENSE) © 2026 Rutvik Patel.
