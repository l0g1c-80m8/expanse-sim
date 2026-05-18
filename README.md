# expanse-sim

> A deterministic, 6-DOF interplanetary spacecraft simulator with a
> ZEM/ZEV rendezvous autopilot, a synthetic sensor model, and a live
> solar-system web dashboard — a digital twin of the solar system built for
> navigation, localization, and autonomy R&D.

The simulator never integrates planetary motion itself — it queries an
ephemeris (NAIF SPICE when present, a deterministic Keplerian propagator
otherwise). It supports two propulsion plants: conventional chemical / ion
and Expanse-style brachistochrone (high-thrust, high-Isp, time-variant
mass and inertia). External autonomy stacks (ROS 2, MPC, EKF, etc.) drive
it through a WebSocket or a lockstep IPC bridge; the bundled web dashboard
shows what the simulator sees in real time.

## Table of contents

- [Highlights](#highlights)
- [Repository layout](#repository-layout)
- [Quick start](#quick-start)
- [Architecture](#architecture)
- [Determinism contract](#determinism-contract)
- [Physics & dynamics](#physics--dynamics)
- [Autopilot (ZEM/ZEV)](#autopilot-zemzev)
- [Synthetic sensor model](#synthetic-sensor-model)
- [Mission, source & target](#mission-source--target)
- [Web dashboard](#web-dashboard)
- [Feature flags](#feature-flags)
- [Testing](#testing)
- [VS Code integration](#vs-code-integration)
- [GitHub Pages deployment](#github-pages-deployment)
- [Debugging tips](#debugging-tips)
- [License](#license)

## Highlights

- **Real RK4** for translational state with quaternion-aware rotational
  integration. Free-rotation quaternion norm holds to machine precision
  over 10⁵ ticks (tested).
- **ZEM/ZEV rendezvous autopilot** (Battin §13 fixed-time guidance) that
  actually nulls both relative position *and* velocity at the target —
  ships arrive softly instead of flying past. Saturation-aware: the
  time-to-go is iteratively grown until the commanded acceleration fits
  the spacecraft's thrust budget.
- **Synthetic sensors** — range, range-rate, inertial bearings, IMU
  (gravity-free specific force + gyro), star-tracker attitude. Gaussian
  noise is **deterministic per `(sim_time, body_id)`** so EKF / nav
  replays see the same samples every run.
- **Data-driven environment** — ephemeris cache pre-loaded with Sun + 8
  planets; swap in NAIF SPICE BSP kernels via `--features spice` for
  arc-second-accurate states.
- **Time-warp without breaking determinism** — `SimTime.dt` is constant;
  warp only changes how many ticks the driver retires per wall-second.
  A PID/MPC tuned at 1× behaves identically at 100 000×.
- **Live web dashboard** — Three.js solar-system view with orbit traces,
  predicted trajectory forecast, body labels, velocity / thrust arrows,
  mission source–target picker, and a sensors / autopilot console.
- **Deployable to GitHub Pages** — static export with a one-shot GitHub
  Actions workflow.
- **53 unit + integration tests** passing on every push.

## Repository layout

```
sim_core/        Rust library — ECS, RK4 dynamics, propulsion, ephemeris,
                 autopilot, sensors, IPC bridges
sim_server/      Rust binary  — axum HTTP/WebSocket server that drives sim_core
                 in a tokio task with adaptive wall-clock pacing
web_dashboard/   Next.js 16 + React 19 + Three.js dashboard (static export)
.github/         CODEOWNERS, workflows (GitHub Pages deploy), issue & PR
                 templates
.vscode/         tasks.json + launch.json + settings.json for one-click
                 build / run / debug / test from the IDE
```

## Quick start

Prerequisites: **Rust ≥ 1.85** (edition 2024) and **Node.js ≥ 20**.

```bash
# Tests — 53 unit + integration, no external deps needed
cargo test --workspace --no-default-features --features sim_core/thermodynamics

# Run the simulation server (release recommended for high warp)
cargo run --release -p sim_server -- \
    --bind 127.0.0.1:8080 \
    --dt 0.05 \
    --warp 60 \
    --telemetry-stride 20

# In another terminal — install + start the dashboard
cd web_dashboard
npm install
npm run dev
# open http://localhost:3000
```

In VS Code: `Tasks: Run Task` → **`run: full stack (server + dashboard)`**.

The dashboard's WebSocket URL defaults to `ws://127.0.0.1:8080/ws` and can
be overridden through the settings overlay (persisted to `localStorage`).

## Architecture

```
┌────────────────── sim_server (tokio) ──────────────────┐
│                                                        │
│   ┌──── ExpanseSim (bevy_ecs World) ────────────────┐  │
│   │  ephemeris_refresh                              │  │
│   │      → autopilot (ZEM/ZEV)                       │  │
│   │      → propulsion                                │  │
│   │      → dynamics (RK4)                            │  │
│   │      → sensors                                   │  │
│   │      → thermal (optional)                        │  │
│   │      → lockstep (autonomy bridge)                │  │
│   └──────────────────────────────────────────────────┘  │
│       ▲                                  │              │
│  mpsc │ ControlCommand                   │ Telemetry    │
│       │                                  ▼              │
│   /ws WebSocket  ────► tokio::broadcast channel         │
└────────┬─────────────────────────────────────┬──────────┘
         │                                     │
   Web dashboard                       Autonomy stack
   (Next.js / Three.js)                (ROS 2 / MPC / EKF)
                                       via `zmq_bridge` feature
```

Threading model — one dedicated `spawn_blocking` task drives the
simulation; tokio handles HTTP/WS; a `tokio::sync::broadcast` channel
fan-outs telemetry to N web clients; commands flow back through an MPSC.
The driver yields adaptively (`thread::yield_now()` when there's budget
to retire, `thread::sleep` otherwise) so warp throughput isn't capped by
a fixed loop interval.

## Determinism contract

- **`SimTime.dt` is invariant.** Warp only changes the tick-retire rate.
- **Schedule order is explicit.** Every system declares `.after()` /
  `.before()` — implicit ordering would break replay.
- **Sensor noise is seeded** off `(sim_time, body_id)`, never wall clock.
- **No floating-point reductions** depend on threading; tests pin numeric
  bounds tightly enough to catch a drift regression.

ROS 2 controllers tuned at 1× warp behave identically at 100 000× — which
is the only reason time-warping is useful for autonomy work.

## Physics & dynamics

- **Translation** — classical fourth-order Runge-Kutta over a constant
  `dt`. The propulsion force is held steady across the substeps; with
  sub-millisecond ticks this is a tight approximation.
- **Rotation** — Euler's rigid-body equation `I ω̇ = τ − ω × (I ω)` with a
  midpoint step on ω and a quaternion exponential map on the attitude.
- **Gravity** — `EphemerisCache::gravity_at(position, t)` sums Newtonian
  gravity from every body in the cache. RK4 substeps see consistent body
  positions because the ephemeris is refreshed once per tick.
- **Body-frame thrust** is rotated through the rigid body's current
  attitude before integration. Body +x is the conventional drive axis.

## Autopilot (ZEM/ZEV)

The headline guidance feature. Given a mission target and a commanded
acceleration budget (in `g`), the autopilot drives the ship to arrival
with near-zero relative velocity.

```
a_cmd =  6 · ZEM / t_go²  −  2 · ZEV / t_go
ZEM   =  (r_target − r_ship) + (v_target − v_ship) · t_go
ZEV   =  v_target − v_ship
```

This is the closed-form optimal solution to the quadratic-cost fixed-time
rendezvous problem (Battin, *Introduction to the Mathematics and Methods
of Astrodynamics*, §13) — the same family of laws used in Apollo descent
guidance and AR&D phasing burns.

The naive form demands unbounded acceleration whenever `v_rel` is poorly
aligned with `r_rel`, so the autopilot **iteratively expands `t_go`**
until the commanded acceleration fits the budget. Tested against:

- a stationary target → soft arrival within 500 km and 5 m/s,
- a laterally-drifting target → soft arrival within 1000 km and 50 m/s,
- a real Earth → Mars transit @ 5 g (release build E2E) → 32 600 km, −29 m/s.

Phases reported on the wire: `Idle / Boost / Brake / Arrived / Hold`.

## Synthetic sensor model

Per-tick `SensorPack` includes:

- **Per-body range** + range-rate + inertial-frame bearing unit vector
- **IMU** — body-frame specific force (gravity-free, as a strapdown reports)
  and gyro angular rate
- **Star tracker** — attitude quaternion with isotropic pointing noise

All sigmas are configurable at runtime (`set_sensor_config` WebSocket
command). The dashboard exposes four presets: **Off**, **DSN-class**
(σᵣ ≈ 1 ppm), **Spacecraft** (σᵣ ≈ 10 ppm + 10 m floor), and **Stressed**
(σᵣ ≈ 0.1 %). Noise is generated with a deterministic xorshift PRNG seeded
from `(sim_time, body_id)` — re-running the same sim produces bit-for-bit
identical sensor streams.

This is what makes the simulator useful as a **digital twin for nav /
localization R&D**: an external EKF or particle filter can be fed the
noisy measurements and benchmarked against the ground truth that the
server is also broadcasting.

## Mission, source & target

A `Mission { source_body, target_body }` resource lives in the ECS world
and is exposed in every telemetry frame. The autonomy stack reads it to
decide where to plan a transit; the dashboard exposes a picker (with
order-of-magnitude Hohmann and 1 g brachistochrone transit estimates) and
a **"Stage at source"** button that respawns the spacecraft on the source
body's current heliocentric orbit. The default mission is **Earth → Mars**.

## Web dashboard

Live components (toggleable in the View panel):

- Solar-system map with bodies sized for legibility and labeled with live
  distances; per-body orbit traces in body-tinted colour.
- Spacecraft hull oriented along velocity, with cyan selection ring,
  thrust plume when burning, green velocity arrow, and orange thrust arrow.
- **Predicted trajectory** — client-side leapfrog forecast under Sun-only
  gravity, configurable horizon (1 day to 5 years).
- Source / target highlighted with coloured halos and connected by a
  dashed transit line.
- Configurable **camera focus** — free orbit / Sun / any planet / ship.

Side panels:

- **Telemetry** — sim-time, requested vs effective warp (flagged amber
  when the throughput ceiling is hit), connection status, spacecraft
  state.
- **Mission** — source / target dropdowns, Stage button, transit estimates.
- **Autopilot** — Engage toggle, accel slider (0.1 – 5 g), phase badge,
  range / closing rate / ETA.
- **Sensors** — radar readout for any body, IMU magnitudes, star-tracker
  quaternion, noise-preset picker.
- **View** — orbit / label / grid / stars / trajectory toggles, horizon
  picker, camera focus.

Keyboard shortcuts: `Space` pause / resume, `[` / `]` warp down / up,
`x` cut engine, `f` focus on ship, `r` reset.

## Feature flags

Compose with `cargo build -p sim_core --features "<a>,<b>"`:

| flag                | what it does                                              |
|---------------------|-----------------------------------------------------------|
| `thermodynamics`    | enables `Thermodynamics` component + Stefan-Boltzmann sink |
| `synthetic_sensors` | reserved for Iceoryx2 zero-copy high-bandwidth sensor publishing |
| `spice`             | swaps the Keplerian propagator for NAIF SPICE              |
| `zmq_bridge`        | lockstep ZeroMQ REP socket for ROS 2 autonomy nodes        |
| `iceoryx`           | shared-memory transport for high-bandwidth sensor frames   |
| `full`              | all of the above                                           |

Native-dependency features (`spice`, `zmq_bridge`, `iceoryx`) are opt-in
on purpose — the core library builds and tests cleanly on a bare machine.

## Testing

```bash
# 53 unit + integration tests, no external deps
cargo test --workspace --no-default-features --features sim_core/thermodynamics

# End-to-end WebSocket smoke tests (need a built binary; ignored by default)
cargo build --release -p sim_server
CARGO_BIN_EXE_sim_server=target/release/sim_server \
    cargo test -p sim_server -- --ignored --nocapture

# Dashboard
cd web_dashboard
npx tsc --noEmit
npm run build
```

Coverage spans:

- clock determinism under warp,
- RK4 conservation (uniform motion, constant force, circular orbit energy),
- quaternion-norm preservation under free rotation,
- Tsiolkovsky Δv and brachistochrone inertia rescaling,
- Keplerian planet periods and orbital bands,
- ephemeris position drift over 30 simulated days,
- autopilot arrival at static and moving targets,
- noise-free vs noisy sensor reproducibility and seeded determinism,
- end-to-end WebSocket telemetry + commands, autopilot Earth → Mars.

Run the **`everything: build, test, lint`** VS Code task to gate a PR.

## VS Code integration

`.vscode/tasks.json` exposes 32+ tasks grouped by **build / run / stop /
clean / test / lint / smoke / meta**. Highlights:

- **`run: full stack (server + dashboard)`** — both processes in parallel
- **`test: E2E websocket smoke test`** / **`test: E2E thrust controller`**
  / **`test: E2E autopilot`**
- **`dashboard: pages-style build`** — mirrors the GitHub Pages CI build
- **`everything: build, test, lint`** — pre-PR gate
- **`smoke: GET /health + /snapshot`** — curl-based liveness check

`.vscode/launch.json` adds LLDB launch configs for the server binary,
unit tests, integration tests (with a per-test name prompt for focused
debugging), and a Chrome / Firefox configuration for the dashboard.

## GitHub Pages deployment

The dashboard is built as a static export and published on every push to
`develop` that touches `web_dashboard/**`. The workflow lives at
[`.github/workflows/deploy-pages.yml`](.github/workflows/deploy-pages.yml).

One-time setup: **Repo Settings → Pages → Source = "GitHub Actions"**.

Live site: <https://l0g1c-80m8.github.io/expanse-sim/>.

To override the default WebSocket endpoint baked into the hosted bundle,
set a repository variable `NEXT_PUBLIC_DEFAULT_WS_URL` to your `wss://`
URL. Users can still change it live through the dashboard's settings
panel.

## Debugging tips

- **High-warp run looks "stuck"** — check the `warp eff` row in the
  Telemetry panel. If it's flagged amber, the requested warp exceeds the
  effective throughput ceiling. Lower the warp or increase `--dt`.
- **Spacecraft teleports / misbehaves at high warp** — debug builds can
  sustain only a few thousand `×` effective warp. Use the release server
  (`cargo run --release -p sim_server`) for the demo.
- **Propulsion goes silent** — confirm `propellant_mass > 0`. The drive
  emits no force after the tank empties even if `thrust_command` is set.
- **Autopilot oscillates** — usually means the target body resolves to a
  fast-orbiting synthetic body (period < transit time). Use the static
  position helper `EphemerisCache::set_state` in tests.
- **Dashboard shows "Disconnected"** — the WS URL in the settings panel
  doesn't match what the server is bound to. Default is
  `ws://127.0.0.1:8080/ws`.
- **Tests fail on energy drift after changing `dt`** — RK4 is stable for
  the default `dt = 0.01` at solar distances; coarsening it may need
  adjusted tolerances.

## License

[MIT](LICENSE) © 2026 Rutvik Patel.
