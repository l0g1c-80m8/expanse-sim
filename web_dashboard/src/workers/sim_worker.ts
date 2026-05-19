/// <reference lib="webworker" />
/**
 * Web Worker that owns a SimWasm instance and runs the simulator loop
 * entirely on the client. Mirrors the responsibilities of `sim_runner.rs`
 * in the Rust server:
 *   - drains a queue of ControlCommand messages between ticks
 *   - tracks wall-clock vs sim-clock budget for time-warp
 *   - emits a TelemetryFrame on every Nth tick
 *
 * Messages from main thread → worker:
 *   { type: 'init', basePath: string }
 *   { type: 'start' }
 *   { type: 'stop' }
 *   { type: 'command', command: ControlCommand }
 *   { type: 'setStride', stride: number }
 *
 * Messages worker → main thread:
 *   { type: 'ready' }
 *   { type: 'frame', frame: TelemetryFrame }
 *   { type: 'error', message: string }
 */

import type { ControlCommand, TelemetryFrame } from '@/lib/telemetry';

// `SimWasm` is imported dynamically inside `init` so the worker can resolve
// the WASM URL relative to the deploy base path.
type SimWasmInstance = {
  tick(): void;
  tickN(n: number): void;
  snapshotJson(tick: bigint, effectiveWarp: number): string;
  applyCommandJson(cmd: string): void;
  simTime(): number;
  dt(): number;
  warp(): number;
  paused(): boolean;
};

let sim: SimWasmInstance | null = null;
let running = false;
let stride = 2;
let tickCount = BigInt(0);
let pendingCommands: ControlCommand[] = [];

// Effective-warp tracking — same one-second rolling window the Rust runner uses.
let lastWarpSampleMs = 0;
let simSecondsInWindow = 0;
let effectiveWarp = 0;

// Tick budget — wall_dt × warp accumulates here; each schedule retires up to
// MAX_TICKS_PER_FRAME ticks (hard cap) OR up to STEP_BUDGET_MS of wall time
// (soft cap, whichever fires first) so the worker stays responsive to
// commands queued by the UI.
let budget = 0;
let lastWallMs = 0;
// Hard tick cap per batch. A modern laptop runs ≈3 k ticks/s inside the
// WASM sim, so 5 k caps the worst-case batch around ~1.5 s. We rely on
// STEP_BUDGET_MS to break out earlier on slower hardware.
const MAX_TICKS_PER_FRAME = 5_000;
// Soft wall-time cap — once a batch has consumed this much CPU it yields
// and lets queued commands (mode toggle, set target, …) apply on the next
// iteration. 80 ms balances responsiveness (~12 batches/s) against the
// per-batch setTimeout overhead (4 ms minimum in workers).
const STEP_BUDGET_MS = 80;
// Minimum interval between worker iterations when nothing's queued — we lean
// on requestAnimationFrame-ish 16 ms cadence for telemetry emit smoothness.
const IDLE_DELAY_MS = 16;

type WorkerOutMessage =
  | { type: 'ready' }
  | { type: 'frame'; frame: TelemetryFrame }
  | { type: 'error'; message: string };

function post(msg: WorkerOutMessage) {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  (self as any).postMessage(msg);
}

async function init(basePath: string) {
  try {
    // `import()` URL must be relative to the worker's own file URL because
    // webworkers don't honour module resolution paths from the main bundle.
    const wasmJsUrl = `${basePath.replace(/\/$/, '')}/wasm/sim_wasm.js`;
    const wasmBinUrl = `${basePath.replace(/\/$/, '')}/wasm/sim_wasm_bg.wasm`;
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const mod: any = await import(/* webpackIgnore: true */ /* @vite-ignore */ wasmJsUrl);
    await mod.default(wasmBinUrl);
    // 0.5 s integration step — matches sim_wasm::SimWasm::new_default and
    // keeps high-warp transits feeling alive. Visualization-side smoothing
    // (camera follow + drei <Line>) hides the step size at low warp.
    sim = new mod.SimWasm(0.5, 60.0, true) as SimWasmInstance;
    post({ type: 'ready' });
  } catch (err) {
    post({
      type: 'error',
      message: `WASM init failed: ${err instanceof Error ? err.message : String(err)}`,
    });
  }
}

function drainCommands() {
  if (!sim) return;
  while (pendingCommands.length > 0) {
    const cmd = pendingCommands.shift()!;
    try {
      sim.applyCommandJson(JSON.stringify(cmd));
    } catch (err) {
      // Bad command JSON — log and keep going. The transport contract is
      // "best effort"; a malformed command shouldn't tear down the worker.
      post({
        type: 'error',
        message: `apply_command failed: ${err instanceof Error ? err.message : String(err)}`,
      });
    }
  }
}

function emitFrame() {
  if (!sim) return;
  try {
    const json = sim.snapshotJson(tickCount, effectiveWarp);
    const frame: TelemetryFrame = JSON.parse(json);
    post({ type: 'frame', frame });
  } catch (err) {
    post({
      type: 'error',
      message: `snapshot failed: ${err instanceof Error ? err.message : String(err)}`,
    });
  }
}

function step() {
  if (!sim || !running) return;
  const now = performance.now();
  const wallDt = lastWallMs === 0 ? 0 : (now - lastWallMs) / 1000;
  lastWallMs = now;

  drainCommands();

  const warp = sim.warp();
  const dt = sim.dt();
  const paused = sim.paused();
  if (!paused) {
    budget += wallDt * warp;
  }

  let ticksToRun = Math.floor(budget / dt);
  if (ticksToRun > MAX_TICKS_PER_FRAME) ticksToRun = MAX_TICKS_PER_FRAME;

  const ZERO = BigInt(0);
  const batchStart = performance.now();
  // Check the wall-time guard every CHECK_EVERY ticks — sampling per tick
  // adds branch overhead, sampling once per batch defeats the purpose.
  const CHECK_EVERY = 64;
  let ticksDone = 0;
  for (let i = 0; i < ticksToRun; i++) {
    sim.tick();
    tickCount += BigInt(1);
    simSecondsInWindow += dt;
    ticksDone++;
    if (tickCount % BigInt(stride) === ZERO) {
      emitFrame();
    }
    if (i > 0 && i % CHECK_EVERY === 0 && performance.now() - batchStart > STEP_BUDGET_MS) {
      break;
    }
  }
  budget -= ticksDone * dt;
  if (budget > dt * MAX_TICKS_PER_FRAME) budget = dt * MAX_TICKS_PER_FRAME;

  // Roll the effective-warp window once per wall-second.
  if (now - lastWarpSampleMs >= 1000) {
    const elapsedS = (now - lastWarpSampleMs) / 1000;
    effectiveWarp = simSecondsInWindow / elapsedS;
    simSecondsInWindow = 0;
    lastWarpSampleMs = now;
  }

  // If we ran no ticks (paused or just below stride), still emit a frame
  // every so often so the UI stays current.
  if (ticksDone === 0) {
    emitFrame();
  }

  // Reschedule. yield via setTimeout(0) when there's pending budget, else
  // sleep IDLE_DELAY_MS so we don't burn CPU when paused.
  const delay = paused || budget < dt ? IDLE_DELAY_MS : 0;
  setTimeout(step, delay);
}

self.addEventListener('message', (evt) => {
  const data = evt.data as
    | { type: 'init'; basePath: string }
    | { type: 'start' }
    | { type: 'stop' }
    | { type: 'command'; command: ControlCommand }
    | { type: 'setStride'; stride: number };

  switch (data.type) {
    case 'init':
      void init(data.basePath);
      break;
    case 'start':
      if (running) break;
      running = true;
      lastWallMs = performance.now();
      lastWarpSampleMs = lastWallMs;
      step();
      break;
    case 'stop':
      running = false;
      break;
    case 'command':
      pendingCommands.push(data.command);
      break;
    case 'setStride':
      stride = Math.max(1, Math.floor(data.stride));
      break;
  }
});
