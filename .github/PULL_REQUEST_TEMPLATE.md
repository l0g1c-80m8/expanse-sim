<!--
Thanks for sending a PR! Replace the placeholders below with your own
content. Sections that don't apply are fine to delete.
-->

## Summary

<!-- 1–3 bullets describing what changed and why. -->
-
-

## Type of change

<!-- check all that apply -->
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Refactor / cleanup (no functional change)
- [ ] Documentation
- [ ] CI / tooling

## Affected components

- [ ] `sim_core` (physics / ECS / ephemeris / autopilot / sensors)
- [ ] `sim_server` (axum / WebSocket / IPC bridges)
- [ ] `web_dashboard` (Next.js / Three.js UI)
- [ ] CI / workflows / community files

## Test plan

<!-- How did you verify this? Reproducible steps, commands, screenshots, video. -->
- [ ] `cargo test --workspace --no-default-features --features sim_core/thermodynamics`
- [ ] Dashboard typecheck: `cd web_dashboard && npx tsc --noEmit`
- [ ] Dashboard production build: `cd web_dashboard && npm run build`
- [ ] End-to-end smoke (`test: E2E websocket smoke test` / `test: E2E thrust controller` / `test: E2E autopilot` if relevant)
- [ ] Manual UI verification (describe what you clicked / what you saw)

## Determinism / physics correctness

<!-- For any change touching sim_core: -->
- [ ] No change to `dt` semantics or schedule ordering
- [ ] Adds tests for any new physics path (conservation, body-frame transforms, etc.)
- [ ] Documents any non-obvious numerical thresholds

## Screenshots / recordings

<!-- For UI / visualization changes: drop a before/after gif or PNG. -->

## Related

<!-- Closes #123, related to #456 -->
