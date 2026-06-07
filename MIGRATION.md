# P5 Migration Checklist

This file tracks the migration as a live checklist rather than a planning log.
The earlier planning document became stale once the protocol cutover and bench
validation work landed.

## Locked Decisions

- Strict P5 cutover only. No backward-compatibility layer.
- Default telemetry remains `20 ms`, but stays runtime-configurable through
  `telemetry_interval`.
- Fault handling stays minimal and protocol-safe.
- Each ESP32 board permanently manages one dial only.
- Long soak and continuous-use robustness validation is deferred until an
  overnight multi-dial bench session is available.

## Firmware And Protocol

- [x] Bump firmware version to `0.3.0`.
- [x] Store and expose one persistent `dial_id` per board.
- [x] Require strict `C,<seq>,<target>,<min>,<max>` frames.
- [x] Reject invalid `C` frames where `min > max`.
- [x] Implement `R,<seq>,<current_pos>` with logical rebasing.
- [x] Switch `S` parameters to unsuffixed single-dial names.
- [x] Extend telemetry to `T,<dial_id>,<seq>,<ang>,<spd>,<tor>,<foc_rate>,<status_bits>`.
- [x] Add `status_bits` packing for enabled features, OOB state, and faults.
- [x] Remove application-facing `M1` and second-motor assumptions.
- [x] Replace serial buffer overflow printouts with telemetry-visible fault reporting.

## Host Tools And Repo Layout

- [x] Update [PROTOCOL.md](PROTOCOL.md) to describe the P5 wire contract.
- [x] Update discovery tooling to use `dial_id` terminology.
- [x] Move calibration into [tools/motor_id_calibration.py](tools/motor_id_calibration.py).
- [x] Add [tools/deploy_upload.py](tools/deploy_upload.py) for batch flashing of matching CH340 devices.
- [x] Add [tools/deploy_set_id.py](tools/deploy_set_id.py) for interactive dial-ID assignment by detected movement.
- [x] Use shared VID/PID-based discovery helpers for validation and deployment tooling.

## Automated Validation

- [x] Add upload-and-smoke validation.
- [x] Add protocol smoke validation.
- [x] Add `R` / Set Current Position validation.
- [x] Add repeated rebase validation.
- [x] Add streaming stability validation.
- [x] Add RTT validation for `C`-echoed telemetry.
- [x] Add RTT validation for `E` echo responses.
- [x] Add sine-tracking validation.
- [x] Add bounds-specific validation with status-bit and torque checks.
- [x] Add markdown / CSV result export for validation scripts.
- [x] Auto-discover the bench device by `dial_id = 0` when `--port` is omitted.

## Manual Acceptance Support

- [x] Add [validation/manual_accept_no_yank.py](validation/manual_accept_no_yank.py).
- [x] Add [validation/manual_accept_bounds_feel.py](validation/manual_accept_bounds_feel.py).
- [ ] Run the manual no-yank acceptance script on hardware and record the result.
- [ ] Run the manual bounds-feel acceptance script on hardware and record the result.

## Deferred Overnight Robustness Work

- [ ] Add an overnight multi-dial soak suite with slow, fast, and sine-like motion patterns.
- [ ] Measure tracking error drift over long runs.
- [ ] Measure FOC-rate / control-rate degradation over long runs.
- [ ] Measure responsiveness degradation, stalls, or parser faults over long runs.
- [ ] Run the overnight soak suite with multiple dials and record results.
- [ ] Document continuous-use failure modes, fixes applied, and remaining limits.
- [ ] Rerun post-fix regression after the overnight robustness pass.

## Notes

- The new automated bounds coverage is [validation/test_bounds_behavior.py](validation/test_bounds_behavior.py).
- The manual acceptance paths are intentionally separate from automated scripts because they depend on felt behavior, not only serial observables.
- Overnight soak work is intentionally deferred until the multi-dial bench session is ready.