# P5 Firmware Migration Plan

This document records the current repository status against the P5 firmware
spec in `HAPTIC_FIRMWARE_P5.md` and proposes a migration plan before any
implementation changes are made.

## Locked Decisions

These implementation rules are now fixed for the first P5 migration pass.

- P5 is an immediate cutover. No backward compatibility layer is needed.
- Default telemetry stays at `20 ms` for now, but it must remain easy to
  change and continue to support runtime adjustment via the existing
  `telemetry_interval` parameter.
- Fault handling should stay minimal in the first pass, favoring simpler code
  and higher runtime performance.
- All `M1` references and second-motor assumptions should be removed as part of
  the migration, because each ESP32 board will permanently manage one dial
  motor only.

## Executive Status

The repository is partially migrated to the P5 single-dial model.

What is already true:

- The active firmware runtime in `src/main.cpp` controls one live dial per
  board.
- The firmware already exposes the core ASCII commands `C`, `S`, `I`, `V`, and
  `E`.
- Telemetry already auto-starts on port open.
- Persistent board identity already exists in NVS.
- Host-side discovery and calibration scripts were already simplified from a
  multi-motor-per-board model to a single active telemetry stream per board.

What is not yet true:

- The firmware still identifies itself as protocol/version `0.2.0`.
- The wire contract still uses old `_0` parameter names and `motor_id_0`
  naming instead of the P5 `dial_id` contract.
- The critical new `R` / Set Current Position command is missing.
- Telemetry is still missing the P5 `status_bits` field.
- Bounds validation and fault handling are still minimal.
- Documentation, scripts, and local tooling are inconsistent with the current
  hardware setup.

## Repository Scan Summary

### Already aligned or mostly aligned

1. Single active motor path in firmware
   - `src/main.cpp` creates one `Dial` instance and only aligns/runs `M0`.
   - Telemetry sends one angle, one speed, one torque, and one FOC rate.

2. Single-board identity persisted in flash
   - `src/main.cpp` stores one ID in NVS and responds to `I,<seq>` queries.

3. Host utilities already assume one logical actuator per board
   - `device_discovery.py` now probes one identity value.
   - `motor_id_calibration.py` assigns one ID per detected board.
   - The RTT and tracking test scripts all send single-target `C` frames.

4. Old dual-board wire format is already gone from the main control path
   - There is no active second dial instance in `src/main.cpp`.
   - The migration commit `42f23b8` already removed the earlier paired control
     surface from the top-level firmware.

### Partially aligned

1. Single-dial behavior exists, but naming still follows 0.2.0
   - The code uses one active identity, but the public name is still
     `motor_id_0` instead of `dial_id`.
   - Parameter names still use suffixes such as `tracking_kp_0`.
   - `PROTOCOL.md` still documents firmware version `0.2.0`.

2. Control frame is close, but not yet strict P5
   - `C,<seq>,<target>[,<min>,<max>]` is accepted today.
   - P5 expects `C,<seq>,<target>,<min>,<max>` with bound validation.
   - Current firmware accepts control frames without bounds and does not reject
     `min > max`.

3. Telemetry payload is close, but not complete
   - Current telemetry already includes one ID, last processed sequence,
     angle, speed, torque, and FOC rate.
   - P5 additionally requires `status_bits` as the final field.

4. One-board runtime is migrated, but library internals still carry dual-motor
   legacy
   - `lib/dial/Dial.cpp` and `lib/dengfoc/DengFOC.*` still contain `M1` paths.
   - This is not currently blocking P5 behavior because the top-level firmware
     only uses `M0`.
   - This is no longer treated as deferred cleanup; removing the second-motor
     surface is part of the implementation scope.

### Missing or below spec

1. No `R` / Set Current Position support
   - `src/main.cpp` does not parse `R`.
   - `Dial` has no API to reset runtime state after a coordinate jump.
   - `Sensor_AS5600` exposes no software angle offset hook, so the new current
     position cannot yet be applied digitally.

2. No explicit P5 fault/status model
   - Invalid `C` frames are not faulted or surfaced in telemetry.
   - There is no `fault_active` state.
   - Telemetry has no `status_bits` field yet.

3. Default telemetry timing does not match the P5 target
   - Current default telemetry interval is `20 ms`.
   - P5 recommends default `10 ms`, but the migration decision is to keep
     `20 ms` as the initial default and leave runtime adjustment in place.

4. Runtime parameter surface still uses old names
   - `tracking_kp_0`, `enable_tracking_0`, and related names remain in the
     parser and protocol documentation.
   - P5 expects unsuffixed names such as `tracking_kp` and `enable_tracking`.

5. Documentation and scripts still use old wording
   - `PROTOCOL.md`, `device_discovery.py`, and `motor_id_calibration.py` still
     speak in terms of `motor_id`.
   - Test scripts still have stale hardcoded ports (`COM8`, `COM9`) and were
     written for the older bench setup.

6. Tooling baseline is not yet ready for repeatable validation
   - `platformio.ini` still sets `upload_port = COM3`, while the ready board is
     currently on `COM4`.
   - Running `python device_discovery.py` from the requested `game` conda
     environment failed because `pyserial` is not installed there.
   - Running `platformio run` in the current shell failed because the
     `platformio` CLI is not on `PATH` in this terminal.

## Status Matrix Against P5 Spec

| P5 requirement | Current status | Notes |
| --- | --- | --- |
| One board manages one dial | Partial | Functionally true in `src/main.cpp`, but public naming still says `motor_id_0`. |
| Single persistent `dial_id` | Partial | One persistent ID exists, but it is still named/documented as `motor_id_0`. |
| `C,<seq>,<target>,<min>,<max>` | Partial | Shape is close, but bounds are optional and invalid bounds are not rejected. |
| `R,<seq>,<current_pos>` | Missing | No parser support and no lower-level coordinate-offset mechanism yet. |
| Unsuffixed `S` parameter names | Missing | Firmware and docs still use `_0` names. |
| Telemetry with `status_bits` | Missing | Current line ends at FOC rate. |
| Fault visibility in telemetry | Missing | No fault bit/state today. |
| Auto-stream telemetry on port open | Complete | Already implemented. |
| `V`, `I`, `E`, `S` during streaming | Partial | Behavior exists, but this has not been revalidated on the current toolchain/hardware baseline. |
| Default telemetry 20 ms with runtime configurability | Partial | Current default is already 20 ms; keep it and preserve runtime configurability. |

## Recommended Migration Order

### Phase 0: Baseline and Validation Setup

Goal: make the repo testable on the current bench before changing behavior.

Tasks:

- Decide whether validation will use the PlatformIO CLI in shell or the VS Code
  PlatformIO task integration.
- Install `pyserial` into the `game` conda environment or agree on another
  Python environment for the host scripts.
- Ensure the chosen Python environment can also run automated serial tests.
- Update local test defaults to the current bench connection:
  - board on `COM4`
  - VID/PID `1A86:7523`
  - supply `12V`
- Update `platformio.ini` upload defaults or the upload workflow so the normal
  bench path targets `COM4`.
- Confirm the current firmware version and identity on the live board with a
  non-destructive probe.

Deliverable:

- Repeatable build/upload/probe commands documented and working.
- A stable environment for automated Python-based serial validation.

### Phase 1: Protocol Surface Cutover to P5

Goal: align the public wire contract without yet changing deeper control logic.

Tasks:

- Bump firmware version to `0.3.0`.
- Rename public identity semantics from `motor_id_0` to `dial_id` in:
  - firmware responses
  - Python discovery/calibration tooling
  - `PROTOCOL.md`
  - comments and user-facing script output
- Change `S` parameter names to the unsuffixed P5 names.
- Do not support legacy `_0` aliases; remove them as part of the strict P5
  cutover.
- Extend telemetry to append `status_bits` even if the first cut sends `0`.
- Make `C` require all three runtime values: target, min, max.
- Reject invalid `C` frames where `min > max`.
- Keep `telemetry_interval` as a runtime parameter and retain `20 ms` as the
  default value for the initial P5 pass.
- Remove all application-facing `M1` mentions and second-motor assumptions.

Deliverable:

- A strict P5 wire contract with no legacy parameter or identity naming.

### Phase 2: Implement `R` / Set Current Position Safely

Goal: support digital coordinate resets without a physical yank.

Tasks:

- Add a software position-offset mechanism.
- Reset speed estimate/state when `R` is processed.
- Reset tracking and pulse-timer state so stale dynamics do not fire after the
  coordinate jump.
- Ensure the next telemetry frame reports the new angle.
- Ensure the internal tracking target is updated to the new angle unless a
  newer `C` has already arrived.

Implementation note:

- The safest first approach appears to be a software offset layered above raw
  sensor angle, not modifying the electrical alignment logic.
- This likely means changes in `Dial` and/or the sensor abstraction rather than
  touching the FOC electrical zeroing path.

Deliverable:

- `R` accepted and acknowledged, with no visible yank in the acceptance test.

### Phase 3: Runtime State and Fault Semantics

Goal: make telemetry informative enough for host bring-up and safer failure
handling.

Tasks:

- Add `status_bits` packing for feature enable states.
- Add a bit for currently out-of-bounds.
- Add a minimal `fault_active` path for first bring-up.
- Decide which failures should immediately disable torque vs only reject a
  frame.

Suggested first-cut fault scope:

- invalid `C` frame (`min > max`)
- serial parse failure for required numeric fields
- encoder read failure only if it can be detected cheaply and reliably from the
  AS5600 path without materially complicating the control loop

Deliverable:

- Telemetry can show whether the board is tracking, enforcing bounds, outside
  limits, or faulted.

### Phase 4: Bench Validation on COM4

Goal: verify the migrated behavior against the actual P5 acceptance needs.

Tasks:

- Verify `V`, `I`, `E`, `S`, `C`, and `R` while telemetry is streaming.
- Run a steady control stream test.
- Verify the `R` no-yank behavior with the shaft physically held still.
- Verify bounds restoration and OOB kick with narrow bounds around zero.
- Capture measured FOC rate on the current hardware.
- Upload firmware from the bench workflow and then run automated Python serial
  tests against the live board on `COM4`.
- Update the Python scripts or add dedicated P5 validation scripts if the
  existing scripts are no longer a clean fit.

Deliverable:

- A short bring-up record confirming which P5 acceptance points passed.

### Phase 5: Continuous-Use Robustness Audit

Goal: harden the firmware for multi-day continuous operation by identifying and
fixing state growth, overflow, leak, drift, and long-run stability issues that
may not appear during short bench validation.

Tasks:

- Audit all long-lived counters, timers, accumulators, offsets, and sequence
  handling for overflow or wraparound hazards.
- Revisit multi-turn angle tracking and logical rebasing behavior to ensure the
  dial can operate for very long periods without coordinate corruption.
- Check for state that can grow unbounded during repeated `C` and `R` updates.
- Check for memory growth or resource leaks in both firmware and host-side
  validation tooling.
- Review serial parsing and buffering paths for long-run resilience under
  continuous traffic, intermittent traffic or overloaded traffic.
- Review telemetry formatting and numeric range assumptions so long-duration use
  cannot silently exceed wire-format limits.
- Add long-duration soak tests and repeated-reset tests that specifically try
  to expose:
  - angle/state drift
  - counter wrap issues
  - stale timer behavior
  - serial parser degradation
  - loss of responsiveness over time

Deliverable:

- A documented list of continuous-use failure modes found, the fixes applied,
  and the remaining operational limits if any.

### Phase 6: Post-Fix Regression and Soak Revalidation

Goal: rerun the functional and robustness checks after Phase 5 fixes so the
final behavior is confirmed rather than assumed.

Tasks:

- Rerun all automated smoke, protocol, `R`, bounds, and streaming tests.
- Rerun the manual no-yank and bounds-feel checks if control behavior changed.
- Run the new soak and repeated-reset validation suite after robustness fixes.
- Confirm no regression in protocol behavior, telemetry format, or runtime
  stability.

Deliverable:

- A final validation record showing both short-run correctness and long-run
  robustness.

## Automated Test Strategy

The existing `test/` directory is the PlatformIO unit-test location. It is not
the right home for PC-driven serial integration tests that upload firmware,
open `COM4`, send protocol commands, and verify telemetry.

Recommended structure:

- Keep `test/` for PlatformIO-native embedded unit tests only.
- Create a separate root folder for host-driven validation, preferably
  `host_tests/` or `validation/`.
- Put Python scripts there that can:
  - upload firmware
  - open the board serial port
  - send protocol commands
  - parse telemetry
  - assert expected behavior
  - print a concise pass/fail summary for bench runs

Recommended first-pass contents:

- `validation/common.py`
  - serial helpers
  - command send/wait helpers
  - telemetry parsing
  - timeouts and assertion utilities
- `validation/upload_and_smoke.py`
  - upload firmware
  - confirm board responds to `V`, `I`, and telemetry stream
- `validation/test_protocol_smoke.py`
  - verify `V`, `I`, `E`, `S`, `C` basic behavior
- `validation/test_set_current_position.py`
  - verify `R` acknowledgement
  - verify next telemetry angle rebases correctly
  - verify speed settles near zero after rebasing
- `validation/test_bounds_behavior.py`
  - verify invalid bounds are rejected
  - verify valid bounds are accepted
  - verify out-of-bounds bit behavior from telemetry
- `validation/test_streaming_stability.py`
  - run sustained `C` updates at target host rate
  - verify telemetry sequence updates and no obvious protocol stalls
- `validation/test_long_run_soak.py`
  - run extended command and telemetry traffic for a long-duration session
  - check responsiveness, sequence progress, and telemetry continuity over time
- `validation/test_repeated_rebase.py`
  - repeatedly issue `R` commands across many cycles
  - verify that rebasing does not accumulate angle corruption or protocol drift

### Tests that can be automated without user intervention

These can run unattended with the connected board and free-spinning dial.

1. Build and upload firmware to `COM4`.
2. Probe `V` and `I` responses.
3. Confirm telemetry starts automatically on port open.
4. Verify `E` round-trip responses.
5. Verify `S` parameter acknowledgement for supported parameters.
6. Verify `C` accepts valid frames and rejects invalid `min > max` frames.
7. Verify telemetry format, including `status_bits` field presence.
8. Verify `R` rebases the reported angle and acknowledges correctly.
9. Verify sustained command streaming updates the echoed sequence number.
10. Run long-duration soak traffic and confirm the board remains responsive.
11. Run repeated `R` cycles and verify no accumulating coordinate corruption.

### Tests that still need manual interaction

These require physical intervention or human judgment about haptic feel.

1. No-yank acceptance for `R` under physically stationary hold.
2. Subjective wall feel and OOB kick quality.
3. Any acceptance step that depends on felt torque quality rather than serially
   observable state.

### Bench-run sequence after implementation starts

Recommended order:

1. Upload firmware.
2. Run automated smoke tests.
3. Run automated protocol and `R` tests.
4. Run automated streaming stability checks.
5. Perform the manual no-yank and bounds-feel checks.
6. Run the long-duration soak and repeated-rebase validation suite after any
  robustness fixes.

## What Should Not Be Phase-1 Scope

These items are real cleanup opportunities, but they should not delay the P5
contract cutover unless they become blockers during implementation.

- Refactoring the `Dial` class naming/style
- Reorganizing all legacy comments from the DengFOC upstream code
- General code cleanup not tied to the P5 acceptance criteria

## Questions To Decide Before Implementation

The major implementation decisions are now resolved. Remaining details should
be handled during implementation rather than treated as planning blockers.

## Proposed Next Step

Start with Phase 0 plus Phase 1, then stop for a bench check before
implementing `R`.

That split keeps the migration low-risk:

- first align the wire contract and tooling
- then validate serial behavior on the live board
- then add the deeper control-state change required for `R`

## Evidence From The Repo

### Already migrated

- src/main.cpp only loads one persistent ID, aligns one motor, runs one FOC
  path, and streams one telemetry payload.
- PROTOCOL.md, device_discovery.py, motor_id_calibration.py, and the dial test
  scripts were already rewritten away from the old dual-dial frame shape.
- Git history shows an intentional partial migration in commit 42f23b8
  (`Switched to ONE Motor Config.`).

### Still on the old contract

- Firmware version is still `0.2.0`.
- Identity is still keyed as `motor_id_0` in NVS and on the wire.
- Runtime parameter names still require `_0` suffixes.
- Telemetry still emits `T,id,seq,ang,spd,tor,foc_rate` without
  `status_bits`.
- The parser does not implement `R`.

### Deeper control-layer gap

The P5 `R` command is not just a parser change. The current control stack has
no digital position rebasing hook:

- Dial uses raw encoder angle/speed from the DengFOC layer.
- Sensor_AS5600 tracks continuous angle through `full_rotations`, but exposes
  no API for resetting the logical position or zeroing the derived velocity.
- `last_kick_time_local`, `kick_state`, and tracking state are not resettable
  from a command handler.

This means `R` requires a deliberate control-state design, not only a new
serial command.

## Recommended Migration Strategy

### Phase 0: Lock the migration rules

Before changing behavior, decide the compatibility rules for this branch.

Decisions needed:

1. Should the first P5 firmware be strict 0.3.0 only, or temporarily accept
   legacy `_0` parameter names and legacy 7-field telemetry during bring-up?
2. Should identity migrate fully to a new `dial_id` NVS key, or should the
   firmware read legacy `motor_id_0` if `dial_id` is absent for upgrade safety?
3. Should the unused M1 code remain in lib/dengfoc for now, or do you want the
   first implementation pass to prune the second-motor surface as well?
4. Do you want the first P5 bring-up to change the default telemetry interval
   to 10 ms immediately, or keep 20 ms until online validation is stable?

### Phase 1: Protocol surface migration

Update the firmware-facing protocol surface first, without changing torque math
 beyond what is needed for correctness.

Tasks:

- Bump firmware version to `0.3.0`.
- Rename identity terminology in code and docs from `motor_id_0` to `dial_id`.
- Change `I` handling to use one `dial_id`.
- Change `S` parsing to accept the plain single-dial parameter names from the
  P5 spec.
- Update telemetry to always emit the extra `status_bits` field.
- Make `C` require exactly `target,min,max` and reject invalid `min > max`
  without mutating active state.

Success criteria:

- `V`, `I`, `S`, `E`, and `C` all follow the P5 wire contract.
- Telemetry shape is stable and forward-compatible even if `status_bits` is
  initially `0`.

### Phase 2: Implement Set Current Position correctly

This is the highest-risk firmware change and should be handled as its own
slice.

Tasks:

- Add one explicit logical position offset or rebasing mechanism in the dial or
  sensor layer.
- Add a `set_current_position()` style operation that:
  - updates the reported dial angle immediately
  - zeroes the derived velocity estimate
  - resets OOB kick timing and pulse state
  - resets any tracking derivative or integrator memory that could create a
    jerk
  - updates the internal tracking target to the new current position unless a
    newer `C` has already arrived
- Add `R,<seq>,<current_pos>` parsing and `R,<seq>` acknowledgement.

Design note:

The cleanest implementation is likely to keep the raw encoder reading intact
and introduce a firmware-level logical angle offset for telemetry, bounds, and
tracking. Directly mutating raw sensor state is possible, but riskier because
it couples serial commands to the encoder unwrap logic.

Success criteria:

- The next telemetry frame reflects the new logical angle.
- A stationary dial remains physically calm after `R`.
- No stale OOB kick pulse fires immediately after rebasing.

### Phase 3: Add status and safety handling

Tasks:

- Define and emit `status_bits`.
- Track at minimum:
  - tracking enabled
  - bounds restoration enabled
  - OOB kick enabled
  - detent enabled
  - vibration enabled
  - currently out of bounds
  - fault active
- Add explicit invalid-frame handling for impossible bounds.
- Add a minimal fault path that clamps or disables torque while keeping
  telemetry alive when the controller detects a faulted condition.

Success criteria:

- Host can infer runtime state from telemetry alone.
- Unsafe command values do not produce uncontrolled torque output.

### Phase 4: Host-side tool migration

Tasks:

- Update PROTOCOL.md to match the P5 contract or clearly mark it as superseded.
- Rename host script terminology from `motor_id` to `dial_id`.
- Update device discovery and calibration tooling to use the final identity
  naming and telemetry shape.
- Update existing test scripts to parse the extra telemetry field.
- Add one new online test script specifically for `R` no-yank behavior.
- Remove hard-coded COM port defaults from ad hoc scripts or move them behind
  arguments.

Success criteria:

- All shipped scripts speak the same protocol as firmware.
- There is one repeatable online acceptance path for COM4 bring-up.

### Phase 7: Optional cleanup after bring-up

Only after P5 behavior is working on hardware:

- Decide whether to prune unused M1 codepaths from lib/dengfoc and lib/dial.
- Normalize naming in comments and docs.
- Remove compatibility shims if a temporary dual-protocol bridge was used.

This cleanup should not block first working P5 firmware.

## Proposed Implementation Order

Recommended order for low-risk delivery:

1. Protocol rename and telemetry shape.
2. `R` state model in the control layer.
3. `R` parser and acknowledgement.
4. `status_bits` and minimal fault reporting.
5. Host tools and validation scripts.
6. Initial bench validation.
7. Continuous-use robustness audit and fixes.
8. Post-fix regression and soak revalidation.
9. Optional library cleanup.

This order keeps the most behavior-sensitive change (`R`) isolated and easier
to test.

## Validation Plan

### Offline checks

- Build with PlatformIO after each protocol/control slice.
- Keep command parsing changes and control-state changes in separate build/test
  steps.
- Verify that the firmware still streams telemetry on port open.

### Online checks on the current setup

Known setup from the workspace request:

- One FOC board connected on COM4
- Dial can rotate freely
- Supply voltage 12 V
- USB VID:PID = 1A86:7523

Bring-up sequence:

1. Query `V` and `I` on COM4.
2. Confirm telemetry starts automatically.
3. Verify `C` handling at 50 Hz.
4. Verify `C` handling at 150 Hz for sustained stability.
5. Run the new `R` no-yank acceptance test while the dial is stationary.
6. Run narrow-bounds wall and OOB kick tests.
7. Confirm telemetry `status_bits` matches runtime mode changes.
8. Run long-duration soak traffic to look for drift, stalls, or overflow
  symptoms.
9. Run repeated rebasing to verify `R` remains stable over many cycles.

### Environment notes discovered during planning

- The current `game` conda environment does not have `pyserial` installed, so
  existing Python tools cannot be run there yet.
- `platformio` is not currently on this shell PATH, so build execution needs
  either a PATH fix or use through the VS Code PlatformIO integration.

These are tooling blockers for validation, not firmware design blockers.

## Recommended Questions To Resolve Before Implementation

Please decide these before code changes begin:

1. Do you want a strict P5 cutover, or a temporary compatibility layer for old
   `_0` parameter names and old telemetry shape during bring-up?
2. Should identity storage move to a new `dial_id` key with fallback migration
   from `motor_id_0`, or should we preserve the old key name and only rename
   the public API?
3. Should I keep the unused M1 lower-layer code untouched for the first pass,
   or include a deeper single-motor cleanup in the same migration?
4. Should the default telemetry interval switch to 10 ms now, or stay at 20 ms
   until the board passes online stability checks?

## Bottom Line

The repository is not starting from zero. The one-dial board migration already
landed in the top-level firmware and host utilities, but the P5 protocol,
control semantics, and long-duration robustness are still incomplete. The main
remaining engineering work is not removing the second dial; it is implementing
a safe and testable logical position rebase (`R`), aligning the protocol
surface around it, and then proving the firmware can survive continuous use
without overflow, drift, or long-run instability.