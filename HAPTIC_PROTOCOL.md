# Haptic Controller Communication Protocol

**Firmware version:** 0.3.0
**Transport:** USB Serial (UART), 230400 baud, 8N1
**Line format:** ASCII, comma-separated fields, terminated by `\n` (newline). `\r` is ignored.
**Max line length:** 127 bytes (excluding newline).

Each ESP32 board drives **one dial**. The dial is identified by one persistent `dial_id` stored in NVS flash.

---

## 1. Physical Layer

| Parameter       | Value              |
|-----------------|--------------------|
| Baud rate       | 230400             |
| Data bits       | 8                  |
| Parity          | None               |
| Stop bits       | 1                  |
| Flow control    | None               |
| USB chip        | CH340 |

Known VID/PID pairs for auto-detection:

| Chip      | VID    | PID    |
|-----------|--------|--------|
| CH340     | 0x1A86 | 0x7522 |
| CH340     | 0x1A86 | 0x7523 |


---

## 2. Units & Encoding Conventions

All numeric fields are transmitted as **ASCII decimal integers** (no floating point on the wire).

| Quantity     | Wire unit       | Conversion                              |
|--------------|-----------------|-----------------------------------------|
| Angle        | Decidegrees     | 1 decidegree = 0.1°. `rad = decideg × π / 1800` |
| Torque       | Milliamps (mA)  | 1000 mA = 1.0 A                        |
| PD gains     | ×1000 fixed-point | `float = wire_value / 1000.0`         |
| Detent distance | ×1000 fixed-point (decidegrees) | `float = (wire_value / 1000.0) × π / 1800` |
| FOC rate     | Hz (integer)    | Direct reading                          |
| Telemetry interval | Milliseconds | Direct reading (no ×1000 scaling)    |
| Sequence number | uint32       | Monotonically increasing, wraps at 2³² |

Angle range: ±1,080,000 decidegrees (±30 full rotations).
Torque clamp: ±10,000 mA on telemetry output.

---

## 3. Commands (Host → Controller)

All commands follow the pattern: `<CMD>,<seq>[,<fields>...]\n`

The `seq` field is a uint32 sequence number chosen by the host. The controller echoes it back in the response (where applicable) and includes the last-processed `C` command's seq in every telemetry frame.

### 3.1 `C` — Control Update

Sends the tracking target and active soft bounds for the dial. This is the **primary real-time command** sent every control cycle.

**Format:**
```
C,<seq>,<target>,<min>,<max>\n
```

| Field  | Type   | Required | Description |
|--------|--------|----------|-------------|
| seq    | uint32 | Yes | Sequence number |
| target | long   | Yes | Tracking target for the dial (decidegrees) |
| min    | long   | Yes | Lower active bound for the dial (decidegrees) |
| max    | long   | Yes | Upper active bound for the dial (decidegrees) |

**Response:** None (acknowledged implicitly via telemetry seq echo).

**Timing:** Send at a steady rate, typically **50 Hz**. The controller applies the latest values on every FOC cycle (~500 Hz). Sending faster than the serial link can handle will cause buffering; sending slower is fine but reduces tracking responsiveness.

If `min > max`, the frame is rejected and the previous valid control state is retained.

**Example:**
```
C,42,1800,-3600,3600\n            → Set target to +180.0° with bounds ±360°
```

### 3.2 `R` — Set Current Position

Digitally redefines the current logical dial position without requiring physical motion and without yanking toward an old target.

**Format:**
```
R,<seq>,<current_pos>\n
```

| Field       | Type   | Description |
|-------------|--------|-------------|
| seq         | uint32 | Sequence number |
| current_pos | long   | Desired current dial angle (decidegrees) |

**Response:**
```
R,<seq>\n
```

### 3.3 `S` — Set Parameter

Modifies a single runtime parameter. Changes take effect immediately but are **not persisted** across reboots (except motor IDs, which use the `I` command).

**Format:**
```
S,<seq>,<param_name>,<value>\n
```

| Field      | Type   | Description |
|------------|--------|-------------|
| seq        | uint32 | Sequence number |
| param_name | string | Parameter name (see table below) |
| value      | long   | Integer value (interpretation depends on parameter) |

**Response:**
```
S,<seq>\n
```

**Timing:** On-demand. Send when configuration needs to change.

#### Available Parameters

Most parameter values use **×1000 fixed-point** encoding: send `5000` to set a float value of `5.0`.

| Parameter name          | Unit (wire)       | Description | Default (float) |
|-------------------------|-------------------|-------------|------------------|
| `tracking_kp`         | ×1000             | Tracking proportional gain | 5.0 |
| `tracking_kd`         | ×1000             | Tracking derivative gain (damping) | 0.1 |
| `detent_kp`           | ×1000             | Detent spring gain | 5.0 |
| `bounds_kp`           | ×1000             | Bounds restoration gain | 20.0 |
| `detent_distance`     | ×1000 (decideg)   | Detent spacing. Converted: `(val/1000) × π/1800` rad | ~10° |
| `vibration_amplitude` | ×1000             | Vibration pulse amplitude (A) | 1.0 |
| `oob_kick_amplitude`  | ×1000             | Out-of-bounds kick amplitude (A) | 1.0 |
| `tracking_max_torque` | ×1000             | Max tracking torque (A) | 2.0 |
| `bounds_max_torque`   | ×1000             | Max bounds restoration torque (A) | 3.0 |
| `detent_max_torque`   | ×1000             | Max detent torque (A) | 1.0 |
| `vibration_pulse_interval_ms` | Milliseconds (raw, no ×1000) | Vibration pulse interval | 1000 ms |
| `oob_kick_pulse_interval_ms`  | Milliseconds (raw, no ×1000) | OOB kick pulse interval | 40 ms |
| `enable_tracking`     | 0 or 1            | Enable position tracking | 1 (enabled) |
| `enable_detent`       | 0 or 1            | Enable detent mode | 0 (disabled) |
| `enable_bounds_restoration` | 0 or 1      | Enable bounds restoration | 1 (enabled) |
| `enable_oob_kick`     | 0 or 1            | Enable OOB kick | 1 (enabled) |
| `enable_vibration`    | 0 or 1            | Enable vibration mode | 0 (disabled) |
| `telemetry_interval`    | Milliseconds (raw, no ×1000) | Telemetry reporting period | 20 ms (50 Hz) |

Unknown parameter names are silently ignored.

**Example:**
```
S,100,tracking_kp,8000\n   → Set tracking Kp to 8.0
S,101,telemetry_interval,10\n → Set telemetry to 100 Hz (10 ms)
```

### 3.4 `I` — Identity (Get/Set Dial ID)

Reads or writes the persistent `dial_id` stored in NVS flash. The dial ID survives reboots and is included in every telemetry frame, allowing the host to map physical devices to logical dials.

**Query format:**
```
I,<seq>\n
```

**Set format:**
```
I,<seq>,<dial_id>\n
```

| Field | Type   | Description |
|-------|--------|-------------|
| seq   | uint32 | Sequence number |
| dial_id | uint8  | Identity for this dial (0 = unconfigured, 1–255 = assigned) |

**Response (both query and set):**
```
I,<seq>,<dial_id>\n
```

**Timing:** On-demand. Typically used once during initial provisioning or device discovery.

**Example:**
```
I,1\n             → Query current ID. Response: I,1,3
I,2,11\n          → Set the dial to ID 11. Response: I,2,11
```

### 3.5 `V` — Version Query

Returns the firmware version string.

**Format:**
```
V,<seq>\n
```

**Response:**
```
V,<seq>,<fw_version>\n
```

**Timing:** On-demand. Used during device discovery to confirm the connected device is running the expected firmware.

**Example:**
```
V,1\n    → Response: V,1,0.3.0
```

### 3.6 `E` — Echo (Ping)

Round-trip latency test. The controller echoes the sequence number immediately.

**Format:**
```
E,<seq>\n
```

**Response:**
```
E,<seq>\n
```

**Timing:** On-demand. Useful for measuring serial round-trip time.

---

## 4. Telemetry (Controller → Host)

The controller emits telemetry frames autonomously at a configurable interval (default 20 ms / 50 Hz). Telemetry starts streaming as soon as the serial port is opened — no subscription command is needed.

On boot, the firmware automatically rebases the current physical shaft angle to logical `0` before closed-loop tracking runs. This makes the startup behavior equivalent to an immediate `R,<seq>,0` without an acknowledgement frame and prevents a torque spike if the absolute encoder powers up away from the previous logical origin.

Closed-loop tracking remains passive until the first valid `C` command is received. Until then, telemetry still streams, but the controller applies zero commanded torque instead of trying to hold a default target with `seq=0`.

**Format:**
```
T,<dial_id>,<seq>,<ang>,<spd>,<tor>,<foc_rate>,<status_bits>\n
```

| Field      | Type   | Description |
|------------|--------|-------------|
| dial_id    | uint8  | Persistent identity of this dial |
| seq        | uint32 | Sequence number of the last processed `C` command (0 if none received) |
| ang        | long   | Current dial angle (decidegrees) |
| spd        | long   | Current dial speed (decidegrees/s) |
| tor        | long   | Current applied torque (milliamps) |
| foc_rate   | long   | FOC loop rate (Hz), measured over 200 ms windows. Range: 0–2000 |
| status_bits | long  | Decimal ASCII bitfield describing runtime state |

`status_bits` layout:

- bit 0: tracking enabled
- bit 1: bounds restoration enabled
- bit 2: OOB kick enabled
- bit 3: detent enabled
- bit 4: vibration enabled
- bit 5: currently out of bounds
- bit 6: fault active

**Example:**
```
T,11,42,1805,500,150,1100,35
```
Interpretation: Dial ID 11, last host seq 42, dial at 180.5° moving at 50.0°/s, torque 0.15 A, FOC running at 1100 Hz, with bits 0, 1, and 5 set.

---

## 5. Typical Interaction Sequence

```
Host                                Controller
 │                                      │
 │  (open serial port)                  │
 │                                      │──── T,0,0,0,0,0,1100,0   (auto-streaming)
 │                                      │──── T,0,0,5,0,0,1100,0
 │                                      │
 │── V,1                               │     (version query)
 │                                      │──── V,1,0.3.0
 │                                      │
 │── I,2                               │     (read dial ID)
 │                                      │──── I,2,0                  (unconfigured)
 │                                      │
 │── I,3,11                            │     (assign ID 11, only if needed)
 │                                      │──── I,3,11
 │                                      │
 │── S,10,telemetry_interval,20        │     (confirm 50 Hz telemetry, only if needed)
 │                                      │──── S,10
 │                                      │
 │  ┌─ 50 Hz control loop ────────┐    │
 │  │ C,100,0,-3600,3600           │    │     (set position + bounds)
 │  │                              │    │──── T,11,100,2,10,50,1100,3
 │  │ R,101,0                      │    │
 │  │                              │    │──── R,101
 │  │                              │    │──── T,11,100,0,0,0,1100,3
 │  │ ...                          │    │
 │  └──────────────────────────────┘    │
```

---

## 6. Error Handling

- **Buffer overflow:** If a command line exceeds 127 bytes, the buffer is reset and the malformed command is discarded.
- **Malformed commands:** Commands with missing required fields are discarded and `status_bits` bit 6 may be set until a later valid command clears the fault state.
- **Unknown command letters:** Ignored and treated as a protocol fault.
- **Unknown `S` parameter names:** Ignored. The command is still acknowledged.

---

## 7. Device Discovery & Multi-Controller Setup

When multiple ESP32 controllers are connected via USB:

1. **Enumerate** all serial ports matching known VID/PID pairs (see Section 1).
2. **Probe** each port by sending `V,<seq>\n` and waiting for `V,<seq>,<version>\n` (timeout ~1.5 s). Filter through any telemetry `T,...` lines that arrive first.
3. **Read identity** with `I,<seq>\n` to get the dial IDs assigned to each board.
4. **Build a device map:** `{dial_id: "COMx", ...}` so the host can address dials by logical ID.
5. **Assign identity** (one-time provisioning): Use the `motor_id_calibration.py` tool or send `I,<seq>,<dial_id>\n` to write a persistent ID. The calibration tool detects which dial is which by monitoring telemetry angle changes while the user physically moves each dial.

---

## 8. Torque Control Modes

The controller computes a composite torque from multiple independently-enabled effects. These are configured via the `S` command parameters and the `C` command bounds. The modes are not mutually exclusive and their torques are summed.

| Mode | Default | Description |
|------|---------|-------------|
| **Tracking** | Enabled | PD controller driving the dial towards `tracking_position` (set by `C` command). Gains: `tracking_kp`, `tracking_kd`. Max torque: `tracking_max_torque`. |
| **Bounds restoration** | Enabled | Strong proportional spring when angle exceeds `bounds_min_angle` / `bounds_max_angle` (set by `C` command). Gain: `bounds_kp`. Max torque: `bounds_max_torque`. |
| **OOB kick** | Enabled | Pulsed corrective force when outside bounds. Amplitude: `oob_kick_amplitude`. Interval: `oob_kick_pulse_interval_ms`. |
| **Detent** | Disabled | Spring-like snap to evenly-spaced detent positions. Spacing: `detent_distance`. Gain: `detent_kp`. Max torque: `detent_max_torque`. |
| **Vibration** | Disabled | Periodic pulse for testing. Amplitude: `vibration_amplitude`. Interval: `vibration_pulse_interval_ms`. |

All modes can be enabled or disabled at runtime via the `S` command using the `enable_<mode>` parameters (e.g., `enable_detent`). Send `0` to disable, `1` to enable. Modes are not mutually exclusive — their torques are summed.
