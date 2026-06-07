import argparse
import csv
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from validation.common import open_serial, parse_telemetry, resolve_test_port, send_and_wait, send_command, wait_for_telemetry


DEFAULT_TELEMETRY_INTERVAL_MS = 20
TUNING_TELEMETRY_INTERVAL_MS = 5
DEFAULT_TRACKING_KP = 2.5
DEFAULT_TRACKING_KD = 0.002
DEFAULT_TRACKING_MAX_TORQUE = 6.0
DEFAULT_WIDE_BOUND_TURNS = 20.0
DEFAULT_SMALL_CAPTURE_S = 2.5
DEFAULT_LARGE_CAPTURE_S = 6.0


def deg_to_decideg(value: float) -> int:
    return int(round(value * 10.0))


def decideg_to_deg(value: int) -> float:
    return value / 10.0


def turns_to_decideg(value: float) -> int:
    return int(round(value * 3600.0))


def parse_turns(raw: str) -> list[float]:
    turns = []
    for part in raw.split(","):
        token = part.strip()
        if token:
            turns.append(float(token))
    if not turns:
        raise ValueError("Expected at least one turn amplitude")
    return turns


def next_seq(seq_state: dict[str, int]) -> int:
    seq_state["value"] += 1
    return seq_state["value"]


def set_runtime_float(ser, seq_state: dict[str, int], name: str, value: float) -> None:
    seq = next_seq(seq_state)
    scaled = int(round(value * 1000.0))
    ack = send_and_wait(ser, f"S,{seq},{name},{scaled}", f"S,{seq}")
    if ack != f"S,{seq}":
        raise RuntimeError(f"Unexpected acknowledgement while setting {name}: {ack}")


def set_runtime_bool(ser, seq_state: dict[str, int], name: str, value: bool) -> None:
    seq = next_seq(seq_state)
    ack = send_and_wait(ser, f"S,{seq},{name},{1 if value else 0}", f"S,{seq}")
    if ack != f"S,{seq}":
        raise RuntimeError(f"Unexpected acknowledgement while setting {name}: {ack}")


def set_runtime_int(ser, seq_state: dict[str, int], name: str, value: int) -> None:
    seq = next_seq(seq_state)
    ack = send_and_wait(ser, f"S,{seq},{name},{value}", f"S,{seq}")
    if ack != f"S,{seq}":
        raise RuntimeError(f"Unexpected acknowledgement while setting {name}: {ack}")


def apply_tracking_params(ser, seq_state: dict[str, int], args: argparse.Namespace) -> None:
    set_runtime_float(ser, seq_state, "tracking_kp", args.tracking_kp)
    set_runtime_float(ser, seq_state, "tracking_kd", args.tracking_kd)
    set_runtime_float(ser, seq_state, "tracking_max_torque", args.tracking_max_torque)


def apply_feature_flags(ser, seq_state: dict[str, int], tracking: bool, bounds: bool, kick: bool) -> None:
    set_runtime_bool(ser, seq_state, "enable_tracking", tracking)
    set_runtime_bool(ser, seq_state, "enable_bounds_restoration", bounds)
    set_runtime_bool(ser, seq_state, "enable_oob_kick", kick)


def reset_current_position_to_zero(ser, seq_state: dict[str, int]) -> None:
    seq = next_seq(seq_state)
    ack = send_and_wait(ser, f"R,{seq},0", f"R,{seq}")
    if ack != f"R,{seq}":
        raise RuntimeError(f"Unexpected acknowledgement while rebasing current position: {ack}")
    wait_for_telemetry(ser)


def capture_window(ser, target_decideg: int, duration_s: float) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    deadline = time.monotonic() + duration_s
    start = time.monotonic()
    buffer = ""

    while time.monotonic() < deadline:
        data = ser.read(ser.in_waiting or 1)
        if not data:
            continue

        buffer += data.decode(errors="ignore")
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if not line.startswith("T,"):
                continue

            telemetry = parse_telemetry(line)
            error_decideg = int(telemetry["angle_decideg"]) - target_decideg
            rows.append(
                {
                    "t_s": round(time.monotonic() - start, 4),
                    **telemetry,
                    "target_decideg": target_decideg,
                    "target_deg": round(decideg_to_deg(target_decideg), 3),
                    "angle_deg": round(decideg_to_deg(int(telemetry["angle_decideg"])), 3),
                    "error_decideg": error_decideg,
                    "error_deg": round(decideg_to_deg(error_decideg), 3),
                    "speed_deg_s": round(decideg_to_deg(int(telemetry["speed_decideg_s"])), 3),
                }
            )

    return rows


def command_and_capture(
    ser,
    seq_state: dict[str, int],
    target_decideg: int,
    minimum_decideg: int,
    maximum_decideg: int,
    duration_s: float,
) -> tuple[int, list[dict[str, Any]]]:
    seq = next_seq(seq_state)
    ser.reset_input_buffer()
    send_command(ser, f"C,{seq},{target_decideg},{minimum_decideg},{maximum_decideg}")
    rows = capture_window(ser, target_decideg, duration_s)
    response_rows = [row for row in rows if int(row["seq"]) == seq]
    if not response_rows:
        raise RuntimeError(f"No telemetry captured for command sequence {seq}")
    return seq, response_rows


def find_reach_time(rows: list[dict[str, Any]], target_decideg: int, fraction: float) -> float | None:
    if not rows:
        return None
    initial_angle = int(rows[0]["angle_decideg"])
    delta = target_decideg - initial_angle
    if delta == 0:
        return 0.0

    threshold = initial_angle + int(round(delta * fraction))
    for row in rows:
        angle = int(row["angle_decideg"])
        if delta > 0 and angle >= threshold:
            return float(row["t_s"])
        if delta < 0 and angle <= threshold:
            return float(row["t_s"])
    return None


def compute_overshoot_decideg(rows: list[dict[str, Any]], target_decideg: int) -> int:
    if not rows:
        return 0
    initial_error = int(rows[0]["angle_decideg"]) - target_decideg
    if initial_error == 0:
        return 0
    sign = 1 if initial_error > 0 else -1
    overshoot = 0
    for row in rows:
        error = int(row["angle_decideg"]) - target_decideg
        opposite = -sign * error
        if opposite > overshoot:
            overshoot = opposite
    return max(0, overshoot)


def mean_abs_error_deg(rows: list[dict[str, Any]]) -> float:
    return round(sum(abs(int(row["error_decideg"])) for row in rows) / len(rows) / 10.0, 3)


def find_settle_metrics(
    rows: list[dict[str, Any]],
    movement_tolerance_decideg: int,
    settle_frames: int,
) -> tuple[float | None, float | None, float | None]:
    if len(rows) < settle_frames:
        return None, None, None
    settle_index = None
    remaining_movement_deg = None
    for index in range(len(rows) - settle_frames + 1):
        tail_rows = rows[index:]
        tail_angles = [int(row["angle_decideg"]) for row in tail_rows]
        remaining_movement_decideg = max(tail_angles) - min(tail_angles)
        if remaining_movement_decideg <= movement_tolerance_decideg:
            settle_index = index
            remaining_movement_deg = decideg_to_deg(remaining_movement_decideg)
            break

    if settle_index is None:
        return None, None, None

    tail_rows = rows[settle_index:]
    settle_error_deg = mean_abs_error_deg(tail_rows)
    return float(rows[settle_index]["t_s"]), settle_error_deg, round(remaining_movement_deg, 3)


def summarize_test(rows: list[dict[str, Any]], target_decideg: int, args: argparse.Namespace) -> dict[str, Any]:
    settle_time_s, settle_error_deg, remaining_movement_deg = find_settle_metrics(
        rows,
        args.settle_movement_decideg,
        args.settle_hold_frames,
    )
    abs_errors_decideg = [abs(int(row["error_decideg"])) for row in rows]
    squared_errors = [decideg_to_deg(int(row["error_decideg"])) ** 2 for row in rows]
    steady_state_window = rows[-min(len(rows), args.steady_state_samples) :]
    steady_state_abs_error = [abs(float(row["error_deg"])) for row in steady_state_window]

    return {
        "sample_count": len(rows),
        "target_turns": round(target_decideg / 3600.0, 4),
        "initial_angle_deg": round(float(rows[0]["angle_deg"]), 3),
        "final_angle_deg": round(float(rows[-1]["angle_deg"]), 3),
        "final_error_deg": round(float(rows[-1]["error_deg"]), 3),
        "peak_abs_error_deg": round(decideg_to_deg(max(abs_errors_decideg)), 3),
        "rms_error_deg": round(math.sqrt(sum(squared_errors) / len(squared_errors)), 3),
        "overshoot_deg": round(decideg_to_deg(compute_overshoot_decideg(rows, target_decideg)), 3),
        "reach_90_time_s": find_reach_time(rows, target_decideg, 0.9),
        "settle_time_s": settle_time_s,
        "settle_error_deg": settle_error_deg,
        "remaining_movement_deg": remaining_movement_deg,
        "steady_state_mean_abs_error_deg": round(sum(steady_state_abs_error) / len(steady_state_abs_error), 3),
        "peak_abs_speed_deg_s": round(max(abs(float(row["speed_deg_s"])) for row in rows), 3),
        "peak_abs_torque_ma": max(abs(int(row["torque_ma"])) for row in rows),
    }


def run_tracking_test(
    ser,
    seq_state: dict[str, int],
    args: argparse.Namespace,
    amplitude_turns: float,
    direction: int,
) -> dict[str, Any]:
    target_decideg = turns_to_decideg(amplitude_turns) * direction
    capture_s = args.large_step_capture_s if abs(amplitude_turns) >= 1.0 else args.small_step_capture_s
    reset_current_position_to_zero(ser, seq_state)
    _, rows = command_and_capture(
        ser,
        seq_state,
        target_decideg,
        -args.wide_bound_decideg,
        args.wide_bound_decideg,
        capture_s,
    )
    test_name = f"tracking_{amplitude_turns:g}turn_{'pos' if direction > 0 else 'neg'}"
    return {
        "test_name": test_name,
        "target_turns": amplitude_turns * direction,
        "target_deg": round(target_decideg / 10.0, 3),
        "rows": rows,
        "metrics": summarize_test(rows, target_decideg, args),
    }


def write_csv(output_path: Path, run_id: str, tests: list[dict[str, Any]]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "run_id",
                "test_name",
                "t_s",
                "seq",
                "dial_id",
                "target_deg",
                "angle_deg",
                "error_deg",
                "speed_deg_s",
                "torque_ma",
                "foc_rate_hz",
                "status_bits",
            ]
        )
        for test in tests:
            for row in test["rows"]:
                writer.writerow(
                    [
                        run_id,
                        test["test_name"],
                        row["t_s"],
                        row["seq"],
                        row["dial_id"],
                        row["target_deg"],
                        row["angle_deg"],
                        row["error_deg"],
                        row["speed_deg_s"],
                        row["torque_ma"],
                        row["foc_rate_hz"],
                        row["status_bits"],
                    ]
                )


def maybe_write_plot(output_path: Path, tests: list[dict[str, Any]]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required for --plot. Install it in the game environment.") from exc

    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    for test in tests:
        times = [float(row["t_s"]) for row in test["rows"]]
        target = [float(row["target_deg"]) for row in test["rows"]]
        angles = [float(row["angle_deg"]) for row in test["rows"]]
        errors = [float(row["error_deg"]) for row in test["rows"]]
        torques = [int(row["torque_ma"]) for row in test["rows"]]
        label = test["test_name"]
        axes[0].plot(times, angles, label=f"{label} angle")
        axes[0].plot(times, target, linestyle="--", alpha=0.7, label=f"{label} target")
        axes[1].plot(times, errors, label=label)
        axes[2].plot(times, torques, label=label)

    axes[0].set_ylabel("Angle (deg)")
    axes[1].set_ylabel("Error (deg)")
    axes[2].set_ylabel("Torque (mA)")
    axes[2].set_xlabel("Time (s)")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize=8)

    figure.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=150)
    plt.close(figure)


def print_row_preview(test: dict[str, Any], preview_rows: int) -> None:
    if preview_rows <= 0:
        return
    rows = test["rows"]
    print(f"\\n{test['test_name']} telemetry preview:")
    print("t_s,angle_deg,error_deg,speed_deg_s,torque_ma")
    if len(rows) <= preview_rows:
        selected = rows
    else:
        head_count = max(1, preview_rows // 2)
        tail_count = max(1, preview_rows - head_count)
        selected = rows[:head_count] + rows[-tail_count:]
    for row in selected:
        print(
            f"{row['t_s']:.4f},{row['angle_deg']:.3f},{row['error_deg']:.3f},"
            f"{row['speed_deg_s']:.3f},{int(row['torque_ma'])}"
        )
    if len(rows) > preview_rows:
        print("...")


def print_test_summary(test: dict[str, Any]) -> None:
    metrics = test["metrics"]
    print(f"\\n{test['test_name']}")
    print(f"  target_turns: {test['target_turns']}")
    for key in [
        "sample_count",
        "peak_abs_error_deg",
        "rms_error_deg",
        "overshoot_deg",
        "reach_90_time_s",
        "settle_time_s",
        "settle_error_deg",
        "remaining_movement_deg",
        "steady_state_mean_abs_error_deg",
        "final_error_deg",
        "peak_abs_speed_deg_s",
        "peak_abs_torque_ma",
    ]:
        print(f"  {key}: {metrics[key]}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run tracking-only step-response tests for one PID setting.")
    parser.add_argument("--port", default=None, help="Serial port to open. If omitted, auto-discovers the requested dial ID.")
    parser.add_argument("--device-id", type=int, default=0, help="Dial ID to auto-discover when --port is omitted.")
    parser.add_argument("--baud", type=int, default=230400, help="Serial baud rate")
    parser.add_argument("--tracking-kp", type=float, default=DEFAULT_TRACKING_KP, help="tracking_kp to apply")
    parser.add_argument("--tracking-kd", type=float, default=DEFAULT_TRACKING_KD, help="tracking_kd to apply")
    parser.add_argument("--tracking-max-torque", type=float, default=DEFAULT_TRACKING_MAX_TORQUE, help="tracking_max_torque to apply")
    parser.add_argument("--turns", default="0.5,5.0", help="Comma-separated step amplitudes in turns")
    parser.add_argument("--wide-bound-turns", type=float, default=DEFAULT_WIDE_BOUND_TURNS, help="Half-width of the very wide bounds used to isolate tracking")
    parser.add_argument("--small-step-capture-s", type=float, default=DEFAULT_SMALL_CAPTURE_S, help="Capture window for sub-1-turn tests")
    parser.add_argument("--large-step-capture-s", type=float, default=DEFAULT_LARGE_CAPTURE_S, help="Capture window for 1-turn-and-larger tests")
    parser.add_argument("--settle-movement-deg", "--settle-band-deg", dest="settle_movement_deg", type=float, default=5.0, help="Maximum remaining dial movement used to declare the response settled")
    parser.add_argument("--settle-speed-deg-s", type=float, default=15.0, help=argparse.SUPPRESS)
    parser.add_argument("--settle-hold-frames", type=int, default=3, help="Minimum trailing telemetry frames required after the settle point")
    parser.add_argument("--steady-state-samples", type=int, default=20, help="Number of tail samples used for steady-state mean abs error")
    parser.add_argument("--preview-rows", type=int, default=8, help="How many telemetry rows to print per test as a preview")
    parser.add_argument("--output-dir", default=str(Path(__file__).resolve().parent / "logs"), help="Directory for CSV and optional plot outputs")
    parser.add_argument("--plot", action="store_true", help="Also write a PNG plot for the captured tests")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    amplitudes_turns = parse_turns(args.turns)
    args.settle_movement_decideg = deg_to_decideg(args.settle_movement_deg)
    args.wide_bound_decideg = turns_to_decideg(args.wide_bound_turns)

    port = resolve_test_port(args.port, args.baud, args.device_id, allow_candidate_fallback=bool(args.port is None))
    run_id = datetime.now(timezone.utc).strftime("tracking_tune_%Y%m%d_%H%M%S")
    output_dir = Path(args.output_dir)
    csv_path = output_dir / f"{run_id}.csv"
    plot_path = output_dir / f"{run_id}.png"

    seq_state = {"value": 1000}
    tests: list[dict[str, Any]] = []
    with open_serial(port, args.baud) as ser:
        set_runtime_int(ser, seq_state, "telemetry_interval", TUNING_TELEMETRY_INTERVAL_MS)
        try:
            wait_for_telemetry(ser)
            apply_tracking_params(ser, seq_state, args)
            apply_feature_flags(ser, seq_state, tracking=True, bounds=False, kick=False)

            for amplitude_turns in amplitudes_turns:
                tests.append(run_tracking_test(ser, seq_state, args, amplitude_turns, 1))
                tests.append(run_tracking_test(ser, seq_state, args, amplitude_turns, -1))
        finally:
            set_runtime_int(ser, seq_state, "telemetry_interval", DEFAULT_TELEMETRY_INTERVAL_MS)
            apply_feature_flags(ser, seq_state, tracking=True, bounds=True, kick=True)

    write_csv(csv_path, run_id, tests)
    if args.plot:
        maybe_write_plot(plot_path, tests)

    print("Tracking step-response run complete")
    print(f"Port: {port}")
    print(f"tracking_kp={args.tracking_kp}, tracking_kd={args.tracking_kd}, tracking_max_torque={args.tracking_max_torque}")
    print(f"CSV log: {csv_path}")
    if args.plot:
        print(f"Plot: {plot_path}")
    print(
        f"Settle rule: remaining dial movement over the rest of the capture <= {args.settle_movement_deg} deg with at least {args.settle_hold_frames} trailing frames"
    )

    for test in tests:
        print_test_summary(test)
        print_row_preview(test, args.preview_rows)


if __name__ == "__main__":
    main()