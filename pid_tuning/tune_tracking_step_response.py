import argparse
import csv
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from validation.common import parse_telemetry, open_serial, resolve_test_port, send_and_wait, send_command, wait_for_telemetry


DEFAULT_TELEMETRY_INTERVAL_MS = 5
RESTORE_TELEMETRY_INTERVAL_MS = 20


def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def deg_to_decideg(value: float) -> int:
    return int(round(value * 10.0))


def turns_to_decideg(value: float) -> int:
    return int(round(value * 3600.0))


def next_seq(seq_state: dict[str, int]) -> int:
    seq_state["value"] += 1
    return seq_state["value"]


def parse_turns(raw: str) -> list[float]:
    turns = []
    for part in raw.split(","):
        token = part.strip()
        if token:
            turns.append(float(token))
    if not turns:
        raise ValueError("Expected at least one turn amplitude")
    return turns


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


def apply_tracking_mode(ser, seq_state: dict[str, int]) -> None:
    set_runtime_bool(ser, seq_state, "enable_tracking", True)
    set_runtime_bool(ser, seq_state, "enable_bounds_restoration", False)
    set_runtime_bool(ser, seq_state, "enable_oob_kick", False)


def reset_current_position_to_zero(ser, seq_state: dict[str, int]) -> dict[str, int]:
    seq = next_seq(seq_state)
    ack = send_and_wait(ser, f"R,{seq},0", f"R,{seq}")
    if ack != f"R,{seq}":
        raise RuntimeError(f"Unexpected acknowledgement while rebasing current position: {ack}")
    return wait_for_telemetry(ser)


def capture_window(ser, duration_s: float) -> list[dict[str, float | int]]:
    rows: list[dict[str, float | int]] = []
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
            rows.append({"t_s": round(time.monotonic() - start, 4), **telemetry})

    return rows


def command_tracking_step(
    ser,
    seq_state: dict[str, int],
    target_decideg: int,
    wide_bound_decideg: int,
    capture_s: float,
) -> tuple[int, list[dict[str, float | int]]]:
    seq = next_seq(seq_state)
    ser.reset_input_buffer()
    send_command(ser, f"C,{seq},{target_decideg},{-wide_bound_decideg},{wide_bound_decideg}")
    rows = capture_window(ser, capture_s)
    response_rows = [row for row in rows if int(row["seq"]) == seq]
    if not response_rows:
        raise RuntimeError(f"No telemetry captured for command sequence {seq}")
    return seq, response_rows


def find_settle_index(
    rows: list[dict[str, float | int]],
    target_decideg: int,
    angle_tolerance_decideg: int,
    speed_tolerance_decideg_s: int,
    settle_frames: int,
) -> int | None:
    consecutive = 0
    start_index = 0
    for index, row in enumerate(rows):
        angle_error = abs(int(row["angle_decideg"]) - target_decideg)
        speed_error = abs(int(row["speed_decideg_s"]))
        if angle_error <= angle_tolerance_decideg and speed_error <= speed_tolerance_decideg_s:
            if consecutive == 0:
                start_index = index
            consecutive += 1
            if consecutive >= settle_frames:
                return start_index
        else:
            consecutive = 0
    return None


def compute_overshoot_decideg(rows: list[dict[str, float | int]], target_decideg: int) -> int:
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


def compute_rise_time_s(rows: list[dict[str, float | int]], start_angle_decideg: int, target_decideg: int) -> float | None:
    step = target_decideg - start_angle_decideg
    if step == 0:
        return 0.0
    threshold = start_angle_decideg + int(round(step * 0.9))
    for row in rows:
        angle = int(row["angle_decideg"])
        if step > 0 and angle >= threshold:
            return round(float(row["t_s"]), 4)
        if step < 0 and angle <= threshold:
            return round(float(row["t_s"]), 4)
    return None


def summarize_metrics(
    rows: list[dict[str, float | int]],
    target_decideg: int,
    angle_tolerance_decideg: int,
    speed_tolerance_decideg_s: int,
    settle_frames: int,
) -> dict[str, float | int | None]:
    if not rows:
        raise RuntimeError("No telemetry captured during tracking step response test")

    start_angle_decideg = int(rows[0]["angle_decideg"])
    settle_index = find_settle_index(rows, target_decideg, angle_tolerance_decideg, speed_tolerance_decideg_s, settle_frames)
    settle_row = rows[settle_index] if settle_index is not None else None

    return {
        "sample_count": len(rows),
        "start_angle_deg": round(start_angle_decideg / 10.0, 3),
        "target_deg": round(target_decideg / 10.0, 3),
        "step_size_deg": round((target_decideg - start_angle_decideg) / 10.0, 3),
        "rise_time_90_s": compute_rise_time_s(rows, start_angle_decideg, target_decideg),
        "settle_time_s": round(float(settle_row["t_s"]), 4) if settle_row is not None else None,
        "settle_distance_deg": round(abs(int(settle_row["angle_decideg"]) - target_decideg) / 10.0, 3) if settle_row is not None else None,
        "overshoot_deg": round(compute_overshoot_decideg(rows, target_decideg) / 10.0, 3),
        "peak_abs_error_deg": round(max(abs(int(row["angle_decideg"]) - target_decideg) for row in rows) / 10.0, 3),
        "peak_abs_speed_deg_s": round(max(abs(int(row["speed_decideg_s"])) for row in rows) / 10.0, 3),
        "peak_abs_torque_ma": max(abs(int(row["torque_ma"])) for row in rows),
        "final_angle_deg": round(int(rows[-1]["angle_decideg"]) / 10.0, 3),
        "final_error_deg": round((int(rows[-1]["angle_decideg"]) - target_decideg) / 10.0, 3),
        "final_speed_deg_s": round(int(rows[-1]["speed_decideg_s"]) / 10.0, 3),
    }


def build_logged_rows(rows: list[dict[str, float | int]], target_decideg: int) -> list[dict[str, float | int]]:
    logged_rows = []
    for row in rows:
        angle_decideg = int(row["angle_decideg"])
        logged_rows.append(
            {
                "t_s": float(row["t_s"]),
                "angle_deg": round(angle_decideg / 10.0, 3),
                "target_deg": round(target_decideg / 10.0, 3),
                "error_deg": round((target_decideg - angle_decideg) / 10.0, 3),
                "speed_deg_s": round(int(row["speed_decideg_s"]) / 10.0, 3),
                "torque_ma": int(row["torque_ma"]),
                "foc_rate_hz": int(row["foc_rate_hz"]),
                "status_bits": int(row["status_bits"]),
                "seq": int(row["seq"]),
            }
        )
    return logged_rows


def preview_rows(rows: list[dict[str, float | int]], preview_count: int) -> list[dict[str, float | int | str]]:
    if preview_count <= 0 or len(rows) <= preview_count * 2:
        return rows
    return rows[:preview_count] + [{"...": "..."}] + rows[-preview_count:]


def write_csv(path: Path, rows: list[dict[str, float | int]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def maybe_write_plot(path: Path, rows: list[dict[str, float | int]], test_name: str) -> bool:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    times = [float(row["t_s"]) for row in rows]
    angles = [float(row["angle_deg"]) for row in rows]
    targets = [float(row["target_deg"]) for row in rows]
    errors = [float(row["error_deg"]) for row in rows]
    speeds = [float(row["speed_deg_s"]) for row in rows]
    torques = [int(row["torque_ma"]) for row in rows]

    figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(times, angles, label="angle_deg")
    axes[0].plot(times, targets, linestyle="--", label="target_deg")
    axes[0].set_ylabel("Angle (deg)")
    axes[0].set_title(test_name)
    axes[0].legend()

    axes[1].plot(times, errors, label="error_deg", color="tab:red")
    axes[1].plot(times, speeds, label="speed_deg_s", color="tab:green")
    axes[1].set_ylabel("Error / Speed")
    axes[1].legend()

    axes[2].plot(times, torques, label="torque_ma", color="tab:purple")
    axes[2].set_ylabel("Torque (mA)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend()

    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return True


def run_tracking_test(ser, seq_state: dict[str, int], args: argparse.Namespace, turn_value: float) -> dict[str, Any]:
    target_decideg = turns_to_decideg(turn_value)
    capture_s = args.large_step_capture_s if abs(turn_value) >= 1.0 else args.small_step_capture_s
    baseline = reset_current_position_to_zero(ser, seq_state)
    apply_tracking_mode(ser, seq_state)
    command_seq, rows = command_tracking_step(
        ser,
        seq_state,
        target_decideg,
        turns_to_decideg(args.tracking_wide_bound_turns),
        capture_s,
    )
    metrics = summarize_metrics(
        rows,
        target_decideg,
        deg_to_decideg(args.target_tolerance_deg),
        deg_to_decideg(args.final_speed_tolerance_deg_s),
        args.settle_hold_frames,
    )
    return {
        "test_name": f"tracking_{str(turn_value).replace('-', 'neg').replace('.', 'p')}_turn",
        "command_seq": command_seq,
        "baseline": {
            "angle_deg": round(int(baseline["angle_decideg"]) / 10.0, 3),
            "speed_deg_s": round(int(baseline["speed_decideg_s"]) / 10.0, 3),
        },
        "turn_value": turn_value,
        "logged_rows": build_logged_rows(rows, target_decideg),
        "metrics": metrics,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run simple tracking step-response tests for one set of PID values.")
    parser.add_argument("--port", default=None, help="Serial port to open. If omitted, auto-discovers the requested dial ID.")
    parser.add_argument("--device-id", type=int, default=0, help="Dial ID to auto-discover when --port is omitted.")
    parser.add_argument("--baud", type=int, default=230400, help="Serial baud rate")
    parser.add_argument("--tracking-kp", type=float, required=True, help="tracking_kp value to test")
    parser.add_argument("--tracking-kd", type=float, required=True, help="tracking_kd value to test")
    parser.add_argument("--tracking-max-torque", type=float, required=True, help="tracking_max_torque value to test")
    parser.add_argument("--turns", default="0.5,5.0", help="Comma-separated step amplitudes in turns")
    parser.add_argument("--target-tolerance-deg", type=float, default=5.0, help="Angle tolerance used for settled detection")
    parser.add_argument("--final-speed-tolerance-deg-s", type=float, default=15.0, help="Speed tolerance used for settled detection")
    parser.add_argument("--settle-hold-frames", type=int, default=3, help="Consecutive in-band frames required for settled detection")
    parser.add_argument("--small-step-capture-s", type=float, default=2.5, help="Capture window for sub-turn tests")
    parser.add_argument("--large-step-capture-s", type=float, default=6.0, help="Capture window for multi-turn tests")
    parser.add_argument("--tracking-wide-bound-turns", type=float, default=20.0, help="Half-width of the wide bounds used to isolate tracking")
    parser.add_argument("--telemetry-interval-ms", type=int, default=DEFAULT_TELEMETRY_INTERVAL_MS, help="Telemetry interval while capturing the response")
    parser.add_argument("--output-dir", default=str(Path(__file__).resolve().parent / "logs"), help="Directory where CSV, summary JSON, and plots are written")
    parser.add_argument("--preview-rows", type=int, default=8, help="How many rows from the start and end of each test to print")
    parser.add_argument("--plot", action="store_true", help="Write PNG response plots when matplotlib is available")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    turn_values = parse_turns(args.turns)
    port = resolve_test_port(args.port, args.baud, args.device_id, allow_candidate_fallback=bool(args.port is None))

    run_id = datetime.now(timezone.utc).strftime("tracking_step_%Y%m%d_%H%M%S")
    run_dir = Path(args.output_dir) / run_id
    run_dir.mkdir(parents=True, exist_ok=True)

    summary: dict[str, Any] = {
        "run_id": run_id,
        "started_utc": utc_now(),
        "port": port,
        "tracking_params": {
            "tracking_kp": args.tracking_kp,
            "tracking_kd": args.tracking_kd,
            "tracking_max_torque": args.tracking_max_torque,
        },
        "turns": turn_values,
        "target_tolerance_deg": args.target_tolerance_deg,
        "final_speed_tolerance_deg_s": args.final_speed_tolerance_deg_s,
        "tests": [],
    }

    seq_state = {"value": 1000}
    with open_serial(port, args.baud) as ser:
        set_runtime_int(ser, seq_state, "telemetry_interval", args.telemetry_interval_ms)
        try:
            wait_for_telemetry(ser)
            apply_tracking_params(ser, seq_state, args)
            apply_tracking_mode(ser, seq_state)

            for turn_value in turn_values:
                test_result = run_tracking_test(ser, seq_state, args, turn_value)
                csv_path = run_dir / f"{test_result['test_name']}.csv"
                write_csv(csv_path, test_result["logged_rows"])

                plot_path = run_dir / f"{test_result['test_name']}.png"
                plot_written = bool(args.plot) and maybe_write_plot(plot_path, test_result["logged_rows"], test_result["test_name"])

                summary["tests"].append(
                    {
                        "test_name": test_result["test_name"],
                        "turn_value": test_result["turn_value"],
                        "baseline": test_result["baseline"],
                        "metrics": test_result["metrics"],
                        "csv_path": str(csv_path),
                        "plot_path": str(plot_path) if plot_written else None,
                    }
                )

                print(f"Test: {test_result['test_name']}")
                print(json.dumps(test_result["metrics"], indent=2))
                print("Logged row preview:")
                for row in preview_rows(test_result["logged_rows"], args.preview_rows):
                    print(json.dumps(row))
                print(f"CSV: {csv_path}")
                if args.plot:
                    if plot_written:
                        print(f"Plot: {plot_path}")
                    else:
                        print("Plot: skipped because matplotlib is not available")
                print()

            reset_current_position_to_zero(ser, seq_state)
            summary["finished_utc"] = utc_now()
        finally:
            set_runtime_int(ser, seq_state, "telemetry_interval", RESTORE_TELEMETRY_INTERVAL_MS)

    summary_path = run_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print("Tracking step-response run complete")
    print(f"Port: {port}")
    print(f"Summary: {summary_path}")


if __name__ == "__main__":
    main()