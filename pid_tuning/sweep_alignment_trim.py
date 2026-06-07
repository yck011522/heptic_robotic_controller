import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import serial


ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))


def deg_to_decideg(value: float) -> int:
    return int(round(value * 10.0))


def turns_to_decideg(value: float) -> int:
    return int(round(value * 3600.0))


def parse_float_list(raw: str) -> list[float]:
    values = []
    for part in raw.split(","):
        token = part.strip()
        if token:
            values.append(float(token))
    if not values:
        raise ValueError("Expected at least one numeric value")
    return values


def normalize_argv(argv: list[str]) -> list[str]:
    normalized: list[str] = []
    index = 0
    while index < len(argv):
        token = argv[index]
        if token == "--trims-deg" and index + 1 < len(argv):
            value = argv[index + 1]
            if value.startswith("-"):
                normalized.append(f"{token}={value}")
                index += 2
                continue
        normalized.append(token)
        index += 1
    return normalized


def read_line_until(ser: serial.Serial, predicate, timeout: float) -> str:
    deadline = time.monotonic() + timeout
    buffer = ""
    while time.monotonic() < deadline:
        data = ser.read(ser.in_waiting or 1)
        if not data:
            continue
        buffer += data.decode(errors="ignore")
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if predicate(line):
                return line
    raise TimeoutError("Timed out waiting for serial response")


def send_command(ser: serial.Serial, command: str) -> None:
    ser.write((command + "\n").encode())


def send_and_wait(ser: serial.Serial, command: str, prefix: str, timeout: float = 1.0) -> str:
    ser.reset_input_buffer()
    send_command(ser, command)
    return read_line_until(ser, lambda line: line.startswith(prefix), timeout)


def next_seq(state: dict[str, int]) -> int:
    state["value"] += 1
    return state["value"]


def wait_until_ready(ser: serial.Serial, state: dict[str, int], timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        seq = next_seq(state)
        try:
            ack = send_and_wait(ser, f"E,{seq}", f"E,{seq}", timeout=0.5)
            if ack == f"E,{seq}":
                return
        except TimeoutError:
            continue
    raise TimeoutError(f"Device did not become ready in {timeout_s:.1f}s")


def set_runtime_float(ser: serial.Serial, state: dict[str, int], name: str, value: float) -> None:
    seq = next_seq(state)
    scaled = int(round(value * 1000.0))
    ack = send_and_wait(ser, f"S,{seq},{name},{scaled}", f"S,{seq}")
    if ack != f"S,{seq}":
        raise RuntimeError(f"Unexpected ack for {name}: {ack}")


def set_runtime_bool(ser: serial.Serial, state: dict[str, int], name: str, value: bool) -> None:
    seq = next_seq(state)
    ack = send_and_wait(ser, f"S,{seq},{name},{1 if value else 0}", f"S,{seq}")
    if ack != f"S,{seq}":
        raise RuntimeError(f"Unexpected ack for {name}: {ack}")


def set_runtime_int(ser: serial.Serial, state: dict[str, int], name: str, value: int) -> None:
    seq = next_seq(state)
    ack = send_and_wait(ser, f"S,{seq},{name},{value}", f"S,{seq}")
    if ack != f"S,{seq}":
        raise RuntimeError(f"Unexpected ack for {name}: {ack}")


def parse_alignment_response(line: str, expected_seq: int) -> dict[str, float]:
    parts = line.split(",")
    if len(parts) != 5 or parts[0] != "A" or int(parts[1]) != expected_seq:
        raise RuntimeError(f"Unexpected alignment response: {line}")
    return {
        "base_deg": int(parts[2]) / 10.0,
        "trim_deg": int(parts[3]) / 10.0,
        "total_deg": int(parts[4]) / 10.0,
    }


def query_alignment(ser: serial.Serial, state: dict[str, int]) -> dict[str, float]:
    seq = next_seq(state)
    response = send_and_wait(ser, f"A,{seq}", f"A,{seq}")
    return parse_alignment_response(response, seq)


def set_alignment_trim(ser: serial.Serial, state: dict[str, int], trim_deg: float) -> dict[str, float]:
    seq = next_seq(state)
    response = send_and_wait(ser, f"A,{seq},{deg_to_decideg(trim_deg)}", f"A,{seq}")
    return parse_alignment_response(response, seq)


def parse_telemetry(line: str) -> dict[str, int]:
    parts = line.split(",")
    return {
        "dial_id": int(parts[1]),
        "seq": int(parts[2]),
        "angle_decideg": int(parts[3]),
        "speed_decideg_s": int(parts[4]),
        "torque_ma": int(parts[5]),
        "foc_rate_hz": int(parts[6]),
        "status_bits": int(parts[7]),
    }


def wait_for_telemetry(ser: serial.Serial, timeout: float = 3.0) -> dict[str, int]:
    line = read_line_until(ser, lambda line: line.startswith("T,"), timeout)
    return parse_telemetry(line)


def reset_current_position_to_zero(ser: serial.Serial, state: dict[str, int]) -> None:
    seq = next_seq(state)
    ack = send_and_wait(ser, f"R,{seq},0", f"R,{seq}")
    if ack != f"R,{seq}":
        raise RuntimeError(f"Unexpected rebase ack: {ack}")
    wait_for_telemetry(ser)


def capture_window(ser: serial.Serial, target_decideg: int, duration_s: float) -> list[dict[str, float | int]]:
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
            row = parse_telemetry(line)
            row["t_s"] = round(time.monotonic() - start, 4)
            row["error_decideg"] = row["angle_decideg"] - target_decideg
            rows.append(row)
    return rows


def command_and_capture(
    ser: serial.Serial,
    state: dict[str, int],
    target_decideg: int,
    min_decideg: int,
    max_decideg: int,
    duration_s: float,
) -> list[dict[str, float | int]]:
    seq = next_seq(state)
    ser.reset_input_buffer()
    send_command(ser, f"C,{seq},{target_decideg},{min_decideg},{max_decideg}")
    rows = capture_window(ser, target_decideg, duration_s)
    response_rows = [row for row in rows if row["seq"] == seq]
    if not response_rows:
        raise RuntimeError(f"No telemetry for seq {seq}")
    return response_rows


def mean_abs_error_deg(rows: list[dict[str, float | int]]) -> float:
    return round(sum(abs(int(row["error_decideg"])) for row in rows) / len(rows) / 10.0, 3)


def find_settle_metrics(
    rows: list[dict[str, float | int]],
    settle_speed_deg_s: float,
    settle_frames: int,
) -> tuple[float | None, float | None, float | None]:
    if len(rows) < settle_frames:
        return None, None, None
    settle_speed_decideg_s = deg_to_decideg(settle_speed_deg_s)
    settle_index = None
    remaining_movement_deg = None
    for index in range(len(rows) - settle_frames + 1):
        tail_rows = rows[index:]
        tail_speeds = [abs(int(row["speed_decideg_s"])) for row in tail_rows]
        if max(tail_speeds) <= settle_speed_decideg_s:
            settle_index = index
            tail_angles = [int(row["angle_decideg"]) for row in tail_rows]
            remaining_movement_decideg = max(tail_angles) - min(tail_angles)
            remaining_movement_deg = round(remaining_movement_decideg / 10.0, 3)
            break
    if settle_index is None:
        return None, None, None
    tail_rows = rows[settle_index:]
    settle_error_deg = mean_abs_error_deg(tail_rows)
    return round(float(rows[settle_index]["t_s"]), 4), settle_error_deg, remaining_movement_deg


def metric_or_penalty(value: float | None, penalty: float) -> float:
    if value is None:
        return round(penalty, 3)
    return round(float(value), 3)


def effective_settle_error_deg(result: dict[str, float | None]) -> float:
    if result["settle_error_deg"] is not None:
        return round(float(result["settle_error_deg"]), 3)
    return round(float(result["steady_state_mean_abs_error_deg"]), 3)


def summarize(rows: list[dict[str, float | int]], target_decideg: int, args: argparse.Namespace) -> dict[str, float | None]:
    settle_time_s, settle_error_deg, remaining_movement_deg = find_settle_metrics(rows, args.settle_speed_deg_s, args.settle_frames)
    tail = rows[-min(len(rows), args.tail_samples):]
    final_error_deg = (int(rows[-1]["angle_decideg"]) - target_decideg) / 10.0
    return {
        "final_error_deg": round(final_error_deg, 3),
        "settle_time_s": settle_time_s,
        "settle_error_deg": settle_error_deg,
        "remaining_movement_deg": remaining_movement_deg,
        "steady_state_mean_abs_error_deg": mean_abs_error_deg(tail),
        "peak_abs_speed_deg_s": round(max(abs(int(row["speed_decideg_s"])) for row in rows) / 10.0, 3),
    }


def run_test(ser: serial.Serial, state: dict[str, int], direction: int, args: argparse.Namespace) -> dict[str, float | None]:
    target_decideg = turns_to_decideg(args.target_turn) * direction
    reset_current_position_to_zero(ser, state)
    rows = command_and_capture(
        ser,
        state,
        target_decideg,
        -turns_to_decideg(args.wide_bound_turns),
        turns_to_decideg(args.wide_bound_turns),
        args.capture_s,
    )
    return summarize(rows, target_decideg, args)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Sweep runtime electrical alignment trim values using the 0.5-turn tracking symmetry test.")
    parser.add_argument("--port", default="COM4", help="Serial port to open")
    parser.add_argument("--baud", type=int, default=230400, help="Serial baud rate")
    parser.add_argument("--tracking-kp", type=float, default=8.0, help="tracking_kp to apply")
    parser.add_argument("--tracking-kd", type=float, default=0.002, help="tracking_kd to apply")
    parser.add_argument("--tracking-max-torque", type=float, default=8.0, help="tracking_max_torque to apply")
    parser.add_argument("--trims-deg", default="-6,-4,-2,0,2,4,6", help="Comma-separated electrical alignment trim values in degrees")
    parser.add_argument("--target-turn", type=float, default=0.5, help="Tracking step amplitude in turns")
    parser.add_argument("--wide-bound-turns", type=float, default=20.0, help="Wide bounds half-width in turns")
    parser.add_argument("--capture-s", type=float, default=2.5, help="Telemetry capture window per test")
    parser.add_argument("--settle-speed-deg-s", type=float, default=10.0, help="Maximum dial speed used to declare the response settled")
    parser.add_argument("--settle-frames", type=int, default=1, help="Minimum trailing telemetry frames required after the settle point")
    parser.add_argument("--tail-samples", type=int, default=5, help="Tail samples used for steady-state error")
    parser.add_argument("--serial-settle-s", type=float, default=0.5, help="Wait after opening the serial port before pinging")
    parser.add_argument("--ready-timeout-s", type=float, default=9.0, help="Maximum readiness ping window")
    parser.add_argument("--output-dir", default=str(Path(__file__).resolve().parent / "logs"), help="Directory where sweep JSON is written")
    return parser


def main() -> None:
    args = build_parser().parse_args(normalize_argv(sys.argv[1:]))
    trims_deg = parse_float_list(args.trims_deg)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    run_id = datetime.now(timezone.utc).strftime("alignment_sweep_%Y%m%d_%H%M%S")
    output_path = output_dir / f"{run_id}.json"

    summary = {
        "run_id": run_id,
        "started_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "port": args.port,
        "tracking_kp": args.tracking_kp,
        "tracking_kd": args.tracking_kd,
        "tracking_max_torque": args.tracking_max_torque,
        "target_turn": args.target_turn,
        "trims_deg": trims_deg,
    }

    state = {"value": 4000}
    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        time.sleep(args.serial_settle_s)
        wait_until_ready(ser, state, args.ready_timeout_s)
        summary["alignment_before"] = query_alignment(ser, state)
        print("alignment_before", json.dumps(summary["alignment_before"]))

        set_runtime_int(ser, state, "telemetry_interval", 5)
        try:
            wait_for_telemetry(ser)
            set_runtime_float(ser, state, "tracking_kp", args.tracking_kp)
            set_runtime_float(ser, state, "tracking_kd", args.tracking_kd)
            set_runtime_float(ser, state, "tracking_max_torque", args.tracking_max_torque)
            set_runtime_bool(ser, state, "enable_tracking", True)
            set_runtime_bool(ser, state, "enable_bounds_restoration", False)
            set_runtime_bool(ser, state, "enable_oob_kick", False)

            results = []
            for trim_deg in trims_deg:
                alignment = set_alignment_trim(ser, state, trim_deg)
                pos = run_test(ser, state, 1, args)
                neg = run_test(ser, state, -1, args)
                final_error_bias_deg = round(float(pos["final_error_deg"]) + float(neg["final_error_deg"]), 3)
                final_error_symmetry_gap_deg = round(abs(abs(float(pos["final_error_deg"])) - abs(float(neg["final_error_deg"]))), 3)
                avg_abs_final_error_deg = round((abs(float(pos["final_error_deg"])) + abs(float(neg["final_error_deg"]))) / 2.0, 3)

                avg_settle_time_s = round(
                    (
                        metric_or_penalty(pos["settle_time_s"], args.capture_s)
                        + metric_or_penalty(neg["settle_time_s"], args.capture_s)
                    )
                    / 2.0,
                    3,
                )
                settle_time_symmetry_gap_s = round(
                    abs(
                        metric_or_penalty(pos["settle_time_s"], args.capture_s)
                        - metric_or_penalty(neg["settle_time_s"], args.capture_s)
                    ),
                    3,
                )
                avg_settle_error_deg = round((effective_settle_error_deg(pos) + effective_settle_error_deg(neg)) / 2.0, 3)
                settle_error_symmetry_gap_deg = round(abs(effective_settle_error_deg(pos) - effective_settle_error_deg(neg)), 3)
                unsettled_count = int(pos["settle_time_s"] is None) + int(neg["settle_time_s"] is None)
                score = round(
                    avg_settle_time_s
                    + settle_time_symmetry_gap_s
                    + avg_settle_error_deg
                    + settle_error_symmetry_gap_deg,
                    3,
                )
                record = {
                    "trim_deg": trim_deg,
                    "alignment": alignment,
                    "pos": pos,
                    "neg": neg,
                    "final_error_bias_deg": final_error_bias_deg,
                    "avg_abs_final_error_deg": avg_abs_final_error_deg,
                    "final_error_symmetry_gap_deg": final_error_symmetry_gap_deg,
                    "avg_settle_time_s": avg_settle_time_s,
                    "settle_time_symmetry_gap_s": settle_time_symmetry_gap_s,
                    "avg_settle_error_deg": avg_settle_error_deg,
                    "settle_error_symmetry_gap_deg": settle_error_symmetry_gap_deg,
                    "unsettled_count": unsettled_count,
                    "score": score,
                }
                results.append(record)
                print(
                    f"trim {trim_deg:+.1f} deg | settle_time avg {avg_settle_time_s:.3f}s gap {settle_time_symmetry_gap_s:.3f}s "
                    f"| settle_error avg {avg_settle_error_deg:.3f}deg gap {settle_error_symmetry_gap_deg:.3f}deg "
                    f"| unsettled {unsettled_count} | score {score:.3f}"
                )

            summary["results"] = results
            summary["best"] = min(results, key=lambda item: item["score"])
            print("best", json.dumps(summary["best"]))
            summary["alignment_restored"] = set_alignment_trim(ser, state, 0.0)
            print("alignment_restored", json.dumps(summary["alignment_restored"]))
        finally:
            set_runtime_int(ser, state, "telemetry_interval", 20)
            set_runtime_bool(ser, state, "enable_bounds_restoration", True)
            set_runtime_bool(ser, state, "enable_oob_kick", True)

    summary["finished_utc"] = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    output_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"JSON log: {output_path}")


if __name__ == "__main__":
    main()