import argparse
import time

from common import (
    add_common_arguments,
    build_summary_rows,
    open_serial,
    parse_telemetry,
    resolve_test_port,
    send_command,
    wait_for_telemetry,
    write_test_result,
)


OOB_STATUS_BIT = 1 << 5


def ask_yes_no(prompt: str) -> bool:
    while True:
        answer = input(f"{prompt} [y/n]: ").strip().lower()
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False
        print("Please answer y or n.")


def capture_telemetry_window(ser, duration_s: float) -> list[dict[str, int]]:
    rows = []
    deadline = time.monotonic() + duration_s
    buffer = ""

    while time.monotonic() < deadline:
        data = ser.read(ser.in_waiting or 1)
        if not data:
            continue

        buffer += data.decode(errors="ignore")
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if line.startswith("T,"):
                rows.append(parse_telemetry(line))

    return rows


def summarize_stage(rows: list[dict[str, int]]) -> dict[str, int | bool]:
    if not rows:
        return {
            "samples": 0,
            "oob_seen": False,
            "max_abs_torque_ma": 0,
            "max_abs_speed_decideg_s": 0,
        }

    return {
        "samples": len(rows),
        "oob_seen": any((row["status_bits"] & OOB_STATUS_BIT) != 0 for row in rows),
        "max_abs_torque_ma": max(abs(row["torque_ma"]) for row in rows),
        "max_abs_speed_decideg_s": max(abs(row["speed_decideg_s"]) for row in rows),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Manual acceptance script for subjective wall feel and OOB kick behavior."
    )
    add_common_arguments(parser)
    parser.add_argument("--wall-half-width", type=int, default=120, help="Half-width of the narrow bounds window in decidegrees")
    parser.add_argument("--capture-duration", type=float, default=3.0, help="Telemetry capture duration for each manual stage")
    args = parser.parse_args()

    port = resolve_test_port(args.port, args.baud, args.device_id)
    sample_rows: list[dict[str, int | str | bool]] = []

    with open_serial(port, args.baud) as ser:
        baseline = wait_for_telemetry(ser)
        center_angle = baseline["angle_decideg"]
        send_command(
            ser,
            f"C,600,{center_angle},{center_angle - args.wall_half_width},{center_angle + args.wall_half_width}",
        )
        wait_for_telemetry(ser)

        print("Manual bounds-feel acceptance test")
        print("A narrow bounds window has been applied around the current angle.")

        input("Push slowly into the upper wall and slightly beyond it, then press Enter to start telemetry capture...")
        upper_rows = capture_telemetry_window(ser, args.capture_duration)
        upper_summary = summarize_stage(upper_rows)
        upper_ok = ask_yes_no("Did the upper wall feel correct and return the dial cleanly?")

        input("Push slowly into the lower wall and slightly beyond it, then press Enter to start telemetry capture...")
        lower_rows = capture_telemetry_window(ser, args.capture_duration)
        lower_summary = summarize_stage(lower_rows)
        lower_ok = ask_yes_no("Did the lower wall feel correct and return the dial cleanly?")

        overall_ok = ask_yes_no("Was the OOB kick / wall behavior acceptable overall?")

    sample_rows.append({"stage": "upper_wall", "operator_ok": upper_ok, **upper_summary})
    sample_rows.append({"stage": "lower_wall", "operator_ok": lower_ok, **lower_summary})
    passed = upper_ok and lower_ok and overall_ok

    result_path = write_test_result(
        __file__,
        args.result_format,
        args.result_dir,
        build_summary_rows(
            port,
            [
                ("wall_half_width", args.wall_half_width),
                ("capture_duration", args.capture_duration),
                ("upper_oob_seen", upper_summary["oob_seen"]),
                ("lower_oob_seen", lower_summary["oob_seen"]),
                ("passed", passed),
            ],
        ),
        sections=[("manual_bounds", sample_rows)],
    )

    print("Manual bounds-feel acceptance complete")
    print(f"Port: {port}")
    print(f"Passed: {passed}")
    print(f"Upper stage summary: {upper_summary}")
    print(f"Lower stage summary: {lower_summary}")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()