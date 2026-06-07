import argparse
import statistics
import time

from common import (
    add_common_arguments,
    assert_true,
    build_summary_rows,
    open_serial,
    parse_telemetry,
    resolve_test_port,
    write_test_result,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Stream C commands for a fixed duration and verify seq progress and telemetry continuity.")
    add_common_arguments(parser)
    parser.add_argument("--duration", type=float, default=10.0, help="Test duration in seconds")
    parser.add_argument("--rate-hz", type=float, default=100.0, help="Control send rate")
    parser.add_argument("--min-final-seq", type=int, default=20, help="Minimum expected final echoed seq")
    args = parser.parse_args()
    port = resolve_test_port(args.port, args.baud, args.device_id)

    interval = 1.0 / args.rate_hz
    seq = 0
    next_send_time = time.perf_counter()
    end_time = next_send_time + args.duration
    last_seq_seen = 0
    foc_rates = []
    telemetry_count = 0
    telemetry_rows = []
    buffer = ""

    with open_serial(port, args.baud) as ser:
        while time.perf_counter() < end_time:
            now = time.perf_counter()
            if now >= next_send_time:
                seq += 1
                ser.write(f"C,{seq},0,-3600,3600\n".encode())
                next_send_time += interval

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
                telemetry_count += 1
                last_seq_seen = max(last_seq_seen, telemetry["seq"])
                foc_rates.append(telemetry["foc_rate_hz"])
                telemetry_rows.append({"capture_index": telemetry_count, **telemetry})

    assert_true(telemetry_count > 0, "No telemetry received during streaming stability test")
    assert_true(last_seq_seen >= args.min_final_seq, f"Expected final seq >= {args.min_final_seq}, got {last_seq_seen}")

    result_path = write_test_result(
        __file__,
        args.result_format,
        args.result_dir,
        build_summary_rows(
            port,
            [
                ("duration_s", args.duration),
                ("rate_hz", args.rate_hz),
                ("telemetry_frames", telemetry_count),
                ("last_seq_seen", last_seq_seen),
                ("foc_rate_avg_hz", statistics.mean(foc_rates)),
                ("foc_rate_min_hz", min(foc_rates)),
                ("foc_rate_max_hz", max(foc_rates)),
            ],
        ),
        sections=[("telemetry", telemetry_rows)],
    )

    print("Streaming stability test passed")
    print(f"Port: {port}")
    print(f"Telemetry frames: {telemetry_count}")
    print(f"Last seq seen: {last_seq_seen}")
    print(f"FOC rate avg: {statistics.mean(foc_rates):.1f} Hz")
    print(f"FOC rate min/max: {min(foc_rates)} / {max(foc_rates)} Hz")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()