import argparse
import threading
import time
from statistics import mean, median

from common import (
    add_common_arguments,
    build_summary_rows,
    calculate_quantile,
    open_serial,
    resolve_test_port,
    wait_for_telemetry,
    write_test_result,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Measure E-command round-trip time while streaming C commands and collecting telemetry."
    )
    add_common_arguments(parser)
    parser.add_argument("--duration", type=float, default=10.0, help="Test duration in seconds")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="C/E send rate")
    args = parser.parse_args()

    port = resolve_test_port(args.port, args.baud, args.device_id)
    interval = 1.0 / args.rate_hz
    send_times: dict[int, float] = {}
    rtts: list[float] = []
    fps_values: list[int] = []
    samples: list[dict[str, float | int]] = []
    lock = threading.Lock()
    running = True

    with open_serial(port, args.baud) as ser:
        wait_for_telemetry(ser)
        ser.timeout = 0

        def handle_line(line: str) -> None:
            now = time.perf_counter()
            if line.startswith("E,"):
                parts = line.split(",")
                if len(parts) < 2:
                    return
                try:
                    seq = int(parts[1])
                except ValueError:
                    return

                with lock:
                    if seq in send_times:
                        rtt_ms = (now - send_times.pop(seq)) * 1000.0
                        rtts.append(rtt_ms)
                        samples.append({"seq": seq, "echo_rtt_ms": rtt_ms})
            elif line.startswith("T,"):
                parts = line.split(",")
                if len(parts) < 8:
                    return
                try:
                    fps = int(parts[6])
                except ValueError:
                    return

                with lock:
                    fps_values.append(fps)

        def reader_thread() -> None:
            nonlocal running
            buffer = ""
            while running:
                data = ser.read(1024).decode(errors="ignore")
                if not data:
                    continue

                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    handle_line(line.strip())

        reader = threading.Thread(target=reader_thread)
        reader.start()

        start_time = time.perf_counter()
        next_tick = start_time
        seq = 0

        while time.perf_counter() - start_time < args.duration:
            now = time.perf_counter()
            if now >= next_tick:
                seq += 1
                ser.write(f"C,{seq},0,-3600,3600\n".encode())
                ser.write(f"E,{seq}\n".encode())
                with lock:
                    send_times[seq] = now
                next_tick += interval
            else:
                time.sleep(0.0005)

        running = False
        reader.join()

    if not rtts:
        raise RuntimeError("No echo RTT data received.")

    summary_rows = build_summary_rows(
        port,
        [
            ("duration_s", args.duration),
            ("rate_hz", args.rate_hz),
            ("echo_replies_received", len(rtts)),
            ("mean_rtt_ms", mean(rtts)),
            ("p50_rtt_ms", median(rtts)),
            ("p95_rtt_ms", calculate_quantile(rtts, 0.95)),
            ("p99_rtt_ms", calculate_quantile(rtts, 0.99)),
            ("max_rtt_ms", max(rtts)),
            ("controller_foc_rate_avg_hz", mean(fps_values) if fps_values else 0.0),
            ("controller_foc_rate_min_hz", min(fps_values) if fps_values else 0),
            ("controller_foc_rate_max_hz", max(fps_values) if fps_values else 0),
        ],
    )
    result_path = write_test_result(
        __file__,
        args.result_format,
        args.result_dir,
        summary_rows,
        sections=[("echo_rtt_samples", samples)],
    )

    print("Echo RTT test passed")
    print(f"Port: {port}")
    print(f"Echo replies received: {len(rtts)}")
    print(f"Mean RTT: {mean(rtts):.2f} ms")
    print(f"P50 RTT: {median(rtts):.2f} ms")
    print(f"P95 RTT: {calculate_quantile(rtts, 0.95):.2f} ms")
    print(f"P99 RTT: {calculate_quantile(rtts, 0.99):.2f} ms")
    print(f"Max RTT: {max(rtts):.2f} ms")
    if fps_values:
        print(f"Controller FPS avg: {mean(fps_values):.1f}")
        print(f"Controller FPS min: {min(fps_values)}")
        print(f"Controller FPS max: {max(fps_values)}")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()