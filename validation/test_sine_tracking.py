import argparse
import math
import threading
import time
from statistics import mean

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
        description="Send a sine-wave trajectory and measure controller FOC rate plus host-loop timing jitter."
    )
    add_common_arguments(parser)
    parser.add_argument("--duration", type=float, default=20.0, help="Test duration in seconds")
    parser.add_argument("--control-hz", type=float, default=50.0, help="Host-side command rate")
    parser.add_argument("--period", type=float, default=5.0, help="Seconds per sine cycle")
    parser.add_argument("--min-angle", type=int, default=-10800, help="Minimum target angle in decidegrees")
    parser.add_argument("--max-angle", type=int, default=10800, help="Maximum target angle in decidegrees")
    parser.add_argument("--amplitude-scale", type=float, default=0.5, help="Fraction of the full range used as sine amplitude")
    args = parser.parse_args()

    port = resolve_test_port(args.port, args.baud, args.device_id)
    interval = 1.0 / args.control_hz
    amplitude = (args.max_angle - args.min_angle) / 2 * args.amplitude_scale
    center = (args.max_angle + args.min_angle) / 2

    fps_values: list[int] = []
    host_loop_intervals: list[float] = []
    command_rows: list[dict[str, float | int]] = []
    running = True
    lock = threading.Lock()

    with open_serial(port, args.baud) as ser:
        wait_for_telemetry(ser)
        ser.timeout = 0

        def handle_line(line: str) -> None:
            if not line.startswith("T,"):
                return

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
        last_tick_time = start_time
        seq = 0

        while time.perf_counter() - start_time < args.duration:
            now = time.perf_counter()
            if now >= next_tick:
                host_interval_ms = (now - last_tick_time) * 1000.0
                host_loop_intervals.append(host_interval_ms)
                last_tick_time = now

                seq += 1
                elapsed = now - start_time
                phase = 2.0 * math.pi * (elapsed / args.period)
                target_decideg = int(center + amplitude * math.sin(phase))
                ser.write(f"C,{seq},{target_decideg},{args.min_angle},{args.max_angle}\n".encode())
                command_rows.append(
                    {
                        "seq": seq,
                        "elapsed_s": elapsed,
                        "target_decideg": target_decideg,
                        "host_interval_ms": host_interval_ms,
                    }
                )
                next_tick += interval
            else:
                time.sleep(0.0005)

        running = False
        reader.join()

    if not host_loop_intervals:
        raise RuntimeError("No host loop timing data received.")

    summary_rows = build_summary_rows(
        port,
        [
            ("duration_s", args.duration),
            ("control_hz", args.control_hz),
            ("period_s", args.period),
            ("min_angle_decideg", args.min_angle),
            ("max_angle_decideg", args.max_angle),
            ("amplitude_scale", args.amplitude_scale),
            ("command_count", len(command_rows)),
            ("host_loop_mean_interval_ms", mean(host_loop_intervals)),
            ("host_loop_p95_interval_ms", calculate_quantile(host_loop_intervals, 0.95)),
            ("host_loop_max_interval_ms", max(host_loop_intervals)),
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
        sections=[("command_samples", command_rows)],
    )

    print("Sine tracking test passed")
    print(f"Port: {port}")
    if fps_values:
        print(f"Controller FPS avg: {mean(fps_values):.1f}")
        print(f"Controller FPS min: {min(fps_values)}")
        print(f"Controller FPS max: {max(fps_values)}")
    print(f"Host loop mean interval: {mean(host_loop_intervals):.2f} ms")
    print(f"Host loop P95 interval: {calculate_quantile(host_loop_intervals, 0.95):.2f} ms")
    print(f"Host loop max interval: {max(host_loop_intervals):.2f} ms")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()