import argparse

from common import (
    add_common_arguments,
    assert_true,
    build_summary_rows,
    open_serial,
    parse_telemetry,
    resolve_test_port,
    send_and_wait,
    wait_for_prefix,
    wait_for_telemetry,
    write_test_result,
)


DEFAULT_TELEMETRY_INTERVAL_MS = 20


def capture_telemetry_frames(ser, frame_count: int) -> list[dict[str, int]]:
    return [parse_telemetry(wait_for_prefix(ser, "T,")) for _ in range(frame_count)]


def max_abs(rows: list[dict[str, int]], key: str) -> int:
    return max(abs(row[key]) for row in rows) if rows else 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Repeat R rebasing with lower-than-last-C seq values and verify no post-rebase torque spike appears."
    )
    add_common_arguments(parser)
    parser.add_argument("--cycles", type=int, default=3, help="Number of rebases to test")
    parser.add_argument("--offset-decideg", type=int, default=1800, help="Logical offset used for alternating rebases")
    parser.add_argument(
        "--capture-frames",
        type=int,
        default=6,
        help="Telemetry frames to capture immediately after each R acknowledgement",
    )
    parser.add_argument(
        "--telemetry-interval-ms",
        type=int,
        default=5,
        help="Telemetry interval used during the capture window",
    )
    parser.add_argument(
        "--torque-spike-limit-ma",
        type=int,
        default=1200,
        help="Maximum allowed absolute torque in the post-R telemetry window",
    )
    parser.add_argument(
        "--final-speed-tolerance",
        type=int,
        default=250,
        help="Maximum allowed absolute speed in the final telemetry sample of the post-R window",
    )
    parser.add_argument(
        "--angle-tolerance",
        type=int,
        default=80,
        help="Allowed final angle error at the end of the post-R telemetry window",
    )
    args = parser.parse_args()

    port = resolve_test_port(args.port, args.baud, args.device_id)
    cycle_rows: list[dict[str, int | str]] = []

    with open_serial(port, args.baud) as ser:
        send_and_wait(ser, f"S,300,telemetry_interval,{args.telemetry_interval_ms}", "S,300")

        try:
            baseline = wait_for_telemetry(ser)
            baseline_angle = baseline["angle_decideg"]
            send_and_wait(
                ser,
                f"C,400,{baseline_angle},{baseline_angle - 3600},{baseline_angle + 3600}",
                "T,",
            )

            for cycle in range(args.cycles):
                direction = 1 if cycle % 2 == 0 else -1
                requested_angle = baseline_angle + direction * args.offset_decideg
                rebase_seq = 401 + cycle
                control_seq = 500 + cycle

                ack = send_and_wait(ser, f"R,{rebase_seq},{requested_angle}", f"R,{rebase_seq}")
                post_rebase_rows = capture_telemetry_frames(ser, args.capture_frames)
                tail = post_rebase_rows[-1]

                cycle_rows.append(
                    {
                        "cycle": cycle + 1,
                        "rebase_seq": rebase_seq,
                        "control_seq": control_seq,
                        "requested_angle_decideg": requested_angle,
                        "ack": ack,
                        "peak_abs_torque_ma": max_abs(post_rebase_rows, "torque_ma"),
                        "peak_abs_speed_decideg_s": max_abs(post_rebase_rows, "speed_decideg_s"),
                        "final_angle_decideg": tail["angle_decideg"],
                        "final_angle_error_decideg": tail["angle_decideg"] - requested_angle,
                        "final_speed_decideg_s": tail["speed_decideg_s"],
                        "final_torque_ma": tail["torque_ma"],
                        "telemetry_seq": tail["seq"],
                    }
                )

                send_and_wait(
                    ser,
                    f"C,{control_seq},{requested_angle},{requested_angle - 3600},{requested_angle + 3600}",
                    "T,",
                )
        finally:
            send_and_wait(ser, f"S,301,telemetry_interval,{DEFAULT_TELEMETRY_INTERVAL_MS}", "S,301")

    for row in cycle_rows:
        assert_true(row["ack"] == f"R,{row['rebase_seq']}", f"Unexpected R acknowledgement: {row}")
        assert_true(
            int(row["peak_abs_torque_ma"]) <= args.torque_spike_limit_ma,
            f"Rebase torque spike exceeded limit: {row}",
        )
        assert_true(
            abs(int(row["final_speed_decideg_s"])) <= args.final_speed_tolerance,
            f"Rebase final speed exceeded tolerance: {row}",
        )
        assert_true(
            abs(int(row["final_angle_error_decideg"])) <= args.angle_tolerance,
            f"Rebase angle drift exceeded tolerance: {row}",
        )

    max_torque = max(int(row["peak_abs_torque_ma"]) for row in cycle_rows)
    max_speed = max(int(row["peak_abs_speed_decideg_s"]) for row in cycle_rows)
    max_angle_error = max(abs(int(row["final_angle_error_decideg"])) for row in cycle_rows)

    result_path = write_test_result(
        __file__,
        args.result_format,
        args.result_dir,
        build_summary_rows(
            port,
            [
                ("cycles", args.cycles),
                ("offset_decideg", args.offset_decideg),
                ("capture_frames", args.capture_frames),
                ("telemetry_interval_ms", args.telemetry_interval_ms),
                ("torque_spike_limit_ma", args.torque_spike_limit_ma),
                ("final_speed_tolerance", args.final_speed_tolerance),
                ("max_peak_abs_torque_ma", max_torque),
                ("max_peak_abs_speed_decideg_s", max_speed),
                ("max_final_angle_error_decideg", max_angle_error),
            ],
        ),
        sections=[("rebase_windows", cycle_rows)],
    )

    print("Rebase no-yank test passed")
    print(f"Port: {port}")
    print(f"Cycles: {args.cycles}")
    print(f"Max peak torque: {max_torque} mA")
    print(f"Max peak speed: {max_speed} decideg/s")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()