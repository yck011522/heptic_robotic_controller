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
    write_test_result,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Repeat R rebasing many times and verify angle convergence stays bounded.")
    add_common_arguments(parser)
    parser.add_argument("--cycles", type=int, default=25, help="Number of R cycles to execute")
    parser.add_argument("--amplitude", type=int, default=1800, help="Absolute decidegree target used for alternating rebases")
    parser.add_argument("--angle-tolerance", type=int, default=80, help="Allowed angle error after each rebase")
    args = parser.parse_args()
    port = resolve_test_port(args.port, args.baud, args.device_id)
    cycle_rows = []

    with open_serial(port, args.baud) as ser:
        send_and_wait(ser, "C,100,0,-3600,3600", "T,")

        for cycle in range(args.cycles):
            target = args.amplitude if cycle % 2 == 0 else -args.amplitude
            seq = 101 + cycle
            ack = send_and_wait(ser, f"R,{seq},{target}", f"R,{seq}")
            telemetry = parse_telemetry(wait_for_prefix(ser, "T,"))

            assert_true(ack == f"R,{seq}", f"Unexpected ack at cycle {cycle}: {ack}")
            assert_true(
                abs(telemetry["angle_decideg"] - target) <= args.angle_tolerance,
                f"Angle drift exceeded tolerance at cycle {cycle}: target={target}, telemetry={telemetry}",
            )
            cycle_rows.append(
                {
                    "cycle": cycle,
                    "seq": seq,
                    "target_decideg": target,
                    "ack": ack,
                    "angle_decideg": telemetry["angle_decideg"],
                    "angle_error_decideg": telemetry["angle_decideg"] - target,
                    "speed_decideg_s": telemetry["speed_decideg_s"],
                    "status_bits": telemetry["status_bits"],
                }
            )

    max_abs_error = max(abs(row["angle_error_decideg"]) for row in cycle_rows)
    result_path = write_test_result(
        __file__,
        args.result_format,
        args.result_dir,
        build_summary_rows(
            port,
            [
                ("cycles", args.cycles),
                ("amplitude_decideg", args.amplitude),
                ("angle_tolerance", args.angle_tolerance),
                ("max_abs_error_decideg", max_abs_error),
            ],
        ),
        sections=[("rebase_cycles", cycle_rows)],
    )

    print("Repeated rebase test passed")
    print(f"Port: {port}")
    print(f"Cycles: {args.cycles}")
    print(f"Amplitude: {args.amplitude} decideg")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()