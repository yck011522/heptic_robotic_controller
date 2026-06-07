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
    parser = argparse.ArgumentParser(description="Verify the R command rebases the logical angle and returns an acknowledgement.")
    add_common_arguments(parser)
    parser.add_argument("--target-decideg", type=int, default=0, help="Logical angle to request with the R command")
    parser.add_argument("--angle-tolerance", type=int, default=50, help="Allowed telemetry angle error in decidegrees")
    parser.add_argument("--speed-tolerance", type=int, default=200, help="Allowed telemetry speed magnitude in decidegrees/s")
    args = parser.parse_args()
    port = resolve_test_port(args.port, args.baud, args.device_id)

    with open_serial(port, args.baud) as ser:
        initial_telemetry = parse_telemetry(send_and_wait(ser, "C,20,0,-3600,3600", "T,"))
        ack = send_and_wait(ser, f"R,21,{args.target_decideg}", "R,21")
        telemetry = parse_telemetry(wait_for_prefix(ser, "T,"))

    assert_true(ack == "R,21", f"Unexpected R acknowledgement: {ack}")
    assert_true(abs(telemetry["angle_decideg"] - args.target_decideg) <= args.angle_tolerance,
                f"Angle did not rebase close enough to target: {telemetry}")
    assert_true(abs(telemetry["speed_decideg_s"]) <= args.speed_tolerance,
                f"Speed remained too large after rebase: {telemetry}")

    result_path = write_test_result(
        __file__,
        args.result_format,
        args.result_dir,
        build_summary_rows(
            port,
            [
                ("target_decideg", args.target_decideg),
                ("angle_tolerance", args.angle_tolerance),
                ("speed_tolerance", args.speed_tolerance),
                ("ack", ack),
                ("post_rebase_angle_error", telemetry["angle_decideg"] - args.target_decideg),
            ],
        ),
        sections=[
            (
                "telemetry",
                [
                    {"label": "baseline", **initial_telemetry},
                    {"label": "post_rebase", **telemetry},
                ],
            )
        ],
    )

    print("Set Current Position test passed")
    print(f"Port: {port}")
    print(f"Ack: {ack}")
    print(f"Telemetry: {telemetry}")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()