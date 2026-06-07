import argparse
import time

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
    parser = argparse.ArgumentParser(description="Exercise the basic strict-P5 protocol surface.")
    add_common_arguments(parser)
    args = parser.parse_args()
    port = resolve_test_port(args.port, args.baud, args.device_id)

    with open_serial(port, args.baud) as ser:
        version = send_and_wait(ser, "V,10", "V,10,")
        identity = send_and_wait(ser, "I,11", "I,11,")
        echo = send_and_wait(ser, "E,12", "E,12")
        send_and_wait(ser, "S,13,telemetry_interval,20", "S,13")

        ser.reset_input_buffer()
        ser.write(b"C,14,0,-3600,3600\n")
        telemetry = parse_telemetry(wait_for_prefix(ser, "T,"))

        ser.reset_input_buffer()
        ser.write(b"C,15,0,3600,-3600\n")
        time.sleep(0.1)
        telemetry_after_invalid = parse_telemetry(wait_for_prefix(ser, "T,"))

    assert_true(version.endswith("0.3.0"), f"Unexpected firmware version: {version}")
    assert_true(identity.startswith("I,11,"), f"Unexpected identity response: {identity}")
    assert_true(echo == "E,12", f"Unexpected echo response: {echo}")
    assert_true(telemetry["seq"] == 14, f"Expected seq echo 14, got {telemetry['seq']}")
    assert_true(telemetry_after_invalid["seq"] == 14, "Invalid control frame should not advance last processed seq")

    result_path = write_test_result(
        __file__,
        args.result_format,
        args.result_dir,
        build_summary_rows(
            port,
            [
                ("version_response", version),
                ("identity_response", identity),
                ("echo_response", echo),
                ("valid_control_seq", telemetry["seq"]),
                ("invalid_control_seq", telemetry_after_invalid["seq"]),
            ],
        ),
        sections=[
            (
                "telemetry",
                [
                    {"label": "valid_control", **telemetry},
                    {"label": "after_invalid_control", **telemetry_after_invalid},
                ],
            )
        ],
    )

    print("Protocol smoke test passed")
    print(f"Port: {port}")
    print(f"Version: {version}")
    print(f"Identity: {identity}")
    print(f"Telemetry: {telemetry}")
    print(f"Telemetry after invalid control: {telemetry_after_invalid}")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()