import argparse

from common import (
    add_common_arguments,
    build_summary_rows,
    open_serial,
    resolve_test_port,
    run_platformio,
    send_and_wait,
    wait_for_telemetry,
    write_test_result,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Upload firmware and run a basic serial smoke test.")
    add_common_arguments(parser, include_platformio=True)
    parser.add_argument("--skip-upload", action="store_true", help="Skip PlatformIO upload and only run the smoke test")
    args = parser.parse_args()

    port = resolve_test_port(
        args.port,
        args.baud,
        args.device_id,
        allow_candidate_fallback=not args.skip_upload,
    )

    if not args.skip_upload:
        run_platformio(args.platformio, ["run", "--target", "upload", "--upload-port", port])

    with open_serial(port, args.baud) as ser:
        telemetry = wait_for_telemetry(ser)
        version = send_and_wait(ser, "V,1", "V,1,")
        identity = send_and_wait(ser, "I,2", "I,2,")
        echo = send_and_wait(ser, "E,3", "E,3")

    result_path = write_test_result(
        __file__,
        args.result_format,
        args.result_dir,
        build_summary_rows(
            port,
            [
                ("skip_upload", args.skip_upload),
                ("version_response", version),
                ("identity_response", identity),
                ("echo_response", echo),
            ],
        ),
        sections=[("telemetry", [telemetry])],
    )

    print("Upload/smoke test passed")
    print(f"Port: {port}")
    print(f"Telemetry: {telemetry}")
    print(f"Version: {version}")
    print(f"Identity: {identity}")
    print(f"Echo: {echo}")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()