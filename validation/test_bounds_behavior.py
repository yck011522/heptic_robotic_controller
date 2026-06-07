import argparse
import time

from common import (
    add_common_arguments,
    assert_true,
    build_summary_rows,
    open_serial,
    parse_telemetry,
    resolve_test_port,
    send_command,
    wait_for_prefix,
    wait_for_telemetry,
    write_test_result,
)


OOB_STATUS_BIT = 1 << 5
FAULT_STATUS_BIT = 1 << 6


def wait_for_telemetry_matching(ser, predicate, timeout: float = 2.0) -> dict[str, int]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = max(0.05, deadline - time.monotonic())
        telemetry = parse_telemetry(wait_for_prefix(ser, "T,", remaining))
        if predicate(telemetry):
            return telemetry
    raise TimeoutError("Timed out waiting for matching telemetry")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Verify invalid bounds faulting plus out-of-bounds status behavior and torque response."
    )
    add_common_arguments(parser)
    parser.add_argument("--inside-margin", type=int, default=60, help="Half-width of the in-bounds window in decidegrees")
    parser.add_argument("--oob-offset", type=int, default=220, help="Distance from the current position to place the out-of-bounds window")
    parser.add_argument("--oob-window", type=int, default=120, help="Width of the out-of-bounds test window in decidegrees")
    parser.add_argument("--min-torque-delta-ma", type=int, default=80, help="Minimum torque change expected when the dial is forced out of bounds")
    args = parser.parse_args()

    port = resolve_test_port(args.port, args.baud, args.device_id)
    sample_rows: list[dict[str, int | str]] = []

    with open_serial(port, args.baud) as ser:
        baseline = wait_for_telemetry(ser)
        current_angle = baseline["angle_decideg"]

        inside_seq = 300
        send_command(
            ser,
            f"C,{inside_seq},{current_angle},{current_angle - args.inside_margin},{current_angle + args.inside_margin}",
        )
        inside = wait_for_telemetry_matching(ser, lambda telemetry: telemetry["seq"] == inside_seq)
        sample_rows.append({"label": "inside", **inside})
        assert_true((inside["status_bits"] & OOB_STATUS_BIT) == 0, f"Dial unexpectedly reported out-of-bounds inside the guard window: {inside}")

        invalid_seq = inside_seq + 1
        send_command(ser, f"C,{invalid_seq},{current_angle},{current_angle + 100},{current_angle - 100}")
        invalid_fault = wait_for_telemetry_matching(
            ser,
            lambda telemetry: telemetry["seq"] == inside_seq and (telemetry["status_bits"] & FAULT_STATUS_BIT) != 0,
        )
        sample_rows.append({"label": "invalid_bounds_fault", **invalid_fault})

        upper_seq = inside_seq + 2
        upper_min = current_angle + args.oob_offset
        upper_max = upper_min + args.oob_window
        upper_target = (upper_min + upper_max) // 2
        send_command(ser, f"C,{upper_seq},{upper_target},{upper_min},{upper_max}")
        upper_oob = wait_for_telemetry_matching(
            ser,
            lambda telemetry: telemetry["seq"] == upper_seq and (telemetry["status_bits"] & OOB_STATUS_BIT) != 0,
        )
        sample_rows.append({"label": "upper_oob", **upper_oob})

        lower_seq = inside_seq + 3
        lower_max = current_angle - args.oob_offset
        lower_min = lower_max - args.oob_window
        lower_target = (lower_min + lower_max) // 2
        send_command(ser, f"C,{lower_seq},{lower_target},{lower_min},{lower_max}")
        lower_oob = wait_for_telemetry_matching(
            ser,
            lambda telemetry: telemetry["seq"] == lower_seq and (telemetry["status_bits"] & OOB_STATUS_BIT) != 0,
        )
        sample_rows.append({"label": "lower_oob", **lower_oob})

        restore_seq = inside_seq + 4
        send_command(
            ser,
            f"C,{restore_seq},{current_angle},{current_angle - args.inside_margin},{current_angle + args.inside_margin}",
        )
        restored = wait_for_telemetry_matching(
            ser,
            lambda telemetry: telemetry["seq"] == restore_seq and (telemetry["status_bits"] & OOB_STATUS_BIT) == 0,
        )
        sample_rows.append({"label": "restored", **restored})

    assert_true((invalid_fault["status_bits"] & FAULT_STATUS_BIT) != 0, f"Invalid bounds did not assert the fault bit: {invalid_fault}")
    assert_true(abs(upper_oob["torque_ma"] - inside["torque_ma"]) >= args.min_torque_delta_ma, f"Upper out-of-bounds torque change was too small: inside={inside}, upper={upper_oob}")
    assert_true(abs(lower_oob["torque_ma"] - inside["torque_ma"]) >= args.min_torque_delta_ma, f"Lower out-of-bounds torque change was too small: inside={inside}, lower={lower_oob}")

    result_path = write_test_result(
        __file__,
        args.result_format,
        args.result_dir,
        build_summary_rows(
            port,
            [
                ("baseline_angle_decideg", current_angle),
                ("inside_margin", args.inside_margin),
                ("oob_offset", args.oob_offset),
                ("oob_window", args.oob_window),
                ("inside_torque_ma", inside["torque_ma"]),
                ("upper_oob_torque_ma", upper_oob["torque_ma"]),
                ("lower_oob_torque_ma", lower_oob["torque_ma"]),
            ],
        ),
        sections=[("bounds_samples", sample_rows)],
    )

    print("Bounds behavior test passed")
    print(f"Port: {port}")
    print(f"Inside telemetry: {inside}")
    print(f"Invalid-bounds fault telemetry: {invalid_fault}")
    print(f"Upper OOB telemetry: {upper_oob}")
    print(f"Lower OOB telemetry: {lower_oob}")
    print(f"Restored telemetry: {restored}")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()