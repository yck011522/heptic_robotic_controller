import argparse

from common import (
    add_common_arguments,
    build_summary_rows,
    open_serial,
    parse_telemetry,
    resolve_test_port,
    send_and_wait,
    wait_for_telemetry,
    write_test_result,
)


def ask_yes_no(prompt: str) -> bool:
    while True:
        answer = input(f"{prompt} [y/n]: ").strip().lower()
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False
        print("Please answer y or n.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Manual acceptance script for verifying that R rebasing does not create a felt yank."
    )
    add_common_arguments(parser)
    parser.add_argument("--cycles", type=int, default=3, help="Number of manual rebase checks to perform")
    parser.add_argument("--offset-decideg", type=int, default=1800, help="Logical offset used for each rebase step")
    args = parser.parse_args()

    port = resolve_test_port(args.port, args.baud, args.device_id)
    cycle_rows: list[dict[str, int | str | bool]] = []

    with open_serial(port, args.baud) as ser:
        baseline = wait_for_telemetry(ser)
        baseline_angle = baseline["angle_decideg"]
        send_and_wait(ser, f"C,400,{baseline_angle},{baseline_angle - 3600},{baseline_angle + 3600}", "T,")

        print("Manual no-yank acceptance test")
        print("Hold the dial physically still during each prompted rebase.")

        for cycle in range(args.cycles):
            direction = 1 if cycle % 2 == 0 else -1
            requested_angle = baseline_angle + direction * args.offset_decideg
            input(f"Cycle {cycle + 1}/{args.cycles}: hold the dial still, then press Enter to issue R...")

            ack = send_and_wait(ser, f"R,{401 + cycle},{requested_angle}", f"R,{401 + cycle}")
            post_telemetry = parse_telemetry(send_and_wait(ser, f"C,{500 + cycle},{requested_angle},{requested_angle - 3600},{requested_angle + 3600}", "T,"))
            stayed_calm = ask_yes_no("Did the dial stay calm with no noticeable yank or jerk?")

            cycle_rows.append(
                {
                    "cycle": cycle + 1,
                    "requested_angle_decideg": requested_angle,
                    "ack": ack,
                    "felt_calm": stayed_calm,
                    **post_telemetry,
                }
            )

    passed = all(bool(row["felt_calm"]) for row in cycle_rows)
    result_path = write_test_result(
        __file__,
        args.result_format,
        args.result_dir,
        build_summary_rows(
            port,
            [
                ("cycles", args.cycles),
                ("offset_decideg", args.offset_decideg),
                ("passed", passed),
            ],
        ),
        sections=[("manual_cycles", cycle_rows)],
    )

    print("Manual no-yank acceptance complete")
    print(f"Port: {port}")
    print(f"Passed: {passed}")
    print(f"Result file: {result_path}")


if __name__ == "__main__":
    main()