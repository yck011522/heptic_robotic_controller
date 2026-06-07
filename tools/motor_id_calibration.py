import argparse

from common import (
    BAUD,
    DEFAULT_MOTION_DETECT_WINDOW,
    DEFAULT_MOTION_THRESHOLD,
    assign_identity,
    close_monitors,
    detect_motion,
    discover_target_devices,
    open_monitors,
    prompt_yes_no,
)


def run_calibration(starting_dial_id=11, baud=BAUD, vid=None, pid=None, firmware_version=None, motion_window=DEFAULT_MOTION_DETECT_WINDOW, motion_threshold=DEFAULT_MOTION_THRESHOLD):
    print("=" * 60)
    print("  Dial ID Calibration Tool")
    print("=" * 60)
    print()
    print("Scanning for haptic controllers...")

    devices = discover_target_devices(vid=vid, pid=pid, baud=baud, required_fw_version=firmware_version)
    if not devices:
        print("No matching controllers found. Check USB connections and firmware version.")
        return

    monitors = open_monitors(devices, baud=baud)
    total_dials = len(monitors)
    dial_ids = list(range(starting_dial_id, starting_dial_id + total_dials))
    assignments = {monitor.port: None for monitor in monitors}
    identified_ports = set()

    print(f"Found {total_dials} controller(s).")
    print(f"Dial IDs to assign sequentially: {dial_ids}")

    for index, dial_id in enumerate(dial_ids, start=1):
        while True:
            print("-" * 60)
            input(f">>> Move dial #{dial_id} ({index}/{total_dials}) now, then press Enter... ")
            print(f"    Watching for motion ({motion_window}s)...")

            monitor, motion = detect_motion(monitors, duration=motion_window, threshold=motion_threshold)
            if monitor is None:
                print(f"    No motion detected (need >{motion_threshold} decideg). Try again.")
                continue

            if monitor.port in identified_ports:
                print(f"    Detected {monitor.port}, but it already has a proposed dial ID. Try a different dial.")
                continue

            print(f"    Detected: {monitor.port} (moved {motion} decideg = {motion / 10:.1f} deg)")
            assignments[monitor.port] = dial_id
            identified_ports.add(monitor.port)
            break

    print()
    print("=" * 60)
    print("  Calibration Results")
    print("=" * 60)
    for port, dial_id in assignments.items():
        print(f"  {port}: dial_id = #{dial_id}")

    if not prompt_yes_no("Write these dial IDs to controller flash?", default=False):
        print("Aborted. No changes written.")
        close_monitors(monitors)
        return

    close_monitors(monitors)
    print()
    for port, dial_id in assignments.items():
        confirmed = assign_identity(port, dial_id, baud=baud)
        print(f"  {port}: wrote {dial_id}, confirmed = {confirmed}")

    print("\nDone! Dial IDs are saved to flash and will persist across reboots.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Sequentially assign dial IDs by detecting which controller is moved.")
    parser.add_argument("--starting-dial-id", type=int, default=11, help="First dial ID to assign sequentially")
    parser.add_argument("--baud", type=int, default=BAUD, help="Serial baud rate")
    parser.add_argument("--vid", type=lambda value: int(value, 0), default=0x1A86, help="USB VID to target")
    parser.add_argument("--pid", type=lambda value: int(value, 0), default=0x7523, help="USB PID to target")
    parser.add_argument("--firmware-version", default=None, help="Optional firmware version filter")
    parser.add_argument("--motion-window", type=float, default=DEFAULT_MOTION_DETECT_WINDOW, help="Seconds to watch for dial movement after each prompt")
    parser.add_argument("--motion-threshold", type=int, default=DEFAULT_MOTION_THRESHOLD, help="Minimum decidegree range that counts as intentional movement")
    args = parser.parse_args()

    run_calibration(
        starting_dial_id=args.starting_dial_id,
        baud=args.baud,
        vid=args.vid,
        pid=args.pid,
        firmware_version=args.firmware_version,
        motion_window=args.motion_window,
        motion_threshold=args.motion_threshold,
    )


if __name__ == "__main__":
    main()