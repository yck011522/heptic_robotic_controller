import argparse
import subprocess

from common import DEFAULT_PLATFORMIO, discover_target_devices, prompt_yes_no


def main() -> None:
    parser = argparse.ArgumentParser(description="Upload firmware to all connected controllers that match one exact VID/PID pair.")
    parser.add_argument("--platformio", default=str(DEFAULT_PLATFORMIO), help="Path to the PlatformIO executable")
    parser.add_argument("--environment", default="lolin32_lite", help="PlatformIO environment name")
    parser.add_argument("--vid", type=lambda value: int(value, 0), default=0x1A86, help="USB VID to target")
    parser.add_argument("--pid", type=lambda value: int(value, 0), default=0x7523, help="USB PID to target")
    parser.add_argument("--yes", action="store_true", help="Skip the upload confirmation prompt")
    args = parser.parse_args()

    devices = discover_target_devices(vid=args.vid, pid=args.pid)
    if not devices:
        print("No matching controllers found.")
        return

    print("Controllers selected for upload:")
    for device in devices:
        print(f"  {device['port']}: fw={device['fw_version']} dial_id={device['dial_id']}")

    if not args.yes and not prompt_yes_no("Upload the current firmware to all listed controllers?", default=False):
        print("Upload cancelled.")
        return

    for device in devices:
        port = device["port"]
        print(f"\nUploading to {port}...")
        subprocess.run(
            [
                args.platformio,
                "run",
                "--environment",
                args.environment,
                "--target",
                "upload",
                "--upload-port",
                port,
            ],
            check=True,
        )

    print("\nUpload complete.")


if __name__ == "__main__":
    main()