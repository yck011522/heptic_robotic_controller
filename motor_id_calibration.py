# motor_id_calibration.py
# Interactive calibration tool that identifies physical motors by detecting movement.
#
# Connects to all haptic controllers, then prompts you to move each motor one at a
# time. Detects which device/motor was moved by monitoring telemetry angle changes,
# and writes the correct motor IDs to each controller's NVS flash.
#
# Telemetry format: T,motor_id_0,motor_id_1,seq,ang0,ang1,tor0,tor1,fps
#   ang0 is at index 4, ang1 is at index 5 (decidegrees)

import serial
import time
import threading
from device_discovery import (
    list_candidate_ports,
    probe_port,
    assign_identity,
    BAUD,
    DRAIN_DELAY,
)

# How long to monitor for motion after user says they're ready (seconds)
MOTION_DETECT_WINDOW = 2.0

# Minimum angle change (decidegrees) to count as intentional motion
# 100 decideg = 10 degrees
MOTION_THRESHOLD = 100


class DeviceMonitor:
    """Holds an open serial connection and tracks angle telemetry."""

    def __init__(self, port, baud=BAUD):
        self.port = port
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(DRAIN_DELAY)
        self.ser.reset_input_buffer()

        # Tracking: min/max angle seen during a detection window
        self.ang0_values = []
        self.ang1_values = []
        self.lock = threading.Lock()
        self._buf = ""

    def reset_tracking(self):
        """Clear recorded angles to start a new detection window."""
        with self.lock:
            self.ang0_values.clear()
            self.ang1_values.clear()

    def read_telemetry(self):
        """Non-blocking read. Parses any complete telemetry lines and records angles."""
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            return
        if not data:
            return
        self._buf += data.decode(errors="ignore")
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()
            if line.startswith("T,"):
                parts = line.split(",")
                if len(parts) >= 6:
                    try:
                        ang0 = int(parts[4])
                        ang1 = int(parts[5])
                        with self.lock:
                            self.ang0_values.append(ang0)
                            self.ang1_values.append(ang1)
                    except ValueError:
                        pass

    def get_motion(self):
        """Return (ang0_range, ang1_range) — how much each motor moved."""
        with self.lock:

            def value_range(vals):
                if len(vals) < 2:
                    return 0
                return max(vals) - min(vals)

            return value_range(self.ang0_values), value_range(self.ang1_values)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def open_all_controllers(baud=BAUD):
    """Discover and open serial connections to all haptic controllers.

    Returns a list of DeviceMonitor objects (caller must close them).
    """
    candidates = list_candidate_ports(filter_by_vid_pid=True)
    monitors = []
    for c in candidates:
        port = c["port"]
        # Quick probe to confirm it's our firmware (opens and closes the port)
        info = probe_port(port, baud)
        if info is not None:
            print(f"  Found controller on {port} (fw={info['fw_version']})")
            mon = DeviceMonitor(port, baud)
            monitors.append(mon)
    return monitors


def detect_motion(monitors, duration=MOTION_DETECT_WINDOW):
    """Monitor all devices for `duration` seconds and return which moved.

    Returns (best_monitor, motor_index, motion_amount) where motor_index is
    0 or 1 indicating which motor on that device was moved.
    Returns (None, None, 0) if no significant motion was detected.
    """
    # Reset all tracking
    for mon in monitors:
        mon.reset_tracking()

    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        for mon in monitors:
            mon.read_telemetry()
        time.sleep(0.005)  # ~200 Hz polling, plenty fast for 50 Hz telemetry

    # Find the device/motor with the most movement
    best_monitor = None
    best_motor = None
    best_motion = 0

    for mon in monitors:
        ang0_range, ang1_range = mon.get_motion()
        if ang0_range > best_motion:
            best_motion = ang0_range
            best_monitor = mon
            best_motor = 0
        if ang1_range > best_motion:
            best_motion = ang1_range
            best_monitor = mon
            best_motor = 1

    if best_motion < MOTION_THRESHOLD:
        return None, None, 0

    return best_monitor, best_motor, best_motion


def run_calibration(starting_motor_id=11):
    """Interactive calibration session.

    starting_motor_id: First motor ID to assign (e.g. 11 -> 11,12,13,...
                       or 21 -> 21,22,23,...). Default is 11.
    """

    print("=" * 60)
    print("  Motor ID Calibration Tool")
    print("=" * 60)
    print()
    print("Scanning for haptic controllers...")
    monitors = open_all_controllers()

    if not monitors:
        print("No controllers found. Check USB connections.")
        return

    total_motors = len(monitors) * 2
    motor_ids = list(range(starting_motor_id, starting_motor_id + total_motors))
    print(f"\nFound {len(monitors)} controller(s) = {total_motors} motors total.")
    print(f"Motor IDs to assign: {motor_ids}")
    print(f"We will identify each motor one at a time.\n")

    # Track assignments: monitor -> [id_for_motor0, id_for_motor1]
    assignments = {mon.port: [None, None] for mon in monitors}
    identified_motors = set()  # (port, motor_index) pairs already identified

    for i, motor_id in enumerate(motor_ids):
        while True:
            print("-" * 60)
            input(
                f">>> Move motor #{motor_id} ({i+1}/{total_motors}) now, then press Enter... "
            )
            print(f"    Watching for motion ({MOTION_DETECT_WINDOW}s)...")

            mon, motor_idx, motion = detect_motion(monitors)

            if mon is None:
                print(
                    f"    No motion detected (need >{MOTION_THRESHOLD} decideg). Try again."
                )
                continue

            key = (mon.port, motor_idx)
            if key in identified_motors:
                prev_id = assignments[mon.port][motor_idx]
                print(
                    f"    Detected {mon.port} motor_{motor_idx}, but that was already "
                    f"assigned as motor #{prev_id}. Try a different motor."
                )
                continue

            print(
                f"    Detected: {mon.port} motor_{motor_idx} "
                f"(moved {motion} decideg = {motion/10:.1f} deg)"
            )
            assignments[mon.port][motor_idx] = motor_id
            identified_motors.add(key)
            break

    # Summary
    print()
    print("=" * 60)
    print("  Calibration Results")
    print("=" * 60)
    for mon in monitors:
        id0, id1 = assignments[mon.port]
        print(f"  {mon.port}: motor_0 = #{id0}, motor_1 = #{id1}")

    # Confirm before writing
    print()
    resp = input("Write these IDs to controller NVS flash? (y/n): ").strip().lower()
    if resp != "y":
        print("Aborted. No changes written.")
        for mon in monitors:
            mon.close()
        return

    # Close monitors before writing (assign_identity opens its own connection)
    for mon in monitors:
        mon.close()

    # Write identities
    print()
    for port, ids in assignments.items():
        id0, id1 = ids
        confirmed = assign_identity(port, id0, id1)
        print(f"  {port}: wrote ({id0}, {id1}), confirmed = {confirmed}")

    print("\nDone! Motor IDs are saved to flash and will persist across reboots.")


if __name__ == "__main__":
    user_input = input("Enter starting motor ID [11]: ").strip()
    start_id = int(user_input) if user_input else 11
    run_calibration(starting_motor_id=start_id)
