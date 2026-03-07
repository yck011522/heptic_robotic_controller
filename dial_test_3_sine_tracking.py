# dial_test_3_sine_tracking.py
# Sends a slow sine-wave position trajectory to both motors at 50 Hz
# and monitors controller FPS and host-side loop timing jitter.
# Verifies that the control loop remains stable under continuous
# position updates over a 20-second window.

import serial
import time
import threading
import math
import statistics

# =========================
# Configuration
# =========================
PORT = "COM9"
BAUD = 230400
TEST_DURATION = 20.0
CONTROL_HZ = 50.0
INTERVAL = 1.0 / CONTROL_HZ
PERIOD = 20.0  # seconds per sine cycle

# Motion limits (decidegrees)
MIN_ANGLE = -10800  # -10 degrees
MAX_ANGLE = 10800  # +10 degrees

AMPLITUDE = (MAX_ANGLE - MIN_ANGLE) / 2 * 0.5  # 50% of the range
CENTER = (MAX_ANGLE + MIN_ANGLE) / 2

ser = serial.Serial(PORT, BAUD, timeout=0)

fps_values = []
host_loop_intervals = []

running = True
lock = threading.Lock()


# =========================
# Reader Thread
# =========================
def reader_thread():
    global running
    buffer = ""
    while running:
        data = ser.read(1024).decode(errors="ignore")
        if data:
            buffer += data
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                handle_line(line.strip())


def handle_line(line):
    if line.startswith("T,"):
        parts = line.split(",")
        if len(parts) >= 11:
            try:
                fps = int(parts[10])
                with lock:
                    fps_values.append(fps)
            except:
                pass


reader = threading.Thread(target=reader_thread)
reader.start()

print("Starting sine tracking test (decidegree)...")

start_time = time.perf_counter()
next_tick = start_time
seq = 0
last_tick_time = start_time

while time.perf_counter() - start_time < TEST_DURATION:

    now = time.perf_counter()

    if now >= next_tick:

        # Host loop timing measurement
        host_loop_intervals.append((now - last_tick_time) * 1000.0)
        last_tick_time = now

        seq += 1
        t = now - start_time

        phase = 2.0 * math.pi * (t / PERIOD)

        pos0 = int(CENTER + AMPLITUDE * math.sin(phase))
        pos1 = int(CENTER + AMPLITUDE * math.sin(phase))

        cmd = f"C,{seq},{pos0},{pos1},{MIN_ANGLE},{MAX_ANGLE},{MIN_ANGLE},{MAX_ANGLE}\n"
        ser.write(cmd.encode())

        next_tick += INTERVAL

    else:
        time.sleep(0.0005)

running = False
reader.join()
ser.close()

print("\nTest complete.")

# =========================
# Results
# =========================
if fps_values:
    print(f"Controller FPS avg: {statistics.mean(fps_values):.1f}")
    print(f"Controller FPS min: {min(fps_values)}")
    print(f"Controller FPS max: {max(fps_values)}")

if host_loop_intervals:
    print(f"\nHost loop mean interval: {statistics.mean(host_loop_intervals):.2f} ms")
    print(
        f"Host loop P95 interval: {statistics.quantiles(host_loop_intervals, n=20)[18]:.2f} ms"
    )
    print(f"Host loop max interval: {max(host_loop_intervals):.2f} ms")
