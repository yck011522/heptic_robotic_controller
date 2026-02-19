import serial
import time
import threading
import statistics

# =========================
# Configuration
# =========================
PORT = "COM5"
BAUD = 230400
TEST_DURATION = 60.0
RATE_HZ = 50.0
INTERVAL = 1.0 / RATE_HZ

# =========================
# Shared State
# =========================
ser = serial.Serial(PORT, BAUD, timeout=0)

send_times = {}  # seq -> send timestamp
rtts = []  # echo RTT values (ms)
fps_values = []  # controller loop FPS samples

lock = threading.Lock()
running = True


# =========================
# Serial Reader Thread
# Continuously parses incoming lines
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


# =========================
# Line Parser
# Handles both R (echo) and T (telemetry)
# =========================
def handle_line(line):
    now = time.perf_counter()

    # Echo reply
    if line.startswith("R,"):
        parts = line.split(",")
        if len(parts) < 2:
            return
        try:
            seq = int(parts[1])
        except:
            return

        with lock:
            if seq in send_times:
                rtt_ms = (now - send_times.pop(seq)) * 1000.0
                rtts.append(rtt_ms)

    # Telemetry reply
    elif line.startswith("T,"):
        parts = line.split(",")
        if len(parts) < 7:
            return
        try:
            fps = int(parts[6])
        except:
            return

        with lock:
            fps_values.append(fps)


# =========================
# Start Reader Thread
# =========================
reader = threading.Thread(target=reader_thread)
reader.start()

print("Starting 50 Hz control + echo RTT test...")

start_time = time.perf_counter()
next_tick = start_time
seq = 0

# =========================
# Main 50 Hz Send Loop
# =========================
while time.perf_counter() - start_time < TEST_DURATION:

    now = time.perf_counter()

    if now >= next_tick:
        seq += 1

        # --- Send Control Command ---
        c_cmd = f"C,{seq},0,0,-3600,3600,-3600,3600\n"
        ser.write(c_cmd.encode())

        # --- Send Echo Command ---
        e_cmd = f"E,{seq}\n"
        ser.write(e_cmd.encode())

        # Record send timestamp for RTT measurement
        with lock:
            send_times[seq] = now

        next_tick += INTERVAL

    else:
        time.sleep(0.0005)

# =========================
# Shutdown
# =========================
running = False
reader.join()
ser.close()

print("\nTest complete.")

# =========================
# Results
# =========================
if rtts:
    print(f"Echo replies received: {len(rtts)}")
    print(f"Mean RTT: {statistics.mean(rtts):.2f} ms")
    print(f"P50 RTT: {statistics.median(rtts):.2f} ms")
    print(f"P95 RTT: {statistics.quantiles(rtts, n=20)[18]:.2f} ms")
    print(f"P99 RTT: {statistics.quantiles(rtts, n=100)[98]:.2f} ms")
    print(f"Max RTT: {max(rtts):.2f} ms")
else:
    print("No echo RTT data received.")

if fps_values:
    print(f"\nController FPS avg: {statistics.mean(fps_values):.1f}")
    print(f"Controller FPS min: {min(fps_values)}")
    print(f"Controller FPS max: {max(fps_values)}")
else:
    print("No FPS data received.")


# Result tested with the Controller at 50Hz T Telemetry speed.
# Starting 50 Hz control + echo RTT test...

# Test complete.
# Echo replies received: 2997
# Mean RTT: 5.40 ms
# P50 RTT: 5.39 ms
# P95 RTT: 6.24 ms
# P99 RTT: 6.46 ms
# Max RTT: 10.03 ms

# Controller FPS avg: 541.8
# Controller FPS min: 540
# Controller FPS max: 574
