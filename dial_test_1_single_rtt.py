# dial_test_1_single_rtt.py
# Measures round-trip time (RTT) of the C (control) command by sending
# position commands at 100 Hz and timing how long until the matching
# seq appears in the T (telemetry) response. Reports mean, P50, P95,
# P99, max RTT and average controller FPS over a 60-second window.

import serial
import time
import threading
import statistics

PORT = "COM9"
BAUD = 230400
TEST_DURATION = 60.0
SEND_HZ = 100.0
SEND_INTERVAL = 1.0 / SEND_HZ

ser = serial.Serial(PORT, BAUD, timeout=0)

send_times = {}
rtts = []
fps_values = []
lock = threading.Lock()
running = True


def reader_thread():
    global running
    buffer = ""
    while running:
        try:
            data = ser.read(1024).decode(errors="ignore")
            if data:
                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    handle_line(line.strip())
        except:
            pass


def handle_line(line):
    global rtts
    if not line.startswith("T,"):
        return
    parts = line.split(",")
    if len(parts) < 9:
        return
    try:
        seq = int(parts[3])
        fps = int(parts[8])
    except:
        return

    now = time.perf_counter()
    with lock:
        if seq in send_times:
            rtt = (now - send_times.pop(seq)) * 1000.0
            rtts.append(rtt)
        fps_values.append(fps)


reader = threading.Thread(target=reader_thread)
reader.start()

print("Starting Phase 1 RTT test...")
start_time = time.perf_counter()
next_send = start_time
seq = 0

while time.perf_counter() - start_time < TEST_DURATION:
    now = time.perf_counter()
    if now >= next_send:
        seq += 1
        cmd = f"C,{seq},0,0,-3600,3600,-3600,3600\n"
        ser.write(cmd.encode())
        with lock:
            send_times[seq] = now
        next_send += SEND_INTERVAL
    else:
        time.sleep(0.0005)

running = False
reader.join()
ser.close()

print("\nTest complete.")

if rtts:
    print(f"Packets received: {len(rtts)}")
    print(f"Mean RTT: {statistics.mean(rtts):.2f} ms")
    print(f"P50 RTT: {statistics.median(rtts):.2f} ms")
    print(f"P95 RTT: {statistics.quantiles(rtts, n=20)[18]:.2f} ms")
    print(f"P99 RTT: {statistics.quantiles(rtts, n=100)[98]:.2f} ms")
    print(f"Max RTT: {max(rtts):.2f} ms")
else:
    print("No RTT data received.")

if fps_values:
    print(f"Controller FPS avg: {statistics.mean(fps_values):.1f}")
