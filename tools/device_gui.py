import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import serial


ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from device_discovery import BAUD, DRAIN_DELAY, list_candidate_ports


PARAMS = [
    "tracking_kp",
    "tracking_kd",
    "detent_kp",
    "bounds_kp",
    "detent_distance",
    "tracking_max_torque",
    "bounds_max_torque",
    "detent_max_torque",
    "oob_kick_amplitude",
    "vibration_amplitude",
    "vibration_pulse_interval_ms",
    "oob_kick_pulse_interval_ms",
    "enable_tracking",
    "enable_bounds_restoration",
    "enable_oob_kick",
    "enable_detent",
    "enable_vibration",
    "telemetry_interval",
]

DEFAULTS = {
    "tracking_kp": 6000,
    "tracking_kd": 2,
    "detent_kp": 5000,
    "bounds_kp": 10000,
    "detent_distance": 10000,
    "tracking_max_torque": 6000,
    "bounds_max_torque": 300,
    "detent_max_torque": 200,
    "oob_kick_amplitude": 3000,
    "vibration_amplitude": 300,
    "vibration_pulse_interval_ms": 1000,
    "oob_kick_pulse_interval_ms": 40,
    "enable_tracking": 1,
    "enable_bounds_restoration": 1,
    "enable_oob_kick": 1,
    "enable_detent": 0,
    "enable_vibration": 0,
    "telemetry_interval": 20,
}


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Haptic Utility")
        self.root.geometry("820x760")

        self.ser: serial.Serial | None = None
        self.buf = ""
        self.seq = 1
        self.port_map: dict[str, str] = {}

        self.selected_port = tk.StringVar()
        self.status = tk.StringVar(value="Disconnected")
        self.info = tk.StringVar(value="-")
        self.target = tk.StringVar(value="0")
        self.minimum = tk.StringVar(value="-3600")
        self.maximum = tk.StringVar(value="3600")
        self.current_pos = tk.StringVar(value="0")
        self.param_vars = {name: tk.StringVar(value=str(DEFAULTS.get(name, 0))) for name in PARAMS}
        self.telemetry_vars = {
            "dial_id": tk.StringVar(value="-"),
            "seq": tk.StringVar(value="-"),
            "angle": tk.StringVar(value="-"),
            "speed": tk.StringVar(value="-"),
            "torque": tk.StringVar(value="-"),
            "foc": tk.StringVar(value="-"),
            "status": tk.StringVar(value="-"),
        }

        self._build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.poll_telemetry)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(3, weight=1)

        top = ttk.LabelFrame(self.root, text="Connection", padding=10)
        top.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 6))
        top.columnconfigure(1, weight=1)

        ttk.Label(top, text="COM Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, textvariable=self.selected_port, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 8))
        ttk.Button(top, text="Connect", command=self.connect).grid(row=0, column=3, padx=(0, 8))
        ttk.Button(top, text="Disconnect", command=self.disconnect).grid(row=0, column=4)
        ttk.Label(top, textvariable=self.status).grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Label(top, textvariable=self.info).grid(row=1, column=2, columnspan=3, sticky="w", pady=(8, 0))

        cmd = ttk.LabelFrame(self.root, text="Commands", padding=10)
        cmd.grid(row=1, column=0, sticky="ew", padx=10, pady=6)
        cmd.columnconfigure(1, weight=1)

        ttk.Label(cmd, text="Target (decideg)").grid(row=0, column=0, sticky="w")
        ttk.Entry(cmd, textvariable=self.target).grid(row=0, column=1, sticky="ew", pady=3)
        ttk.Label(cmd, text="Min (decideg)").grid(row=1, column=0, sticky="w")
        ttk.Entry(cmd, textvariable=self.minimum).grid(row=1, column=1, sticky="ew", pady=3)
        ttk.Label(cmd, text="Max (decideg)").grid(row=2, column=0, sticky="w")
        ttk.Entry(cmd, textvariable=self.maximum).grid(row=2, column=1, sticky="ew", pady=3)
        ttk.Button(cmd, text="Apply Target + Bounds (C)", command=self.apply_control).grid(row=3, column=0, columnspan=2, sticky="ew", pady=(6, 8))
        ttk.Label(cmd, text="set_current_position (decideg)").grid(row=4, column=0, sticky="w")
        ttk.Entry(cmd, textvariable=self.current_pos).grid(row=4, column=1, sticky="ew", pady=3)
        ttk.Button(cmd, text="Send R", command=self.set_current_position).grid(row=5, column=0, columnspan=2, sticky="ew", pady=(6, 0))

        telemetry = ttk.LabelFrame(cmd, text="Live Telemetry", padding=8)
        telemetry.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        telemetry.columnconfigure(1, weight=1)
        telemetry.columnconfigure(3, weight=1)

        ttk.Label(telemetry, text="Dial ID").grid(row=0, column=0, sticky="w")
        ttk.Label(telemetry, textvariable=self.telemetry_vars["dial_id"]).grid(row=0, column=1, sticky="w", padx=(6, 12))
        ttk.Label(telemetry, text="Seq").grid(row=0, column=2, sticky="w")
        ttk.Label(telemetry, textvariable=self.telemetry_vars["seq"]).grid(row=0, column=3, sticky="w")

        ttk.Label(telemetry, text="Angle").grid(row=1, column=0, sticky="w")
        ttk.Label(telemetry, textvariable=self.telemetry_vars["angle"]).grid(row=1, column=1, sticky="w", padx=(6, 12))
        ttk.Label(telemetry, text="Speed").grid(row=1, column=2, sticky="w")
        ttk.Label(telemetry, textvariable=self.telemetry_vars["speed"]).grid(row=1, column=3, sticky="w")

        ttk.Label(telemetry, text="Torque").grid(row=2, column=0, sticky="w")
        ttk.Label(telemetry, textvariable=self.telemetry_vars["torque"]).grid(row=2, column=1, sticky="w", padx=(6, 12))
        ttk.Label(telemetry, text="FOC Hz").grid(row=2, column=2, sticky="w")
        ttk.Label(telemetry, textvariable=self.telemetry_vars["foc"]).grid(row=2, column=3, sticky="w")

        ttk.Label(telemetry, text="Status Bits").grid(row=3, column=0, sticky="w")
        ttk.Label(telemetry, textvariable=self.telemetry_vars["status"]).grid(row=3, column=1, columnspan=3, sticky="w", padx=(6, 0))

        cfg = ttk.LabelFrame(self.root, text="Runtime Parameters", padding=10)
        cfg.grid(row=2, column=0, sticky="nsew", padx=10, pady=6)
        cfg.columnconfigure(1, weight=1)
        cfg.columnconfigure(3, weight=1)

        half = (len(PARAMS) + 1) // 2
        left = PARAMS[:half]
        right = PARAMS[half:]
        for row, name in enumerate(left):
            ttk.Label(cfg, text=name).grid(row=row, column=0, sticky="w", pady=2, padx=(0, 8))
            ttk.Entry(cfg, textvariable=self.param_vars[name], width=14).grid(row=row, column=1, sticky="ew", pady=2)
        for row, name in enumerate(right):
            ttk.Label(cfg, text=name).grid(row=row, column=2, sticky="w", pady=2, padx=(16, 8))
            ttk.Entry(cfg, textvariable=self.param_vars[name], width=14).grid(row=row, column=3, sticky="ew", pady=2)

        row = max(len(left), len(right)) + 1
        ttk.Button(cfg, text="Read Params (G)", command=self.read_params).grid(row=row, column=0, columnspan=2, sticky="ew", pady=(8, 0), padx=(0, 8))
        ttk.Button(cfg, text="Upload Params (S)", command=self.write_params).grid(row=row, column=2, columnspan=2, sticky="ew", pady=(8, 0))

        log_frame = ttk.LabelFrame(self.root, text="Log", padding=10)
        log_frame.grid(row=3, column=0, sticky="nsew", padx=10, pady=(6, 10))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = ScrolledText(log_frame, height=10, wrap="word", state="disabled")
        self.log.grid(row=0, column=0, sticky="nsew")

    def log_line(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", f"[{time.strftime('%H:%M:%S')}] {text}\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def refresh_ports(self) -> None:
        candidates = list_candidate_ports(filter_by_vid_pid=False)
        labels = []
        self.port_map = {}
        for c in candidates:
            label = f"{c['port']} | {c.get('description', 'unknown')}"
            labels.append(label)
            self.port_map[label] = c["port"]
        self.port_combo["values"] = labels
        if labels and self.selected_port.get() not in labels:
            self.selected_port.set(labels[0])
        if not labels:
            self.selected_port.set("")
        self.log_line(f"Found {len(labels)} serial port(s)")

    def connect(self) -> None:
        label = self.selected_port.get().strip()
        port = self.port_map.get(label)
        if not port:
            messagebox.showerror("Connect", "Select a COM port")
            return

        self.disconnect(log=False)
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.05)
            time.sleep(DRAIN_DELAY)
            self.reset_input()
            fw = self.query_value("V")
            did = self.query_value("I")
            try:
                telemetry = self.read_until(lambda line: line.startswith("T,"), timeout=1.0)
                self.update_telemetry(telemetry)
            except TimeoutError:
                pass
            self.status.set(f"Connected: {port}")
            self.info.set(f"fw={fw}, dial_id={did}")
            self.log_line(f"Connected to {port}; fw={fw}, dial_id={did}")
        except Exception as e:
            self.disconnect(log=False)
            self.status.set("Disconnected")
            self.info.set("-")
            messagebox.showerror("Connect", str(e))
            self.log_line(f"Connect failed: {e}")

    def disconnect(self, log: bool = True) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.buf = ""
        self.status.set("Disconnected")
        self.info.set("-")
        for field in self.telemetry_vars.values():
            field.set("-")
        if log:
            self.log_line("Disconnected")

    def require_serial(self) -> serial.Serial:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Connect first")
        return self.ser

    def next_seq(self) -> int:
        s = self.seq
        self.seq += 1
        return s

    def reset_input(self) -> None:
        ser = self.require_serial()
        ser.reset_input_buffer()
        self.buf = ""

    def read_until(self, predicate, timeout: float = 1.5) -> str:
        ser = self.require_serial()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            data = ser.read(ser.in_waiting or 1)
            if not data:
                continue
            self.buf += data.decode(errors="ignore")
            while "\n" in self.buf:
                line, self.buf = self.buf.split("\n", 1)
                line = line.strip()
                if line and predicate(line):
                    return line
        raise TimeoutError("Timed out waiting for device response")

    def send_wait(self, command: str, prefix: str, timeout: float = 1.5) -> str:
        self.reset_input()
        self.require_serial().write((command + "\n").encode())
        return self.read_until(lambda line: line.startswith(prefix), timeout)

    def update_telemetry(self, line: str) -> None:
        parts = line.split(",")
        if len(parts) != 8 or parts[0] != "T":
            return

        self.telemetry_vars["dial_id"].set(parts[1])
        self.telemetry_vars["seq"].set(parts[2])
        self.telemetry_vars["angle"].set(parts[3])
        self.telemetry_vars["speed"].set(parts[4])
        self.telemetry_vars["torque"].set(parts[5])
        self.telemetry_vars["foc"].set(parts[6])
        self.telemetry_vars["status"].set(parts[7])

    def poll_telemetry(self) -> None:
        if self.ser is not None and self.ser.is_open:
            try:
                data = self.ser.read(self.ser.in_waiting or 0)
                if data:
                    self.buf += data.decode(errors="ignore")
                    while "\n" in self.buf:
                        line, self.buf = self.buf.split("\n", 1)
                        line = line.strip()
                        if line.startswith("T,"):
                            self.update_telemetry(line)
            except (serial.SerialException, OSError) as e:
                self.log_line(f"Telemetry read failed: {e}")
                self.disconnect(log=False)

        self.root.after(100, self.poll_telemetry)

    def query_value(self, cmd: str) -> str:
        seq = self.next_seq()
        resp = self.send_wait(f"{cmd},{seq}", f"{cmd},{seq}")
        parts = resp.split(",", 2)
        if len(parts) < 3:
            raise ValueError(f"Unexpected response: {resp}")
        return parts[2]

    def read_params(self) -> None:
        try:
            for name in PARAMS:
                seq = self.next_seq()
                resp = self.send_wait(f"G,{seq},{name}", f"G,{seq},{name},")
                self.param_vars[name].set(resp.rsplit(",", 1)[1])
            self.log_line("Read runtime parameters")
        except Exception as e:
            messagebox.showerror("Read Params", str(e))
            self.log_line(f"Read params failed: {e}")

    def write_params(self) -> None:
        try:
            for name in PARAMS:
                value = int(self.param_vars[name].get().strip())
                seq = self.next_seq()
                self.send_wait(f"S,{seq},{name},{value}", f"S,{seq}")
            self.log_line("Uploaded runtime parameters")
        except Exception as e:
            messagebox.showerror("Upload Params", str(e))
            self.log_line(f"Upload params failed: {e}")

    def apply_control(self) -> None:
        try:
            target = int(self.target.get().strip())
            minimum = int(self.minimum.get().strip())
            maximum = int(self.maximum.get().strip())
            if minimum > maximum:
                raise ValueError("Min must be <= Max")
            seq = self.next_seq()
            self.reset_input()
            self.require_serial().write(f"C,{seq},{target},{minimum},{maximum}\n".encode())
            telemetry = self.read_until(lambda line: line.startswith("T,") and line.split(",", 3)[2] == str(seq), timeout=2.0)
            self.update_telemetry(telemetry)
            self.log_line(f"Applied C target={target}, min={minimum}, max={maximum}; {telemetry}")
        except Exception as e:
            messagebox.showerror("Apply Control", str(e))
            self.log_line(f"Apply control failed: {e}")

    def set_current_position(self) -> None:
        try:
            value = int(self.current_pos.get().strip())
            seq = self.next_seq()
            self.send_wait(f"R,{seq},{value}", f"R,{seq}")
            self.log_line(f"Sent set_current_position={value}")
        except Exception as e:
            messagebox.showerror("set_current_position", str(e))
            self.log_line(f"set_current_position failed: {e}")

    def on_close(self) -> None:
        self.disconnect(log=False)
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    style = ttk.Style(root)
    if "vista" in style.theme_names():
        style.theme_use("vista")
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
