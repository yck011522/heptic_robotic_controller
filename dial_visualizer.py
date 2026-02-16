"""
Dial Position Visualizer
Real-time GUI to display motor angle and torque data from the FOC Controller

Features:
- Reads serial data from the microcontroller
- Displays motor positions as arrows on a -270 to +270 degree number line
- Visualizes torque magnitude and direction through arrow color and labels
- Real-time updating with FPS display
"""

import serial
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.patches as patches
from collections import deque
from datetime import datetime


class DialVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("Dial Position Visualizer")
        self.root.geometry("1200x600")

        # Serial connection
        self.ser = None
        self.serial_thread = None
        self.stop_event = threading.Event()

        # Data storage
        self.data = {
            "timestamp": 0,
            "fps": 0.0,
            "M0_torque": 0.0,
            "M0_angle": 0.0,
            "M1_torque": 0.0,
            "M1_angle": 0.0,
        }

        # Data history for optional plotting
        self.history_size = 100
        self.history = {
            "M0_angle": deque(maxlen=self.history_size),
            "M0_torque": deque(maxlen=self.history_size),
            "M1_angle": deque(maxlen=self.history_size),
            "M1_torque": deque(maxlen=self.history_size),
        }

        # GUI update throttling (50ms = 20Hz max update rate for plot)
        self.plot_update_interval_ms = 50  # Update visualization at 20Hz
        self.last_plot_update_time = 0
        self.data_dirty = False  # Flag to indicate new data needs rendering

        self.setup_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_ui(self):
        """Create the UI layout"""
        # Top control panel
        control_frame = ttk.Frame(self.root)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        # Serial port selection
        ttk.Label(control_frame, text="Serial Port:").pack(anchor=tk.W)
        self.port_var = tk.StringVar(value="COM5")
        port_entry = ttk.Entry(control_frame, textvariable=self.port_var, width=15)
        port_entry.pack(anchor=tk.W, pady=(0, 10))

        # Baud rate selection
        ttk.Label(control_frame, text="Baud Rate:").pack(anchor=tk.W)
        self.baud_var = tk.StringVar(value="115200")
        baud_spinbox = ttk.Spinbox(
            control_frame, from_=300, to=921600, textvariable=self.baud_var, width=15
        )
        baud_spinbox.pack(anchor=tk.W, pady=(0, 10))

        # Connect/Disconnect buttons
        self.connect_btn = ttk.Button(
            control_frame, text="Connect", command=self.connect_serial
        )
        self.connect_btn.pack(fill=tk.X, pady=(0, 5))

        self.disconnect_btn = ttk.Button(
            control_frame,
            text="Disconnect",
            command=self.disconnect_serial,
            state=tk.DISABLED,
        )
        self.disconnect_btn.pack(fill=tk.X, pady=(0, 20))

        # Status display
        ttk.Label(control_frame, text="Status:").pack(anchor=tk.W)
        self.status_var = tk.StringVar(value="Disconnected")
        status_label = ttk.Label(
            control_frame,
            textvariable=self.status_var,
            foreground="red",
            font=("Arial", 10, "bold"),
        )
        status_label.pack(anchor=tk.W, pady=(0, 20))

        # Data display panel
        ttk.Label(control_frame, text="Live Data:", font=("Arial", 11, "bold")).pack(
            anchor=tk.W
        )

        self.fps_var = tk.StringVar(value="FPS: 0.0")
        ttk.Label(control_frame, textvariable=self.fps_var).pack(anchor=tk.W)

        ttk.Label(control_frame, text="Motor 0:", font=("Arial", 10, "bold")).pack(
            anchor=tk.W, pady=(10, 0)
        )
        self.m0_angle_var = tk.StringVar(value="Angle: 0.0°")
        ttk.Label(control_frame, textvariable=self.m0_angle_var).pack(anchor=tk.W)

        self.m0_torque_var = tk.StringVar(value="Torque: 0.00 A")
        ttk.Label(control_frame, textvariable=self.m0_torque_var).pack(anchor=tk.W)

        ttk.Label(control_frame, text="Motor 1:", font=("Arial", 10, "bold")).pack(
            anchor=tk.W, pady=(10, 0)
        )
        self.m1_angle_var = tk.StringVar(value="Angle: 0.0°")
        ttk.Label(control_frame, textvariable=self.m1_angle_var).pack(anchor=tk.W)

        self.m1_torque_var = tk.StringVar(value="Torque: 0.00 A")
        ttk.Label(control_frame, textvariable=self.m1_torque_var).pack(anchor=tk.W)

        # Plot area
        self.fig = Figure(figsize=(10, 6), dpi=80)
        self.ax = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(
            side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10
        )

        # Initial plot setup
        self.update_plot()

        # Schedule periodic GUI updates from main thread
        self.scheduled_gui_update()

    def connect_serial(self):
        """Connect to the serial port"""
        try:
            port = self.port_var.get()
            baud = int(self.baud_var.get())

            self.ser = serial.Serial(port, baud, timeout=1)
            self.status_var.set("Connected ✓")

            # Change UI state
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.status_var.set("Connected ✓")

            # Start reading thread
            self.stop_event.clear()
            self.serial_thread = threading.Thread(
                target=self.read_serial_data, daemon=True
            )
            self.serial_thread.start()

        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
            self.status_var.set("Error")

    def disconnect_serial(self):
        """Disconnect from serial port"""
        self.stop_event.set()
        if self.ser and self.ser.is_open:
            self.ser.close()

        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.status_var.set("Disconnected")

    def read_serial_data(self):
        """Read and parse serial data from the microcontroller

        This runs in a separate thread and only handles parsing.
        GUI updates are handled by scheduled_gui_update() on the main thread.
        """
        while not self.stop_event.is_set():
            try:
                if self.ser and self.ser.is_open:
                    line = self.ser.readline().decode("utf-8").strip()
                    if line:
                        self.parse_protocol(line)
                        self.data_dirty = True  # Flag that new data is available
            except Exception as e:
                print(f"Serial read error: {e}")
                self.root.after(100)

    def parse_protocol(self, line):
        """Parse the communication protocol: T:value,F:value,M0T:value,..."""
        try:
            fields = line.split(",")
            for field in fields:
                if ":" not in field:
                    continue
                key, value = field.split(":")
                value = float(value)

                if key == "T":
                    self.data["timestamp"] = int(value)
                elif key == "F":
                    self.data["fps"] = value
                elif key == "M0T":
                    self.data["M0_torque"] = value
                elif key == "M0A":
                    self.data["M0_angle"] = value
                elif key == "M1T":
                    self.data["M1_torque"] = value
                elif key == "M1A":
                    self.data["M1_angle"] = value

            # Store in history
            self.history["M0_angle"].append(self.data["M0_angle"])
            self.history["M0_torque"].append(self.data["M0_torque"])
            self.history["M1_angle"].append(self.data["M1_angle"])
            self.history["M1_torque"].append(self.data["M1_torque"])

        except Exception as e:
            print(f"Parse error: {e}")

    def update_display(self):
        """Update the status labels with current data (called from main thread)"""
        self.fps_var.set(f"FPS: {self.data['fps']:.1f}")
        self.m0_angle_var.set(f"Angle: {self.data['M0_angle']:.1f}°")
        self.m0_torque_var.set(f"Torque: {self.data['M0_torque']:.2f} A")
        self.m1_angle_var.set(f"Angle: {self.data['M1_angle']:.1f}°")
        self.m1_torque_var.set(f"Torque: {self.data['M1_torque']:.2f} A")

    def scheduled_gui_update(self):
        """Periodically update GUI from main thread at throttled rate

        This is called from the main tkinter event loop rather than the serial thread,
        preventing GUI performance issues from high-frequency serial data (50Hz).
        """
        import time

        now = time.time() * 1000  # Convert to milliseconds

        # Always update text labels if data is available
        if self.data_dirty:
            self.update_display()

            # Update plot only at throttled interval
            if now - self.last_plot_update_time >= self.plot_update_interval_ms:
                self.update_plot()
                self.last_plot_update_time = now
                self.data_dirty = False

        # Schedule next update (20ms = 50Hz check rate)
        self.root.after(20, self.scheduled_gui_update)

    def update_plot(self):
        """Update the visualization plot"""
        self.ax.clear()

        # Configure the axis
        self.ax.set_xlim(-300, 300)
        self.ax.set_ylim(-2, 3)
        self.ax.set_xlabel("Angle (degrees)", fontsize=12, fontweight="bold")
        self.ax.set_title("Motor Dial Positions", fontsize=14, fontweight="bold")

        # Remove y-axis
        self.ax.set_yticks([])
        self.ax.spines["left"].set_visible(False)
        self.ax.spines["right"].set_visible(False)
        self.ax.spines["top"].set_visible(False)

        # Draw the main number line
        self.ax.axhline(y=0, color="black", linewidth=2, zorder=1)

        # Add tick marks and labels
        tick_positions = range(-270, 271, 45)
        for tick in tick_positions:
            self.ax.plot([tick, tick], [-0.1, 0.1], "k-", linewidth=1, zorder=2)
            self.ax.text(tick, -0.4, f"{tick}°", ha="center", fontsize=9)

        # Add center reference line
        self.ax.plot([0, 0], [-0.5, 2.5], "k--", linewidth=1, alpha=0.3, zorder=0)
        self.ax.text(0, 2.7, "0°", ha="center", fontsize=10, fontweight="bold")

        # Draw Motor 0 arrow
        m0_angle = self.data["M0_angle"]
        m0_torque = self.data["M0_torque"]
        self.draw_arrow(m0_angle, m0_torque, 1.5, label="M0", color="blue")

        # Draw Motor 1 arrow
        m1_angle = self.data["M1_angle"]
        m1_torque = self.data["M1_torque"]
        self.draw_arrow(m1_angle, m1_torque, 1.0, label="M1", color="red")

        # Add legend with torque information
        legend_text = f"M0: {m0_angle:.1f}° ({m0_torque:+.2f}A)\nM1: {m1_angle:.1f}° ({m1_torque:+.2f}A)"
        self.ax.text(
            0.02,
            0.98,
            legend_text,
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
        )

        self.canvas.draw()

    def draw_arrow(self, angle, torque, y_position, label, color):
        """Draw an arrow at the specified angle and height"""
        # Clamp angle to valid range
        angle = max(-270, min(270, angle))

        # Arrow size based on torque magnitude
        arrow_scale = min(2.0, max(0.3, abs(torque))) / 2.0

        # Color intensity based on torque direction and magnitude
        if torque > 0:
            arrow_color = color
            alpha = min(1.0, 0.5 + abs(torque) * 0.25)
        else:
            arrow_color = color
            alpha = min(1.0, 0.5 + abs(torque) * 0.25)

        # Draw the arrow
        arrow = patches.FancyArrowPatch(
            (angle - 10, y_position),
            (angle + 10, y_position),
            arrowstyle="->",
            mutation_scale=30,
            linewidth=2,
            color=arrow_color,
            alpha=alpha,
            zorder=10,
        )
        self.ax.add_patch(arrow)

        # Add label above arrow
        self.ax.text(
            angle,
            y_position + 0.35,
            label,
            ha="center",
            fontsize=10,
            fontweight="bold",
            color=color,
            zorder=11,
        )

    def on_closing(self):
        """Handle window closing"""
        self.disconnect_serial()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = DialVisualizer(root)
    root.mainloop()
