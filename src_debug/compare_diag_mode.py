import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import serial


DEFAULT_BAUD = 230400
DEFAULT_PORT = "COM9"
DEFAULT_TURNS = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare encoder capture with diagnostics OFF vs ON (fixed turns capture)."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    parser.add_argument("--turns", type=int, default=DEFAULT_TURNS, help="Turns for both runs")
    parser.add_argument("--seq-start", type=int, default=100, help="Starting sequence number")
    parser.add_argument("--timeout", type=float, default=20.0, help="Capture timeout (seconds)")
    parser.add_argument(
        "--startup-settle",
        type=float,
        default=1.0,
        help="Delay after opening serial before first command",
    )
    parser.add_argument(
        "--inter-run-delay",
        type=float,
        default=0.5,
        help="Delay between OFF and ON runs",
    )
    parser.add_argument(
        "--output-prefix",
        default="encoder_diag_compare",
        help="Output file prefix",
    )
    return parser.parse_args()


def read_until_line(ser: serial.Serial, predicate, timeout_s: float) -> str:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue
        print(line)
        if predicate(line):
            return line
    raise TimeoutError("Timed out waiting for expected serial response")


def send_diag_mode(ser: serial.Serial, seq: int, enabled: bool, timeout_s: float) -> None:
    value = 1 if enabled else 0
    cmd = f"D,{seq},{value}\n".encode("ascii")
    ser.write(cmd)
    ser.flush()

    def match(line: str) -> bool:
        if line.startswith(f"D,{seq},ERR"):
            raise RuntimeError(f"Firmware rejected D command: {line}")
        return line == f"D,{seq},{value}"

    read_until_line(ser, match, timeout_s)


def read_capture_lines(ser: serial.Serial, seq: int, turns: int, timeout_s: float) -> list[str]:
    cmd = f"R,{seq},{turns}\n".encode("ascii")
    ser.write(cmd)
    ser.flush()

    deadline = time.time() + timeout_s
    in_log = False
    lines: list[str] = []

    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        print(line)

        if line.startswith(f"R,{seq},ERR"):
            raise RuntimeError(f"Firmware rejected R command: {line}")

        if line.startswith("LOG_BEGIN,"):
            in_log = True
            lines = [line]
            continue

        if in_log:
            lines.append(line)
            if line.startswith("LOG_END,"):
                return lines

    raise TimeoutError(f"Timed out waiting for capture seq={seq}, turns={turns}")


def parse_log_lines(lines: list[str], diag_enabled: int, turns: int) -> list[dict[str, int | float]]:
    rows: list[dict[str, int | float]] = []

    for line in lines:
        if line.startswith("LOG_BEGIN,") or line.startswith("LOG_HEADER,") or line.startswith("LOG_END,"):
            continue

        payload = line
        if line.startswith("LOG_DATA,"):
            payload = line.split(",", 1)[1]

        parts = payload.split(",")
        if len(parts) != 7:
            continue

        try:
            rows.append(
                {
                    "t_us": int(parts[0]),
                    "openloop_el_angle_rad": float(parts[1]),
                    "sensor_raw": int(parts[2]),
                    "sensor_error": int(parts[3]),
                    "status": int(parts[4]),
                    "agc": int(parts[5]),
                    "magnitude": int(parts[6]),
                    "diag_enabled": diag_enabled,
                    "turns": turns,
                }
            )
        except ValueError:
            continue

    if not rows:
        raise ValueError("No LOG_DATA rows parsed")

    return rows


def write_csv(rows: list[dict[str, int | float]], path: Path) -> None:
    fields = [
        "t_us",
        "openloop_el_angle_rad",
        "sensor_raw",
        "sensor_error",
        "status",
        "agc",
        "magnitude",
        "diag_enabled",
        "turns",
    ]
    with path.open("w", newline="", encoding="utf-8") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def wrap_aware_steps(raw: list[int]) -> list[int]:
    out: list[int] = []
    for i in range(1, len(raw)):
        d = raw[i] - raw[i - 1]
        if d > 2048:
            d -= 4096
        elif d < -2048:
            d += 4096
        out.append(d)
    return out


def compute_metrics(rows: list[dict[str, int | float]], spike_thr: int = 100) -> dict[str, int | float]:
    raw = [int(r["sensor_raw"]) for r in rows]
    naive_steps = [abs(raw[i] - raw[i - 1]) for i in range(1, len(raw))]
    wrap_steps = [abs(v) for v in wrap_aware_steps(raw)]

    return {
        "samples": len(rows),
        "sensor_error_count": sum(int(r["sensor_error"]) for r in rows),
        "count_raw_0": sum(1 for v in raw if v == 0),
        "count_raw_4095": sum(1 for v in raw if v == 4095),
        "naive_spike_count": sum(1 for v in naive_steps if v > spike_thr),
        "wrap_spike_count": sum(1 for v in wrap_steps if v > spike_thr),
    }


def write_summary(summary_rows: list[dict[str, int | float]], path: Path) -> None:
    fields = [
        "diag_enabled",
        "samples",
        "sensor_error_count",
        "count_raw_0",
        "count_raw_4095",
        "naive_spike_count",
        "wrap_spike_count",
    ]
    with path.open("w", newline="", encoding="utf-8") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summary_rows)


def write_compare_plot(
    rows_off: list[dict[str, int | float]],
    rows_on: list[dict[str, int | float]],
    metrics_off: dict[str, int | float],
    metrics_on: dict[str, int | float],
    path: Path,
) -> None:
    t_off = [float(r["t_us"]) / 1000.0 for r in rows_off]
    t_on = [float(r["t_us"]) / 1000.0 for r in rows_on]
    raw_off = [int(r["sensor_raw"]) for r in rows_off]
    raw_on = [int(r["sensor_raw"]) for r in rows_on]

    step_off = [abs(v) for v in wrap_aware_steps(raw_off)]
    step_on = [abs(v) for v in wrap_aware_steps(raw_on)]

    status_on = [int(r["status"]) for r in rows_on]
    agc_on = [int(r["agc"]) for r in rows_on]
    mag_on = [int(r["magnitude"]) for r in rows_on]

    md_on = [((value >> 5) & 0x01) if value >= 0 else 0 for value in status_on]
    ml_on = [((value >> 4) & 0x01) if value >= 0 else 0 for value in status_on]
    mh_on = [((value >> 3) & 0x01) if value >= 0 else 0 for value in status_on]

    fig, axes = plt.subplots(5, 1, figsize=(12, 13), sharex=False)

    axes[0].plot(t_off, raw_off, label="diag OFF", linewidth=1.0)
    axes[0].plot(t_on, raw_on, label="diag ON", linewidth=1.0, alpha=0.85)
    axes[0].set_ylabel("AS5600 Raw")
    axes[0].set_title(
        "Diagnostics OFF vs ON (turns=2) | "
        f"wrap spikes OFF={metrics_off['wrap_spike_count']} ON={metrics_on['wrap_spike_count']}"
    )
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_off[1:], step_off, label="OFF |Δraw|", linewidth=1.0)
    axes[1].plot(t_on[1:], step_on, label="ON |Δraw|", linewidth=1.0, alpha=0.85)
    axes[1].axhline(100, color="red", linestyle="--", linewidth=0.8, label="spike threshold")
    axes[1].set_ylabel("|Δraw| wrap-aware")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].step(t_on, md_on, where="post", label="MD", linewidth=1.0)
    axes[2].step(t_on, ml_on, where="post", label="ML", linewidth=1.0)
    axes[2].step(t_on, mh_on, where="post", label="MH", linewidth=1.0)
    axes[2].set_ylabel("Status (diag ON)")
    axes[2].set_ylim(-0.1, 1.1)
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t_on, agc_on, linewidth=1.0)
    axes[3].set_ylabel("AGC (diag ON)")
    axes[3].grid(True, alpha=0.3)

    axes[4].plot(t_on, mag_on, linewidth=1.0)
    axes[4].set_ylabel("Magnitude (diag ON)")
    axes[4].set_xlabel("Time (ms)")
    axes[4].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main() -> int:
    args = parse_args()

    if args.turns != 2:
        print("WARN: This comparator is intended for turns=2. Continuing with requested value.")

    out_dir = Path(__file__).resolve().parent
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = f"{args.output_prefix}_{stamp}"

    seq = args.seq_start

    try:
        with serial.Serial(args.port, args.baud, timeout=0.25) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            if args.startup_settle > 0:
                time.sleep(args.startup_settle)
                ser.reset_input_buffer()

            send_diag_mode(ser, seq, enabled=False, timeout_s=args.timeout)
            seq += 1
            lines_off = read_capture_lines(ser, seq, turns=args.turns, timeout_s=args.timeout)
            seq += 1
            rows_off = parse_log_lines(lines_off, diag_enabled=0, turns=args.turns)

            if args.inter_run_delay > 0:
                time.sleep(args.inter_run_delay)

            send_diag_mode(ser, seq, enabled=True, timeout_s=args.timeout)
            seq += 1
            lines_on = read_capture_lines(ser, seq, turns=args.turns, timeout_s=args.timeout)
            rows_on = parse_log_lines(lines_on, diag_enabled=1, turns=args.turns)

    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    csv_off = out_dir / f"{base}_diag_off_turns{args.turns}.csv"
    csv_on = out_dir / f"{base}_diag_on_turns{args.turns}.csv"
    plot_path = out_dir / f"{base}_compare_turns{args.turns}.png"
    summary_path = out_dir / f"{base}_summary_turns{args.turns}.csv"

    write_csv(rows_off, csv_off)
    write_csv(rows_on, csv_on)

    metrics_off = compute_metrics(rows_off)
    metrics_on = compute_metrics(rows_on)

    summary_rows = [
        {"diag_enabled": 0, **metrics_off},
        {"diag_enabled": 1, **metrics_on},
    ]

    write_summary(summary_rows, summary_path)
    write_compare_plot(rows_off, rows_on, metrics_off, metrics_on, plot_path)

    print(f"Saved CSV (diag off): {csv_off}")
    print(f"Saved CSV (diag on):  {csv_on}")
    print(f"Saved summary:        {summary_path}")
    print(f"Saved plot:           {plot_path}")
    print("Comparison (turns=2):")
    print(
        "  diag OFF -> wrap spikes "
        f"{metrics_off['wrap_spike_count']} / {max(1, int(metrics_off['samples']) - 1)}"
    )
    print(
        "  diag ON  -> wrap spikes "
        f"{metrics_on['wrap_spike_count']} / {max(1, int(metrics_on['samples']) - 1)}"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
