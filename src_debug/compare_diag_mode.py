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
DEFAULT_MODES = "0,1,2,3"


def mode_label(mode: int) -> str:
    labels = {
        0: "raw_only",
        1: "status_raw",
        2: "status_raw_agc",
        3: "status_raw_agc_mag",
    }
    return labels.get(mode, f"mode_{mode}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare encoder capture across firmware read modes (D=0..3)."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    parser.add_argument("--turns", type=int, default=DEFAULT_TURNS, help="Turns for each run")
    parser.add_argument(
        "--modes",
        default=DEFAULT_MODES,
        help="Comma-separated mode list from {0,1,2,3}",
    )
    parser.add_argument("--seq-start", type=int, default=100, help="Starting sequence number")
    parser.add_argument("--timeout", type=float, default=20.0, help="Capture timeout seconds")
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
        help="Delay between mode runs",
    )
    parser.add_argument(
        "--output-prefix",
        default="encoder_mode_compare",
        help="Output file prefix",
    )
    parser.add_argument(
        "--spike-threshold",
        type=int,
        default=100,
        help="Absolute step threshold for spike counting",
    )
    return parser.parse_args()


def parse_modes(modes_arg: str) -> list[int]:
    modes: list[int] = []
    for token in modes_arg.split(","):
        text = token.strip()
        if not text:
            continue

        try:
            mode = int(text)
        except ValueError as exc:
            raise ValueError(f"Invalid mode: {text}") from exc

        if mode not in (0, 1, 2, 3):
            raise ValueError(f"Mode must be one of 0,1,2,3 (got {mode})")

        modes.append(mode)

    if not modes:
        raise ValueError("No valid modes provided")

    return modes


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


def send_diag_mode(ser: serial.Serial, seq: int, mode: int, timeout_s: float) -> None:
    cmd = f"D,{seq},{mode}\n".encode("ascii")
    ser.write(cmd)
    ser.flush()

    def match(line: str) -> bool:
        if line.startswith(f"D,{seq},ERR"):
            raise RuntimeError(f"Firmware rejected D command: {line}")
        return line == f"D,{seq},{mode}"

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


def parse_log_lines(lines: list[str], mode: int, turns: int) -> list[dict[str, int | float]]:
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
                    "mode": mode,
                    "mode_label": mode_label(mode),
                    "turns": turns,
                }
            )
        except ValueError:
            continue

    if not rows:
        raise ValueError("No LOG_DATA rows parsed")

    return rows


def write_csv(rows: list[dict[str, int | float | str]], path: Path) -> None:
    fields = [
        "t_us",
        "openloop_el_angle_rad",
        "sensor_raw",
        "sensor_error",
        "status",
        "agc",
        "magnitude",
        "mode",
        "mode_label",
        "turns",
    ]
    with path.open("w", newline="", encoding="utf-8") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def wrap_aware_steps(raw: list[int]) -> list[int]:
    out: list[int] = []
    for index in range(1, len(raw)):
        delta = raw[index] - raw[index - 1]
        if delta > 2048:
            delta -= 4096
        elif delta < -2048:
            delta += 4096
        out.append(delta)
    return out


def compute_metrics(rows: list[dict[str, int | float]], spike_thr: int) -> dict[str, int | float]:
    raw = [int(row["sensor_raw"]) for row in rows]
    naive_steps = [abs(raw[index] - raw[index - 1]) for index in range(1, len(raw))]
    wrap_steps = [abs(value) for value in wrap_aware_steps(raw)]

    return {
        "samples": len(rows),
        "sensor_error_count": sum(int(row["sensor_error"]) for row in rows),
        "count_raw_0": sum(1 for value in raw if value == 0),
        "count_raw_4095": sum(1 for value in raw if value == 4095),
        "naive_spike_count": sum(1 for value in naive_steps if value > spike_thr),
        "wrap_spike_count": sum(1 for value in wrap_steps if value > spike_thr),
        "wrap_step_max": max(wrap_steps) if wrap_steps else 0,
    }


def write_summary(summary_rows: list[dict[str, int | float | str]], path: Path) -> None:
    fields = [
        "mode",
        "mode_label",
        "samples",
        "sensor_error_count",
        "count_raw_0",
        "count_raw_4095",
        "naive_spike_count",
        "wrap_spike_count",
        "wrap_step_max",
    ]
    with path.open("w", newline="", encoding="utf-8") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summary_rows)


def write_compare_plot(
    rows_by_mode: dict[int, list[dict[str, int | float]]],
    summary_rows: list[dict[str, int | float | str]],
    turns: int,
    spike_thr: int,
    path: Path,
) -> None:
    mode_order = [int(row["mode"]) for row in summary_rows]

    fig, axes = plt.subplots(len(mode_order) + 2, 1, figsize=(12, 3.0 * (len(mode_order) + 2)), sharex=False)

    for plot_idx, mode in enumerate(mode_order):
        rows = rows_by_mode[mode]
        t_ms = [float(r["t_us"]) / 1000.0 for r in rows]
        raw = [int(r["sensor_raw"]) for r in rows]
        axes[plot_idx].plot(t_ms, raw, linewidth=1.0)
        axes[plot_idx].set_ylabel(f"Raw (m{mode})")
        axes[plot_idx].set_title(mode_label(mode))
        axes[plot_idx].grid(True, alpha=0.3)

    ax_steps = axes[len(mode_order)]
    for mode in mode_order:
        rows = rows_by_mode[mode]
        t_ms = [float(r["t_us"]) / 1000.0 for r in rows]
        raw = [int(r["sensor_raw"]) for r in rows]
        step = [abs(value) for value in wrap_aware_steps(raw)]
        ax_steps.plot(t_ms[1:], step, linewidth=1.0, label=f"m{mode}")

    ax_steps.axhline(spike_thr, color="red", linestyle="--", linewidth=0.8, label="spike threshold")
    ax_steps.set_ylabel("|Δraw| wrap-aware")
    ax_steps.set_title("Step magnitude comparison")
    ax_steps.legend(loc="upper right")
    ax_steps.grid(True, alpha=0.3)

    ax_bar = axes[len(mode_order) + 1]
    labels = [mode_label(mode) for mode in mode_order]
    wrap_spikes = [int(row["wrap_spike_count"]) for row in summary_rows]
    ax_bar.bar(labels, wrap_spikes)
    ax_bar.set_ylabel("Wrap spikes")
    ax_bar.set_title(f"Spike count summary (turns={turns})")
    ax_bar.grid(True, axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main() -> int:
    args = parse_args()

    try:
        modes = parse_modes(args.modes)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    out_dir = Path(__file__).resolve().parent
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = f"{args.output_prefix}_{stamp}"

    seq = args.seq_start
    rows_by_mode: dict[int, list[dict[str, int | float]]] = {}

    try:
        with serial.Serial(args.port, args.baud, timeout=0.25) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            if args.startup_settle > 0:
                time.sleep(args.startup_settle)
                ser.reset_input_buffer()

            for mode_index, mode in enumerate(modes):
                send_diag_mode(ser, seq, mode, timeout_s=args.timeout)
                seq += 1

                lines = read_capture_lines(ser, seq, turns=args.turns, timeout_s=args.timeout)
                seq += 1

                rows = parse_log_lines(lines, mode=mode, turns=args.turns)
                rows_by_mode[mode] = rows

                csv_path = out_dir / f"{base}_mode{mode}_turns{args.turns}.csv"
                write_csv(rows, csv_path)
                print(f"Saved CSV (mode {mode}): {csv_path}")

                if mode_index != len(modes) - 1 and args.inter_run_delay > 0:
                    time.sleep(args.inter_run_delay)

    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    summary_rows: list[dict[str, int | float | str]] = []
    for mode in modes:
        metrics = compute_metrics(rows_by_mode[mode], spike_thr=args.spike_threshold)
        summary_rows.append(
            {
                "mode": mode,
                "mode_label": mode_label(mode),
                **metrics,
            }
        )

    summary_path = out_dir / f"{base}_summary_turns{args.turns}.csv"
    plot_path = out_dir / f"{base}_compare_turns{args.turns}.png"

    write_summary(summary_rows, summary_path)
    write_compare_plot(rows_by_mode, summary_rows, turns=args.turns, spike_thr=args.spike_threshold, path=plot_path)

    print(f"Saved summary: {summary_path}")
    print(f"Saved plot:    {plot_path}")
    print("Comparison summary:")
    for row in summary_rows:
        print(
            f"  mode={row['mode']} ({row['mode_label']}) -> "
            f"wrap_spikes={row['wrap_spike_count']} "
            f"naive_spikes={row['naive_spike_count']} "
            f"raw0={row['count_raw_0']} raw4095={row['count_raw_4095']}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
