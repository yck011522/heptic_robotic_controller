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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Trigger debug encoder capture(s), save CSV, and generate plots."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port (default: COM9)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    parser.add_argument("--seq-start", type=int, default=1, help="Starting sequence number")
    parser.add_argument(
        "--turns",
        default="0,1,2",
        help="Comma-separated turn list (allowed values: 0,1,2)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Maximum seconds to wait for completion of each capture",
    )
    parser.add_argument(
        "--output-prefix",
        default="encoder_capture",
        help="Output file prefix",
    )
    parser.add_argument(
        "--inter-run-delay",
        type=float,
        default=0.4,
        help="Delay in seconds between captures",
    )
    parser.add_argument(
        "--startup-settle",
        type=float,
        default=1.0,
        help="Seconds to wait after opening serial before first command",
    )
    return parser.parse_args()


def parse_turns_arg(turns_arg: str) -> list[int]:
    turns: list[int] = []
    for token in turns_arg.split(","):
        value = token.strip()
        if not value:
            continue

        try:
            turns_value = int(value)
        except ValueError as exc:
            raise ValueError(f"Invalid turns value: {value}") from exc

        if turns_value not in (0, 1, 2):
            raise ValueError(f"Turns must be one of 0,1,2 (got {turns_value})")

        turns.append(turns_value)

    if not turns:
        raise ValueError("No valid turns provided")

    return turns


def read_capture_lines(ser: serial.Serial, seq: int, turns: int, timeout_s: float) -> list[str]:
    deadline = time.time() + timeout_s
    capture_lines: list[str] = []

    saw_start = False
    saw_done = False
    in_log = False

    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        print(line)

        if line.startswith(f"R,{seq},START"):
            saw_start = True
            continue

        if line.startswith(f"R,{seq},DONE"):
            saw_done = True
            continue

        if line.startswith(f"R,{seq},ERR"):
            raise RuntimeError(f"Firmware rejected command: {line}")

        if line.startswith("LOG_BEGIN,"):
            in_log = True
            capture_lines = [line]
            continue

        if in_log:
            capture_lines.append(line)
            if line.startswith("LOG_END,"):
                return capture_lines

    state = f"turns={turns}, start={saw_start}, done={saw_done}, lines={len(capture_lines)}"
    raise TimeoutError(f"Timed out waiting for full log stream ({state})")


def parse_log_lines(lines: list[str], turns: int) -> list[dict[str, float | int]]:
    rows: list[dict[str, float | int]] = []

    for line in lines:
        if line.startswith("LOG_DATA,"):
            payload = line.split(",", 1)[1]
            parts = payload.split(",")
        else:
            if line.startswith("LOG_BEGIN,") or line.startswith("LOG_HEADER,") or line.startswith("LOG_END,"):
                continue
            parts = line.split(",")

        if len(parts) not in (4, 7):
            raise ValueError(f"Unexpected data row format: {line}")

        try:
            t_us = int(parts[0])
            openloop_el = float(parts[1])
            sensor_raw = int(parts[2])
            sensor_error = int(parts[3])
            status = int(parts[4]) if len(parts) == 7 else -1
            agc = int(parts[5]) if len(parts) == 7 else -1
            magnitude = int(parts[6]) if len(parts) == 7 else -1
        except ValueError:
            continue

        rows.append(
            {
                "t_us": t_us,
                "openloop_el_angle_rad": openloop_el,
                "sensor_raw": sensor_raw,
                "sensor_error": sensor_error,
                "status": status,
                "agc": agc,
                "magnitude": magnitude,
                "turns": turns,
            }
        )

    if not rows:
        raise ValueError("No data rows parsed from log output")

    return rows


def write_csv(rows: list[dict[str, float | int]], csv_path: Path) -> None:
    fieldnames = [
        "t_us",
        "openloop_el_angle_rad",
        "sensor_raw",
        "sensor_error",
        "status",
        "agc",
        "magnitude",
        "turns",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def compute_metrics(rows: list[dict[str, float | int]]) -> dict[str, float | int]:
    raw = [int(row["sensor_raw"]) for row in rows]
    deltas = [abs(raw[i] - raw[i - 1]) for i in range(1, len(raw))]
    spike_count = sum(1 for value in deltas if value > 100)
    spike_rate_pct = (100.0 * spike_count / len(deltas)) if deltas else 0.0

    return {
        "samples": len(rows),
        "spike_count_gt100": spike_count,
        "spike_rate_pct": spike_rate_pct,
        "sensor_error_count": sum(int(row["sensor_error"]) for row in rows),
        "count_raw_0": sum(1 for value in raw if value == 0),
        "count_raw_4095": sum(1 for value in raw if value == 4095),
    }


def write_plot(rows: list[dict[str, float | int]], plot_path: Path, turns: int, metrics: dict[str, float | int]) -> None:
    t_ms = [float(row["t_us"]) / 1000.0 for row in rows]
    openloop = [float(row["openloop_el_angle_rad"]) for row in rows]
    raw = [int(row["sensor_raw"]) for row in rows]
    err = [int(row["sensor_error"]) for row in rows]
    status = [int(row["status"]) for row in rows]
    agc = [int(row["agc"]) for row in rows]
    magnitude = [int(row["magnitude"]) for row in rows]

    md = [((value >> 5) & 0x01) if value >= 0 else 0 for value in status]
    ml = [((value >> 4) & 0x01) if value >= 0 else 0 for value in status]
    mh = [((value >> 3) & 0x01) if value >= 0 else 0 for value in status]

    fig, axes = plt.subplots(6, 1, figsize=(12, 14), sharex=True)

    axes[0].plot(t_ms, raw, linewidth=1.0)
    axes[0].set_ylabel("AS5600 Raw")
    axes[0].set_title(
        f"Encoder Capture (turns={turns}) | spikes>100={metrics['spike_rate_pct']:.2f}% | "
        f"raw0={metrics['count_raw_0']} raw4095={metrics['count_raw_4095']}"
    )
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_ms, openloop, linewidth=1.0)
    axes[1].set_ylabel("Openloop El (rad)")
    axes[1].grid(True, alpha=0.3)

    axes[2].step(t_ms, err, where="post")
    axes[2].set_ylabel("I2C Error")
    axes[2].set_ylim(-0.1, 1.1)
    axes[2].grid(True, alpha=0.3)

    axes[3].step(t_ms, md, where="post", label="MD", linewidth=1.0)
    axes[3].step(t_ms, ml, where="post", label="ML", linewidth=1.0)
    axes[3].step(t_ms, mh, where="post", label="MH", linewidth=1.0)
    axes[3].set_ylabel("Status Bits")
    axes[3].set_ylim(-0.1, 1.1)
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.3)

    axes[4].plot(t_ms, agc, linewidth=1.0)
    axes[4].set_ylabel("AGC")
    axes[4].grid(True, alpha=0.3)

    axes[5].plot(t_ms, magnitude, linewidth=1.0)
    axes[5].set_ylabel("Magnitude")
    axes[5].set_xlabel("Time (ms)")
    axes[5].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)


def write_summary(summary_rows: list[dict[str, float | int]], summary_path: Path) -> None:
    fieldnames = [
        "turns",
        "samples",
        "spike_count_gt100",
        "spike_rate_pct",
        "sensor_error_count",
        "count_raw_0",
        "count_raw_4095",
    ]
    with summary_path.open("w", newline="", encoding="utf-8") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)


def main() -> int:
    args = parse_args()

    try:
        turns_list = parse_turns_arg(args.turns)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    out_dir = Path(__file__).resolve().parent
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = f"{args.output_prefix}_{stamp}"

    summary_rows: list[dict[str, float | int]] = []

    try:
        with serial.Serial(args.port, args.baud, timeout=0.25) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            if args.startup_settle > 0:
                time.sleep(args.startup_settle)
                ser.reset_input_buffer()

            for index, turns in enumerate(turns_list):
                seq = args.seq_start + index
                cmd = f"R,{seq},{turns}\n".encode("ascii")
                ser.write(cmd)
                ser.flush()

                lines = read_capture_lines(ser, seq, turns, args.timeout)
                rows = parse_log_lines(lines, turns)

                csv_path = out_dir / f"{base}_turns{turns}.csv"
                plot_path = out_dir / f"{base}_turns{turns}.png"

                write_csv(rows, csv_path)
                metrics = compute_metrics(rows)
                write_plot(rows, plot_path, turns, metrics)

                summary_row = {"turns": turns}
                summary_row.update(metrics)
                summary_rows.append(summary_row)

                print(f"Saved CSV: {csv_path}")
                print(f"Saved plot: {plot_path}")
                print(
                    f"Turns {turns}: spikes>100={metrics['spike_count_gt100']} "
                    f"({metrics['spike_rate_pct']:.2f}%)"
                )

                if index != len(turns_list) - 1 and args.inter_run_delay > 0:
                    time.sleep(args.inter_run_delay)
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    summary_path = out_dir / f"{base}_summary.csv"
    write_summary(summary_rows, summary_path)

    print(f"Saved summary: {summary_path}")
    print("Spike-rate comparison:")
    for row in summary_rows:
        print(
            f"  turns={row['turns']} -> {row['spike_rate_pct']:.2f}% "
            f"(count={row['spike_count_gt100']}, raw0={row['count_raw_0']}, raw4095={row['count_raw_4095']})"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
