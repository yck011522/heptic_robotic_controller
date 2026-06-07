import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable, Sequence

import serial


ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from device_discovery import discover_devices, list_candidate_ports


DEFAULT_BAUD = 230400
DEFAULT_DEVICE_ID = 0
DEFAULT_TIMEOUT = 1.5
DEFAULT_TELEMETRY_TIMEOUT = 8.0
DEFAULT_PLATFORMIO = Path.home() / ".platformio" / "penv" / "Scripts" / "platformio.exe"
DEFAULT_RESULTS_DIR = Path(__file__).resolve().parent / "results"
MARKDOWN_PREVIEW_ROWS = 20


def add_common_arguments(parser: argparse.ArgumentParser, include_platformio: bool = False) -> None:
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port to open. If omitted, auto-discovers the requested dial_id.",
    )
    parser.add_argument(
        "--device-id",
        type=int,
        default=DEFAULT_DEVICE_ID,
        help="Dial ID to auto-discover when --port is omitted.",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    if include_platformio:
        parser.add_argument(
            "--platformio",
            default=str(DEFAULT_PLATFORMIO),
            help="Path to platformio executable",
        )
    parser.add_argument(
        "--result-format",
        choices=("markdown", "csv"),
        default="markdown",
        help="Result file format written beside the test output.",
    )
    parser.add_argument(
        "--result-dir",
        default=str(DEFAULT_RESULTS_DIR),
        help="Directory where result files are written.",
    )


def resolve_test_port(
    port: str | None,
    baud: int,
    device_id: int = DEFAULT_DEVICE_ID,
    allow_candidate_fallback: bool = False,
) -> str:
    if port:
        return port

    devices = discover_devices(baud=baud)
    matches = [device for device in devices if device["dial_id"] == device_id]
    if len(matches) == 1:
        return matches[0]["port"]
    if len(matches) > 1:
        ports = ", ".join(device["port"] for device in matches)
        raise RuntimeError(f"Multiple devices found for dial_id={device_id}: {ports}")

    if allow_candidate_fallback:
        candidates = list_candidate_ports(filter_by_vid_pid=True)
        if len(candidates) == 1:
            return candidates[0]["port"]

    discovered = ", ".join(f"{device['port']}(dial_id={device['dial_id']})" for device in devices) or "none"
    raise RuntimeError(
        f"No device found for dial_id={device_id}. Discovered devices: {discovered}."
    )


def open_serial(port: str, baud: int, settle_delay: float = 0.2) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(settle_delay)
    ser.reset_input_buffer()
    return ser


def read_line_until(ser: serial.Serial, predicate, timeout: float):
    deadline = time.monotonic() + timeout
    buffer = ""

    while time.monotonic() < deadline:
        data = ser.read(ser.in_waiting or 1)
        if not data:
            continue

        buffer += data.decode(errors="ignore")
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if predicate(line):
                return line

    raise TimeoutError("Timed out waiting for serial response")


def wait_for_prefix(ser: serial.Serial, prefix: str, timeout: float = DEFAULT_TIMEOUT) -> str:
    return read_line_until(ser, lambda line: line.startswith(prefix), timeout)


def send_command(ser: serial.Serial, command: str) -> None:
    ser.write((command + "\n").encode())


def send_and_wait(ser: serial.Serial, command: str, prefix: str, timeout: float = DEFAULT_TIMEOUT) -> str:
    ser.reset_input_buffer()
    send_command(ser, command)
    return wait_for_prefix(ser, prefix, timeout)


def parse_telemetry(line: str) -> dict[str, int]:
    parts = line.split(",")
    if len(parts) != 8 or parts[0] != "T":
        raise ValueError(f"Unexpected telemetry line: {line}")

    return {
        "dial_id": int(parts[1]),
        "seq": int(parts[2]),
        "angle_decideg": int(parts[3]),
        "speed_decideg_s": int(parts[4]),
        "torque_ma": int(parts[5]),
        "foc_rate_hz": int(parts[6]),
        "status_bits": int(parts[7]),
    }


def wait_for_telemetry(ser: serial.Serial, timeout: float = DEFAULT_TELEMETRY_TIMEOUT) -> dict[str, int]:
    line = wait_for_prefix(ser, "T,", timeout)
    return parse_telemetry(line)


def run_platformio(platformio_exe: str, args: list[str]) -> None:
    subprocess.run([platformio_exe, *args], check=True)


def calculate_quantile(values: Sequence[float], percentile: float) -> float:
    if not values:
        raise ValueError("Cannot calculate quantile for empty input")

    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]

    position = (len(ordered) - 1) * percentile
    lower_index = int(position)
    upper_index = min(lower_index + 1, len(ordered) - 1)
    weight = position - lower_index
    return ordered[lower_index] * (1.0 - weight) + ordered[upper_index] * weight


def build_summary_rows(port: str, extra_rows: Iterable[tuple[str, Any]]) -> list[tuple[str, Any]]:
    rows = [
        ("port", port),
        ("generated_utc", time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())),
    ]
    rows.extend(extra_rows)
    return rows


def write_test_result(
    script_path: str,
    result_format: str,
    result_dir: str,
    summary_rows: Sequence[tuple[str, Any]],
    sections: Sequence[tuple[str, Sequence[dict[str, Any]]]] = (),
) -> Path:
    output_dir = Path(result_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    extension = ".md" if result_format == "markdown" else ".csv"
    output_path = output_dir / f"{Path(script_path).stem}{extension}"

    if result_format == "markdown":
        _write_markdown_result(output_path, script_path, summary_rows, sections)
    else:
        _write_csv_result(output_path, summary_rows, sections)

    return output_path


def assert_true(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def _write_markdown_result(
    output_path: Path,
    script_path: str,
    summary_rows: Sequence[tuple[str, Any]],
    sections: Sequence[tuple[str, Sequence[dict[str, Any]]]],
) -> None:
    lines = [f"# {Path(script_path).stem}", "", "## Summary", "", "| Metric | Value |", "| --- | --- |"]

    for key, value in summary_rows:
        lines.append(f"| {_escape_markdown(str(key))} | {_escape_markdown(_serialize_value(value))} |")

    for title, rows in sections:
        lines.extend(["", f"## {title}", "", f"Rows collected: {len(rows)}", ""])
        if not rows:
            lines.append("(none)")
            continue

        preview_rows = list(rows[:MARKDOWN_PREVIEW_ROWS])
        if len(rows) > len(preview_rows):
            lines.append(f"Showing first {len(preview_rows)} rows only. Use CSV output for the full dataset.")
            lines.append("")

        headers = _ordered_headers(preview_rows)
        lines.append("| " + " | ".join(_escape_markdown(header) for header in headers) + " |")
        lines.append("| " + " | ".join("---" for _ in headers) + " |")
        for row in preview_rows:
            lines.append(
                "| " + " | ".join(_escape_markdown(_serialize_value(row.get(header, ""))) for header in headers) + " |"
            )

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_csv_result(
    output_path: Path,
    summary_rows: Sequence[tuple[str, Any]],
    sections: Sequence[tuple[str, Sequence[dict[str, Any]]]],
) -> None:
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["section", "row", "key", "value"])
        for key, value in summary_rows:
            writer.writerow(["summary", "", key, _serialize_value(value)])
        for title, rows in sections:
            for index, row in enumerate(rows):
                for key, value in row.items():
                    writer.writerow([title, index, key, _serialize_value(value)])


def _ordered_headers(rows: Sequence[dict[str, Any]]) -> list[str]:
    headers: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in headers:
                headers.append(key)
    return headers


def _serialize_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.6f}".rstrip("0").rstrip(".")
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value)
    return str(value)


def _escape_markdown(text: str) -> str:
    return text.replace("|", "\\|").replace("\n", "<br>")
