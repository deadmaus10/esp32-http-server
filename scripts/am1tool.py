#!/usr/bin/env python3
"""Offline utilities for ESP32 measurement capture files.

This script can inspect and convert `.am1` capture files produced by the
firmware. It understands the packed binary header and frame layout and uses
NumPy for efficient decoding.

Usage examples:
    python3 scripts/am1tool.py info path/to/file.am1
    python3 scripts/am1tool.py csv path/to/file.am1 --cols full --output file.csv
"""
from __future__ import annotations

import argparse
import datetime as _dt
import os
import sys
from typing import Iterable, List, Tuple

import numpy as np

HEADER_DTYPE = np.dtype(
    [
        ("magic", "S4"),
        ("ver", "<u2"),
        ("reserved", "<u2"),
        ("start_epoch", "<u4"),
        ("time_scale_us", "<u4"),
        ("sps0", "<u2"),
        ("sps1", "<u2"),
        ("gain0_code", "u1"),
        ("gain1_code", "u1"),
        ("sh0", "<f4"),
        ("sh1", "<f4"),
        ("fs0", "<f4"),
        ("fs1", "<f4"),
        ("off0", "<f4"),
        ("off1", "<f4"),
    ],
    align=False,
)

FRAME_DTYPE = np.dtype(
    [
        ("t_10us", "<u4"),
        ("raw0", "<i2"),
        ("raw1", "<i2"),
    ],
    align=False,
)

HEADER_SIZE = HEADER_DTYPE.itemsize
FRAME_SIZE = FRAME_DTYPE.itemsize

GAIN_TABLE = {
    0: ("2/3x", 0.1875),
    1: ("1x", 0.1250),
    2: ("2x", 0.0625),
    3: ("4x", 0.03125),
    4: ("8x", 0.015625),
    5: ("16x", 0.0078125),
}


class Am1File:
    """Helper to expose header fields and NumPy views of AM01 captures."""

    def __init__(self, path: str):
        self.path = path
        raw = np.memmap(path, dtype=np.uint8, mode="r")
        if raw.size < HEADER_SIZE:
            raise ValueError("file too small to contain header")
        header_buf = raw[:HEADER_SIZE].tobytes()
        self.header = np.frombuffer(header_buf, dtype=HEADER_DTYPE, count=1)[0]
        if self.header["magic"] != b"AM01":
            raise ValueError("unexpected magic bytes; not an AM01 capture")
        if self.header["ver"] != 1:
            raise ValueError(f"unsupported header version: {self.header['ver']}")

        payload = raw[HEADER_SIZE:]
        if payload.size % FRAME_SIZE:
            raise ValueError("payload size is not a multiple of frame size")
        self.frames = payload.view(FRAME_DTYPE)

    @property
    def sample_count(self) -> int:
        return int(self.frames.shape[0])

    @property
    def duration_seconds(self) -> float:
        if self.sample_count == 0:
            return 0.0
        t_last = float(self.frames["t_10us"][-1])
        return t_last * (self.header["time_scale_us"] / 1_000_000.0)

    def gain_info(self, channel: int) -> Tuple[str, float]:
        code = int(self.header[f"gain{channel}_code"])
        return GAIN_TABLE.get(code, (f"unknown({code})", np.nan))

    def lsb_mv(self, channel: int) -> float:
        return self.gain_info(channel)[1]

    def shunt_ohms(self, channel: int) -> float:
        return float(self.header[f"sh{channel}"])

    def fullscale_mm(self, channel: int) -> float:
        return float(self.header[f"fs{channel}"])

    def offset_mm(self, channel: int) -> float:
        return float(self.header[f"off{channel}"])

    def start_datetime(self) -> _dt.datetime:
        return _dt.datetime.utcfromtimestamp(int(self.header["start_epoch"]))


def cmd_info(am1: Am1File) -> None:
    hdr = am1.header
    dt = am1.start_datetime()
    duration = am1.duration_seconds
    count = am1.sample_count
    filesize = os.path.getsize(am1.path)

    print(f"File:       {am1.path}")
    print(f"Size:       {filesize} bytes")
    print(f"Samples:    {count}")
    print(f"Duration:   {duration:.3f} s")
    if duration > 0 and count > 1:
        rate = (count - 1) / duration
        print(f"Pair rate:  {rate:.2f} Hz (approx)")
    else:
        print("Pair rate:  n/a")
    print(f"Start UTC:  {dt.isoformat()} (epoch {int(hdr['start_epoch'])})")
    print(f"Time unit:  {int(hdr['time_scale_us'])} µs per tick")

    for ch in (0, 1):
        gain_label, lsb = am1.gain_info(ch)
        print(f"\nChannel {ch}:")
        print(f"  SPS target: {int(hdr[f'sps{ch}'])}")
        print(f"  Gain:       code {int(hdr[f'gain{ch}_code'])} ({gain_label}, {lsb:.6f} mV/LSB)")
        print(f"  Shunt:      {am1.shunt_ohms(ch):.3f} Ω")
        print(f"  Full scale: {am1.fullscale_mm(ch):.3f} mm")
        print(f"  Offset:     {am1.offset_mm(ch):.3f} mm")


def compute_columns(am1: Am1File, cols: str) -> Tuple[np.ndarray, List[str], List[str]]:
    frames = am1.frames
    hdr = am1.header
    if frames.size == 0:
        return np.empty((0, 3)), ["time_s", "raw0", "raw1"], ["%.6f", "%d", "%d"]

    time_scale = hdr["time_scale_us"] / 1_000_000.0
    times = frames["t_10us"].astype(np.float64) * time_scale
    raw0 = frames["raw0"].astype(np.int32)
    raw1 = frames["raw1"].astype(np.int32)

    columns: List[np.ndarray] = [times, raw0, raw1]
    headers = ["time_s", "raw0", "raw1"]
    formats = ["%.6f", "%d", "%d"]

    if cols in {"rawmv", "full"}:
        mv0 = raw0.astype(np.float64) * am1.lsb_mv(0)
        mv1 = raw1.astype(np.float64) * am1.lsb_mv(1)
        columns.extend([mv0, mv1])
        headers.extend(["mv0", "mv1"])
        formats.extend(["%.3f", "%.3f"])
    else:
        mv0 = mv1 = None  # for type checking

    if cols == "full":
        sh0 = am1.shunt_ohms(0)
        sh1 = am1.shunt_ohms(1)
        mv0 = columns[3] if mv0 is None else mv0
        mv1 = columns[4] if mv1 is None else mv1
        ma0 = np.where(sh0 > 0.1, mv0 / sh0, 0.0)
        ma1 = np.where(sh1 > 0.1, mv1 / sh1, 0.0)
        pct0 = np.clip((ma0 - 4.0) / 16.0 * 100.0, 0.0, 100.0)
        pct1 = np.clip((ma1 - 4.0) / 16.0 * 100.0, 0.0, 100.0)
        mm0 = pct0 / 100.0 * am1.fullscale_mm(0) + am1.offset_mm(0)
        mm1 = pct1 / 100.0 * am1.fullscale_mm(1) + am1.offset_mm(1)
        columns.extend([ma0, ma1, mm0, mm1])
        headers.extend(["ma0", "ma1", "mm0", "mm1"])
        formats.extend(["%.3f", "%.3f", "%.2f", "%.2f"])

    stacked = np.column_stack(columns) if columns else np.empty((0, 0))
    return stacked, headers, formats


def cmd_csv(am1: Am1File, out_path: str, cols: str) -> None:
    data, headers, formats = compute_columns(am1, cols)
    out_stream = sys.stdout if out_path == "-" else open(out_path, "w", encoding="utf-8")
    try:
        header_line = ",".join(headers)
        np.savetxt(out_stream, data, fmt=formats, delimiter=",", header=header_line, comments="")
    finally:
        if out_stream is not sys.stdout:
            out_stream.close()


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect or convert AM01 measurement captures.")
    sub = parser.add_subparsers(dest="command", required=True)

    p_info = sub.add_parser("info", help="Print header and timing details")
    p_info.add_argument("path", help="Path to .am1 capture")

    p_csv = sub.add_parser("csv", help="Convert capture to CSV")
    p_csv.add_argument("path", help="Path to .am1 capture")
    p_csv.add_argument("--cols", choices=["full", "rawmv", "raw"], default="full", help="Column set to export")
    p_csv.add_argument("--output", "-o", default="-", help="Output CSV path (default: stdout)")

    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    try:
        am1 = Am1File(args.path)
    except Exception as exc:  # noqa: BLE001
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if args.command == "info":
        cmd_info(am1)
    elif args.command == "csv":
        try:
            cmd_csv(am1, args.output, args.cols)
        except Exception as exc:  # noqa: BLE001
            print(f"error: {exc}", file=sys.stderr)
            return 1
    else:  # pragma: no cover - argparse enforces choices
        raise AssertionError(f"unknown command {args.command}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())