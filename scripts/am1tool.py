#!/usr/bin/env python3
# am1python.py — ultra-fast .am1 info/CSV tool (NumPy memmap)
#
# source venv/bin/activate then pyhton3 ...
#
# Commands:
#  info <file.am1>             # print header / quick stats
#  csv  <file.am1> [options]   # write CSV next to the input
#
# Options (for 'csv'):
#  --out OUT.csv            output path (default: <file>.csv)
#  --cols raw|rawmv|full    columns (default: full)
#  --step N                 decimate: keep every Nth frame (default: 1)
#  --limit N                stop after N rows (default: 0=all)
#  --start T                start at time T seconds (default: 0)
#  --end T                  end at time T seconds (default: +inf)
# Usage:
#   python3 am1tool.py info file.am1
#   python3 am1tool.py csv  file.am1 [--out OUT.csv] [--cols raw|rawmv|full]
#                               [--step N] [--limit N] [--start T] [--end T]
import sys, os, struct, argparse
import numpy as np

HDR_FMT = "<4sHHIIHHBBffffff"                     # 46 bytes, packed on ESP32
HDR_SIZE = struct.calcsize(HDR_FMT)
FRAME_DTYPE = np.dtype([("t10us","<u4"),("raw0","<i2"),("raw1","<i2")])  # 8B

def ads_lsb_mV(gcode:int)->float:
    table = [0.1875, 0.1250, 0.0625, 0.03125, 0.015625, 0.0078125]
    return table[gcode] if 0 <= gcode < len(table) else 0.1250

def read_header(fp):
    head = fp.read(64)
    if len(head) < HDR_SIZE:
        raise ValueError("file too small for AM01 header")
    magic, ver, res, start_epoch, ts_us, sps0, sps1, g0, g1, sh0, sh1, fs0, fs1, off0, off1 = \
        struct.unpack_from(HDR_FMT, head, 0)
    if magic != b"AM01":
        raise ValueError("bad magic (not AM01)")
    return dict(ver=ver, start_epoch=start_epoch, ts_us=ts_us,
                sps0=sps0, sps1=sps1, g0=g0, g1=g1,
                sh0=sh0, sh1=sh1, fs0=fs0, fs1=fs1, off0=off0, off1=off1,
                header_size=HDR_SIZE)

def mmap_frames(path, offset):
    size = os.path.getsize(path)
    if size < offset: raise ValueError("file truncated")
    nbytes = size - offset
    nbytes -= (nbytes % FRAME_DTYPE.itemsize)    # tolerate partial tail
    n = nbytes // FRAME_DTYPE.itemsize
    if n == 0: return np.empty((0,), dtype=FRAME_DTYPE)
    return np.memmap(path, dtype=FRAME_DTYPE, mode="r", offset=offset, shape=(n,))

def cmd_info(path):
    with open(path, "rb") as f:
        H = read_header(f)
    frames = (os.path.getsize(path) - H["header_size"]) // FRAME_DTYPE.itemsize
    dur_s = 0.0
    if frames > 0:
        fr = mmap_frames(path, H["header_size"])
        dur_s = float(fr[-1]["t10us"]) * (H["ts_us"]/1e6)
    lsb0, lsb1 = ads_lsb_mV(H["g0"]), ads_lsb_mV(H["g1"])
    print(f"File: {path}")
    print(f"Header: ver={H['ver']} ts_us={H['ts_us']} start_epoch={H['start_epoch']}")
    print(f"SPS: ch0={H['sps0']} ch1={H['sps1']}  GAIN: ch0={H['g0']} ch1={H['g1']}")
    print(f"Shunt Ω: ch0={H['sh0']} ch1={H['sh1']}  FS mm: ch0={H['fs0']} ch1={H['fs1']}  Off mm: ch0={H['off0']} ch1={H['off1']}")
    print(f"LSB mV: ch0={lsb0} ch1={lsb1}")
    print(f"Frames: {frames}  Duration: ~{dur_s:.2f} s")

def cmd_csv(path, out, cols, step, limit, tstart, tend):
    with open(path, "rb") as f:
        H = read_header(f)
    fr = mmap_frames(path, H["header_size"])
    if fr.size == 0:
        open(out, "w").close(); return

    t = fr["t10us"].astype(np.float64) * (H["ts_us"]/1e6)   # seconds
    mask = np.ones(fr.size, dtype=bool)
    if tstart is not None: mask &= (t >= tstart)
    if tend   is not None: mask &= (t <= tend)
    idx = np.nonzero(mask)[0]
    if step > 1: idx = idx[::step]
    if limit and idx.size > limit: idx = idx[:limit]

    r0 = fr["raw0"][idx].astype(np.int32)
    r1 = fr["raw1"][idx].astype(np.int32)
    tt = t[idx]

    if cols == "raw":
        arr = np.column_stack([tt, r0, r1])
        header = "t_s,raw0,raw1"; fmt = ["%.6f","%d","%d"]
    else:
        lsb0, lsb1 = ads_lsb_mV(H["g0"]), ads_lsb_mV(H["g1"])
        mv0 = r0 * lsb0; mv1 = r1 * lsb1
        if cols == "rawmv":
            arr = np.column_stack([tt, r0, r1, mv0, mv1])
            header = "t_s,raw0,raw1,mV0,mV1"; fmt = ["%.6f","%d","%d","%.3f","%.3f"]
        else:
            ma0 = np.where(H["sh0"]>0.1, mv0/H["sh0"], 0.0)
            ma1 = np.where(H["sh1"]>0.1, mv1/H["sh1"], 0.0)
            pct0 = np.clip((ma0-4.0)/16.0*100.0, 0.0, 100.0)
            pct1 = np.clip((ma1-4.0)/16.0*100.0, 0.0, 100.0)
            mm0 = pct0/100.0*H["fs0"] + H["off0"]
            mm1 = pct1/100.0*H["fs1"] + H["off1"]
            arr = np.column_stack([tt, r0, r1, mv0, mv1, ma0, ma1, mm0, mm1])
            header="t_s,raw0,raw1,mV0,mV1,mA0,mA1,mm0,mm1"
            fmt = ["%.6f","%d","%d","%.3f","%.3f","%.3f","%.3f","%.2f","%.2f"]

    with open(out, "w", buffering=1024*1024) as fo:
        fo.write(header + "\n")
        np.savetxt(fo, arr, delimiter=",", fmt=fmt)
    print(f"Wrote: {out}  rows={arr.shape[0]}")

def main():
    ap = argparse.ArgumentParser(description="AM01 .am1 tool")
    sub = ap.add_subparsers(dest="cmd")
    ap_i = sub.add_parser("info", help="show header")
    ap_i.add_argument("file")
    ap_c = sub.add_parser("csv", help="convert to CSV")
    ap_c.add_argument("file")
    ap_c.add_argument("--out", default=None)
    ap_c.add_argument("--cols", choices=["raw","rawmv","full"], default="full")
    ap_c.add_argument("--step", type=int, default=1)
    ap_c.add_argument("--limit", type=int, default=0)
    ap_c.add_argument("--start", type=float, default=None)
    ap_c.add_argument("--end", type=float, default=None)
    args = ap.parse_args()

    if args.cmd == "info":
        cmd_info(args.file); return
    if args.cmd == "csv":
        out = args.out or (os.path.splitext(args.file)[0] + ".csv")
        step  = max(1, int(args.step))
        limit = max(0, int(args.limit))
        cmd_csv(args.file, out, args.cols, step, limit, args.start, args.end); return
    ap.print_help()

if __name__ == "__main__":
    main()
