#!/usr/bin/env python3
"""SMPTE VITC as the GCV2 draws it (vcr_sim/src/streams.c vitc_make, vitc_line) and the DE1 reads it.

Commands:
  word FRAME [FIELD]   print the 90 bits
  stim                 lines of DE1 luma through a modelled channel, for chipset/sim/run.sh vitc
  score                compare the Questa run's decodes with the truth
  check                read the DE1's VITC probe over JTAG (--count, --interval, --reset)
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy import signal

ROOT = Path(__file__).resolve().parent.parent
QUARTUS_BIN = Path(os.environ.get("QUARTUS_BIN", ""))   # unset: quartus_stp on PATH
FS_DAC, SPL, X0, SPAN = 14318182.0, 910, 160, 716
FS_Y, Y_SAV = 13.5e6, 122                     # ADV7180 luma rate; first active sample after 0H


def ltc_data(frame: int, user: int = 0) -> list[int]:
    f, s, m, h = frame % 30, frame // 30 % 60, frame // 1800 % 60, frame // 108000 % 24
    d = [f % 10, f // 10, s % 10, s // 10, m % 10, m // 10, h % 10, h // 10]
    w = [d[k] | (user >> 4 * k & 15) << 4 for k in range(8)]
    return [w[i >> 3] >> (i & 7) & 1 for i in range(64)]


def word(frame: int, field: int, user: int = 0) -> list[int]:
    d = ltc_data(frame, user)
    d[27] = field & 1
    b = []
    for g in range(9):
        b += [1, 0] + (d[8 * g:8 * g + 8] if g < 8 else [0] * 8)
    col = 0
    for k in range(82):
        col ^= b[k] << (k & 7)
    for k in range(82, 90):
        b[k] = col >> (k & 7) & 1
    return b


def decode(b: list[int]) -> dict | None:
    col = 0
    for k, v in enumerate(b):
        col ^= v << (k & 7)
    if len(b) != 90 or col or any(b[10 * g] != 1 or b[10 * g + 1] != 0 for g in range(9)):
        return None
    d = [b[10 * g + 2 + i] for g in range(8) for i in range(8)]
    dig = [sum(d[8 * k + i] << i for i in range(4)) for k in range(8)]
    tens = [dig[1] & 3, dig[3] & 7, dig[5] & 7, dig[7] & 3]
    frame = ((tens[3] * 10 + dig[6]) * 60 + tens[2] * 10 + dig[4]) * 1800 + (tens[1] * 10 + dig[2]) * 30 \
        + tens[0] * 10 + dig[0]
    return {"frame": frame, "field": d[27]}


def ire_code(ire: float) -> int:
    return int(min(max(round((ire + 40) / 180 * 63), 0), 63))


def dac_line(b: list[int]) -> np.ndarray:
    """One line of DAC codes, as vitc_line draws it (blanking elsewhere; no sync)."""
    lo, hi = ire_code(0), ire_code(80)
    bb = b + [0]
    cov = np.zeros(SPAN + 1)
    for i in range(SPAN):
        a = 115 * i
        k = min(a // 910, 90)
        in_k = min(115, 910 * (k + 1) - a)
        cov[i] = in_k * bb[k] + (115 - in_k) * bb[min(k + 1, 90)]
    line = np.full(SPL, lo, float)
    prev = 0
    for i in range(SPAN):
        s = prev + 2 * cov[i] + cov[i + 1]
        line[X0 + i] = lo + ((hi - lo) * s + 230) // 460
        prev = cov[i]
    return line


def channel(dac: np.ndarray, rng, bw_hz: float, noise_ire: float, gain: float, shift_us: float) -> np.ndarray:
    """DAC codes -> IRE, low-pass, noise, gain, a timing shift, then 720 DE1 luma codes from SAV."""
    ire = dac / 63 * 180 - 40
    if bw_hz:
        sos = signal.butter(2, bw_hz, fs=FS_DAC, output="sos")
        ire = signal.sosfiltfilt(sos, ire)
    ire = ire * gain + rng.normal(0, noise_ire, ire.shape)
    t = (Y_SAV + np.arange(720)) / FS_Y - shift_us * 1e-6
    y_ire = np.interp(t * FS_DAC, np.arange(SPL), ire)
    return np.clip(np.round(16 + (y_ire - 7.5) * 219 / 92.5), 1, 254).astype(int)


def cmd_word(args) -> None:
    b = word(args.frame, args.field)
    print(" ".join("".join(map(str, b[10 * g:10 * g + 10])) for g in range(9)))
    print(decode(b))


def cmd_stim(args) -> None:
    rng = np.random.default_rng(args.seed)
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    truth, rows = [], []
    for n in range(args.lines):
        frame, field = int(rng.integers(0, 24 * 108000)), int(rng.integers(0, 2))
        gain = 1 + rng.uniform(-args.gain, args.gain)
        shift = rng.uniform(-args.shift, args.shift)
        y = channel(dac_line(word(frame, field)), rng, args.bw_khz * 1e3, args.noise, gain, shift)
        rows.append(" ".join(f"{v:02x}" for v in y))
        truth.append([frame, field])
    (out / "vitc_lines.txt").write_text("\n".join(rows) + "\n")
    (out / "vitc_params.vh").write_text(f"localparam integer NL = {args.lines};\n")
    (out / "vitc_truth.json").write_text(json.dumps({"truth": truth, "args": vars(args) | {"out_dir": str(out)}}))
    print(f"{args.lines} lines: bw {args.bw_khz} kHz, noise {args.noise} IRE, gain +-{args.gain}, "
          f"shift +-{args.shift} us")


def cmd_score(args) -> None:
    truth = json.loads((args.out_dir / "vitc_truth.json").read_text())["truth"]
    rows = [l.split() for l in (args.out_dir / "vitc_out.txt").read_text().splitlines() if l.strip()]
    good = wrong = 0
    tags = sum(r[2] == "1" for r in rows)
    for (frame, field), r in zip(truth, rows):
        if r[0] != "1":
            continue
        d = int(r[1], 16)
        bits = [d >> i & 1 for i in range(64)]
        dig = [sum(bits[8 * k + i] << i for i in range(4)) for k in range(8)]
        tens = [dig[1] & 3, dig[3] & 7, dig[5] & 7, dig[7] & 3]
        got = ((tens[3] * 10 + dig[6]) * 60 + tens[2] * 10 + dig[4]) * 1800 + (tens[1] * 10 + dig[2]) * 30 \
            + tens[0] * 10 + dig[0]
        if got == frame and bits[27] == field:
            good += 1
        else:
            wrong += 1
    print(f"{len(rows)} lines: {good} read ({100 * good / len(rows):.1f}%), {wrong} wrong; "
          f"tag_slicer took {tags} as tags")
    if wrong or tags:
        raise SystemExit(1)


PROBE = [("reads", 16), ("seq", 8), ("line", 5), ("age", 6), ("mark", 1), ("tc", 32), ("pos", 24), ("user", 32)]


def parse(bits: str) -> dict:
    out, i = {}, 0
    for name, w in PROBE:
        out[name] = int(bits[i:i + w], 2)
        i += w
    return out


def stp(*cmds: str) -> list[str]:
    r = subprocess.run([str(QUARTUS_BIN / "quartus_stp"), "-t", str(ROOT / "chipset/probe.tcl"), "DE-SoC",
                        "VITC", *cmds], capture_output=True, text=True)
    if r.returncode:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit("quartus_stp failed")
    return [l.split()[1] for l in r.stdout.splitlines() if l.startswith("BITS ")]


def tc_str(tc: int) -> str:
    return f"{tc >> 24:02x}:{tc >> 16 & 255:02x}:{tc >> 8 & 255:02x}:{tc & 255:02x}"


def cmd_check(args) -> None:
    if args.reset:
        stp("src 1", "src 0")
    t0, prev = time.monotonic(), None
    for k in range(args.count):
        if k:
            time.sleep(args.interval)
        s = parse(stp("read")[-1])
        t = time.monotonic()
        rate = f"  {(s['reads'] - prev[1]['reads']) / (t - prev[0]):6.1f} reads/s" if prev else ""
        print(f"{t - t0:6.1f} s  tc {tc_str(s['tc'])}.{s['mark'] + 1}  pos {s['pos']} (frame {s['pos'] >> 1})  "
              f"line {s['line']}  age {s['age']}  reads {s['reads']}{rate}", flush=True)
        prev = (t, s)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("word")
    p.add_argument("frame", type=int)
    p.add_argument("field", type=int, nargs="?", default=0)
    p = sub.add_parser("stim")
    p.add_argument("--out-dir", type=Path, default=ROOT / "chipset/sim/out")
    p.add_argument("--lines", type=int, default=400)
    p.add_argument("--bw-khz", type=float, default=3000)
    p.add_argument("--noise", type=float, default=0, help="IRE rms")
    p.add_argument("--gain", type=float, default=0, help="+- fraction")
    p.add_argument("--shift", type=float, default=0, help="+- us of line position")
    p.add_argument("--seed", type=int, default=1)
    p = sub.add_parser("score")
    p.add_argument("--out-dir", type=Path, default=ROOT / "chipset/sim/out")
    p = sub.add_parser("check")
    p.add_argument("--count", type=int, default=5)
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--reset", action="store_true", help="clear the read counter first")
    args = ap.parse_args()
    {"word": cmd_word, "stim": cmd_stim, "score": cmd_score, "check": cmd_check}[args.cmd](args)


if __name__ == "__main__":
    main()
