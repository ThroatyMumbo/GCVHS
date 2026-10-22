#!/usr/bin/env python3
"""Position tags as the GCV2 draws them (vcr_sim/src/streams.c ptag_make, ptag_line) and the DE1
reads them (chipset/src/ptag_slicer.v).

Commands:
  word POS             print one copy's 32 cells
  stim                 lines of DE1 luma through a modelled channel that keeps only a piece of each line,
                       as a paused or cueing deck does, plus garbage lines; for chipset/sim/run.sh ptag
  score                compare the Questa run's reads with the truth
  check                read the DE1's PTAG probe over JTAG
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vitc  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
QUARTUS_BIN = Path(os.environ.get("QUARTUS_BIN", ""))   # unset: quartus_stp on PATH
X0, CELL, CELLS, COPIES = 160, 7, 32, 3
Y_PER_CELL = CELL * vitc.FS_Y / vitc.FS_DAC


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def word(pos: int) -> list[int]:
    d = [pos & 255, pos >> 8 & 255]
    d.append(crc8(bytes(d)))
    c = []
    for g in range(3):
        c += [1, 0] + [d[g] >> i & 1 for i in range(8)]
    return c + [0, 0]


def dac_line(c: list[int]) -> np.ndarray:
    lo, hi = vitc.ire_code(0), vitc.ire_code(80)
    mid = (lo + hi + 1) // 2
    line = np.full(vitc.SPL, lo, float)
    prev = 0
    for n in range(COPIES * CELLS):
        b = c[n % CELLS]
        a = X0 + n * CELL
        line[a:a + CELL] = hi if b else lo
        if b != prev:
            line[a] = mid
        prev = b
    return line


def cmd_word(args) -> None:
    c = word(args.pos)
    print(" ".join("".join(map(str, c[10 * g:10 * g + 10])) for g in range(3)) + " " + "".join(map(str, c[30:])))


def cmd_stim(args) -> None:
    rng = np.random.default_rng(args.seed)
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    truth, rows, whole = [], [], 0
    for n in range(args.lines):
        if rng.random() < args.garbage:
            y = np.clip(rng.normal(100, 80, 720), 1, 254).astype(int)
            if rng.random() < 0.5:          # or random cells at the right width
                cells = rng.integers(0, 2, 120)
                y = np.where(np.repeat(cells, 7)[:720] > 0, 190, 5) + rng.integers(-10, 10, 720)
            truth.append([-1, 0])
        else:
            pos = int(rng.integers(0, 65536))
            gain = 1 + rng.uniform(-args.gain, args.gain)
            y = vitc.channel(dac_line(word(pos)), rng, args.bw_khz * 1e3, args.noise, gain,
                             rng.uniform(-args.shift, args.shift))
            keep = 720
            if rng.random() < args.frag:        # a piece of the line survives, the rest reads ~1
                keep = int(rng.integers(args.piece_min, 721))
                a = int(rng.integers(0, 721 - keep))
                y = np.where((np.arange(720) >= a) & (np.arange(720) < a + keep), y, 1)
            # copies wholly inside the kept piece, in DE1 samples (first cell ~ 29 samples after SAV)
            t0 = (X0 / vitc.FS_DAC - vitc.Y_SAV / vitc.FS_Y) * vitc.FS_Y
            starts = [t0 + k * CELLS * Y_PER_CELL for k in range(COPIES)]
            if keep == 720:
                a = 0
            n_whole = sum(1 for s in starts if s >= a + 2 and s + 30 * Y_PER_CELL <= a + keep - 2)
            whole += n_whole > 0
            truth.append([pos, n_whole])
        rows.append(" ".join(f"{int(v):02x}" for v in np.clip(y, 1, 254)))
    (out / "ptag_lines.txt").write_text("\n".join(rows) + "\n")
    (out / "ptag_params.vh").write_text(f"localparam integer NL = {args.lines};\n")
    (out / "ptag_truth.json").write_text(json.dumps({"truth": truth}))
    print(f"{args.lines} lines ({args.garbage:.0%} garbage, {args.frag:.0%} cut to a piece of >= {args.piece_min}); "
          f"{whole} hold a whole copy")


def cmd_score(args) -> None:
    truth = json.loads((args.out_dir / "ptag_truth.json").read_text())["truth"]
    rows = [l.split() for l in (args.out_dir / "ptag_out.txt").read_text().splitlines() if l.strip()]
    read = wrong = missed = false = could = 0
    for (pos, n_whole), r in zip(truth, rows):
        got = [int(v, 16) for v in r[1:1 + min(int(r[0]), 3)]]
        if pos < 0:
            false += len(got)
            continue
        could += n_whole > 0
        wrong += sum(g != pos for g in got)
        if any(g == pos for g in got):
            read += 1
        elif n_whole:
            missed += 1
    tags = sum(r[4] == "1" for r in rows)
    print(f"{len(rows)} lines: {read} read of {could} with a whole copy, {missed} missed, {wrong} wrong reads, "
          f"{false} reads off garbage lines; tag_slicer took {tags} as tags")
    if wrong or false or tags:
        raise SystemExit(1)


PROBE = [("reads", 16), ("fields_read", 16), ("fields_agreed", 16), ("fields_conflict", 16), ("seq", 8),
         ("line", 5), ("at", 7), ("age", 6), ("pos", 16), ("field_reads", 5), ("agreed", 16)]


def parse(bits: str) -> dict:
    out, i = {}, 0
    for name, w in PROBE:
        out[name] = int(bits[i:i + w], 2)
        i += w
    return out


def stp(*cmds: str) -> list[str]:
    r = subprocess.run([str(QUARTUS_BIN / "quartus_stp"), "-t", str(ROOT / "chipset/probe.tcl"), "DE-SoC",
                        "PTAG", *cmds], capture_output=True, text=True)
    if r.returncode:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit("quartus_stp failed")
    return [l.split()[1] for l in r.stdout.splitlines() if l.startswith("BITS ")]


def cmd_check(args) -> None:
    if args.reset:
        stp("src 1", "src 0")
    t0, prev = time.monotonic(), None
    for k in range(args.count):
        if k:
            time.sleep(args.interval)
        s = parse(stp("read")[-1])
        t = time.monotonic()
        rate = ""
        if prev:
            dt, p = t - prev[0], prev[1]
            rate = (f"  {((s['reads'] - p['reads']) & 0xFFFF) / dt:6.1f} reads/s, fields read "
                    f"{((s['fields_read'] - p['fields_read']) & 0xFFFF) / dt:5.1f}/s")
        print(f"{t - t0:6.1f} s  pos {s['pos']} (agreed {s['agreed']})  line {s['line']} at {s['at']}  age {s['age']}  "
              f"per field {s['field_reads']}  agreed {s['fields_agreed']} conflict {s['fields_conflict']}{rate}",
              flush=True)
        prev = (t, s)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("word")
    p.add_argument("pos", type=int)
    p = sub.add_parser("stim")
    p.add_argument("--out-dir", type=Path, default=ROOT / "chipset/sim/out")
    p.add_argument("--lines", type=int, default=600)
    p.add_argument("--bw-khz", type=float, default=3000)
    p.add_argument("--noise", type=float, default=0, help="IRE rms")
    p.add_argument("--gain", type=float, default=0, help="+- fraction")
    p.add_argument("--shift", type=float, default=0, help="+- us of line position")
    p.add_argument("--frag", type=float, default=0, help="fraction of lines cut to one piece")
    p.add_argument("--piece-min", type=int, default=100, help="shortest piece, DE1 samples")
    p.add_argument("--garbage", type=float, default=0, help="fraction of lines that are noise or random cells")
    p.add_argument("--seed", type=int, default=1)
    p = sub.add_parser("score")
    p.add_argument("--out-dir", type=Path, default=ROOT / "chipset/sim/out")
    p = sub.add_parser("check")
    p.add_argument("--count", type=int, default=5)
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--reset", action="store_true", help="clear the counters first")
    args = ap.parse_args()
    {"word": cmd_word, "stim": cmd_stim, "score": cmd_score, "check": cmd_check}[args.cmd](args)


if __name__ == "__main__":
    main()
