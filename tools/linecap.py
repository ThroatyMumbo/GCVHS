#!/usr/bin/env python3
"""Raw luma of chosen lines, and why VITC lines fail, off the DE1 (JTAG instances LCAP, VDIA).

Commands:
  cap LINE... [--save DIR]   capture each active line of the next field; print a VITC-eye summary
  vdia [--clear]             vitc_slicer's verdicts on VITC-like lines since the last clear
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
QUARTUS_BIN = Path(os.environ.get("QUARTUS_BIN", ""))   # unset: quartus_stp on PATH
THR, CELL = 100, 858 / 115              # vitc_slicer's threshold; Y samples per VITC bit
WHY = ["ok", "no edge", "cells missing", "sync", "crc", "digits"]


def stp(*cmds: str) -> list[str]:
    r = subprocess.run([str(QUARTUS_BIN / "quartus_stp"), "-t", str(ROOT / "chipset/lcap.tcl"), "DE-SoC", *cmds],
                       capture_output=True, text=True)
    if r.returncode:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit("quartus_stp failed")
    return [l for l in r.stdout.splitlines() if l.startswith(("CAP", "PAGE", "VDIA"))]


def lines_of(out: list[str]) -> dict[int, np.ndarray]:
    caps, cur, pages = {}, None, {}
    for l in out + ["CAP -1 end"]:
        w = l.split()
        if w[0] == "CAP":
            if cur is not None and len(pages) == 23:
                y = []
                for p in range(23):
                    bits = pages[p]
                    y += [int(bits[len(bits) - 8 * (i + 1):len(bits) - 8 * i], 2) for i in range(32)]
                caps[cur] = np.array(y[:720])
            cur, pages = (int(w[1]) if w[2] != "TIMEOUT" else None), {}
        elif w[0] == "PAGE":
            pages[int(w[1])] = w[2]
    return caps


def summary(line: int, y: np.ndarray) -> str:
    hi = y > THR
    if not hi.any():
        return f"line {line:3d}: nothing over {THR} (max {y.max()})"
    e0 = int(np.argmax(hi))
    run = int(np.argmax(~hi[e0:])) if (~hi[e0:]).any() else 720 - e0
    rises = np.flatnonzero(~hi[:-1] & hi[1:]) + 1
    falls = np.flatnonzero(hi[:-1] & ~hi[1:]) + 1
    edges = np.sort(np.concatenate([rises, falls]))
    cells = (edges - e0) / CELL
    err = np.abs(cells - np.round(cells))
    ones = y[hi]
    return (f"line {line:3d}: first high at {e0}, first run {run} samples; {len(edges)} edges, off-grid "
            f"median {np.median(err):.2f} max {err.max():.2f} cells; span {(edges[-1] - e0) / CELL:.1f} cells; "
            f"low {np.median(y[~hi]):.0f} high {np.median(ones):.0f} max {y.max()}")


def cmd_cap(args) -> None:
    caps = lines_of(stp(*[f"cap {n}" for n in args.lines]))
    for n in args.lines:
        if n not in caps:
            print(f"line {n:3d}: no capture")
            continue
        print(summary(n, caps[n]))
        if args.save:
            args.save.mkdir(parents=True, exist_ok=True)
            np.save(args.save / f"line{n}.npy", caps[n])


def cmd_vdia(args) -> None:
    b = stp("vdia", *(["vclr"] if args.clear else []))[0].split()[1]
    why = [int(b[16 * i:16 * i + 16], 2) for i in range(6)][::-1]
    grp = [int(b[96 + 12 * i:108 + 12 * i], 2) for i in range(10)][::-1]
    tot = sum(why) or 1
    print("VITC-like lines: " + ", ".join(f"{WHY[i]} {why[i]} ({100 * why[i] / tot:.0f}%)" for i in range(6)))
    print("first bad sync group: " + " ".join(f"{g}:{grp[g]}" for g in range(9)) + f" none:{grp[9]}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("cap")
    p.add_argument("lines", type=int, nargs="+")
    p.add_argument("--save", type=Path)
    p = sub.add_parser("vdia")
    p.add_argument("--clear", action="store_true", help="clear after reading")
    args = ap.parse_args()
    {"cap": cmd_cap, "vdia": cmd_vdia}[args.cmd](args)


if __name__ == "__main__":
    main()
