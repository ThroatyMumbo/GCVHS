#!/usr/bin/env python3
"""Tape position as SMPTE LTC on the deck audio's right channel.

Commands:
  word   print the LTC word for a frame number
  stim   write the GCV2's LTC through a modelled audio path, plus DE1 field starts, for tb_ltc.v
  score  compare tb_ltc.v's per-field output with the truth stim wrote
  check  read the DE1's LTCP probe over JTAG (and the GCV2's ctl over SWD) and report the lock
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

ROOT = Path(__file__).resolve().parent.parent
QUARTUS_BIN = Path(os.environ.get("QUARTUS_BIN", ""))   # unset: quartus_stp on PATH

A_T, LINE_T = 13125, 40040              # streams.c: 48 kHz frame and line, in half sys clocks
SYS_HZ = 315e6
LINE_S = LINE_T / 2 / SYS_HZ
CODEC_HZ = 50e6 / 1024                  # chipset audio_out
TB_DIV = 64                             # tb_ltc.v ticks are 64 CLOCK_50 cycles; a codec frame is 16
FIELD_CLKS = 834168                     # CLOCK_50 cycles per field


def ltc_word(frame: int, user: int = 0) -> list[int]:
    """streams.c ltc_make: 80 bits, bit 0 first."""
    f, s, m, h = frame % 30, frame // 30 % 60, frame // 1800 % 60, frame // 108000 % 24
    d = [f % 10, f // 10, s % 10, s // 10, m % 10, m // 10, h % 10, h // 10]
    w = [d[k] | (user >> 4 * k & 15) << 4 for k in range(8)] + [0xFC, 0xBF]
    if sum(bin(b).count("1") for b in w) & 1:
        w[3] |= 0x08
    return [w[i >> 3] >> (i & 7) & 1 for i in range(80)]


def gcv2_audio(lines: int, labels, amp: int = 4096, il: bool = True) -> np.ndarray:
    """streams.c audio_line over `lines` lines from frame line 0: the right channel at 48 kHz.
    labels(k) gives the LTC frame number sent in video frame k."""
    flen = 525 if il else 524
    out, a_rel, lvl, word, h_last, k = [], 0, 1, None, -1, -1
    for gl in range(lines):
        line = gl % flen
        if line == 0:
            k += 1
            word, h_last = ltc_word(labels(k)), -1
        length = flen * 1001
        while a_rel < LINE_T:
            h = min((line * LINE_T + a_rel) * 4 // length, 159)
            edge = False
            while h_last < h:
                h_last += 1
                j = h_last
                if not (j & 1) or word[j >> 1]:
                    lvl, edge = -lvl, True
            out.append(0 if edge else lvl * amp)
            a_rel += A_T
        a_rel -= LINE_T
    return np.array(out, np.float64)


def cmd_word(args) -> None:
    bits = ltc_word(args.frame, args.user)
    print("".join(map(str, bits)))


def cmd_stim(args) -> None:
    rng = np.random.default_rng(args.seed)
    splices = [tuple(map(int, s.split(":"))) for s in args.splice]
    def labels(k: int) -> int:
        return args.start + k + sum(d for at, d in splices if k >= at)

    lines = int(args.seconds / LINE_S) + 600
    x48 = gcv2_audio(lines, labels, args.amp)
    t48 = np.arange(len(x48)) / 48000.0

    # PCM5102A -> deck -> WM8731: band-limited, gain 2.1 (2.1 Vrms DAC into a 1 Vrms line in), delayed.
    fine = 16
    tf = np.arange(len(x48) * fine) / (48000.0 * fine)
    xf = np.interp(tf, t48, x48)
    a = 1 - np.exp(-2 * np.pi * args.bw / (48000.0 * fine))
    y, acc = np.empty_like(xf), 0.0
    for i, v in enumerate(xf):
        acc += a * (v - acc)
        y[i] = acc
    for s in args.dropout:
        t0, dt = (float(v) / 1000 for v in s.split(":"))
        y[(tf >= t0) & (tf < t0 + dt)] = 0
    n = int(args.seconds * CODEC_HZ)
    tc = np.arange(n) / CODEC_HZ - args.audio_delay / 1000
    xs = np.interp(tc, tf, y, left=0) * 2.1 * args.gain + rng.normal(0, args.noise, n)
    xs = np.clip(np.round(xs), -32768, 32767).astype(np.int64)

    # DE1 field starts: GCV2 field i begins at 262.5 i lines; the DE1 marks it video_delay later.
    fields, truth, i = [], [], 0
    while True:
        t = i * 262.5 * LINE_S + args.video_delay / 1000
        if t >= n / CODEC_HZ:
            break
        k, fif = divmod(i, 2)
        fields.append(int(round(t * 50e6 / TB_DIV)))
        truth.append(2 * labels(k) + fif)
        i += 1

    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    np.savetxt(out / "ltc_x.hex", xs & 0xFFFF, fmt="%04x")
    np.savetxt(out / "ltc_fs.hex", np.array(fields), fmt="%08x")
    (out / "ltc_params.vh").write_text(
        f"localparam integer NS = {n};\nlocalparam integer NF = {len(fields)};\n"
        f"localparam [19:0] FIELD_T = {round(FIELD_CLKS / TB_DIV)};\n")
    (out / "ltc_truth.json").write_text(json.dumps({"truth": truth, "args": vars(args) | {"out_dir": str(out)}}))
    print(f"{out}: {args.seconds} s, {n} codec frames, {len(fields)} fields, LTC from frame {args.start}, "
          f"peak {np.abs(xs).max()}")


def phase_ms(clks: int) -> float:
    """ltc_rx phase: where word ends land against the field starts, audio late positive."""
    return (clks - FIELD_CLKS if clks >= FIELD_CLKS // 2 else clks) / 50e3


def cmd_score(args) -> None:
    truth = json.loads((args.out_dir / "ltc_truth.json").read_text())["truth"]
    rows = [l.split() for l in (args.out_dir / "ltc_fields.txt").read_text().splitlines() if l.strip()]
    first_lock = wrong = unlocked_after = 0
    for i, r in enumerate(rows):
        field, locked = int(r[1]), int(r[2])
        if locked and not first_lock:
            first_lock = i + 1
        if first_lock and locked and field != truth[i]:
            wrong += 1
            if wrong <= 5:
                print(f"  field {i}: DE1 {field} truth {truth[i]}")
        if first_lock and not locked:
            unlocked_after += 1
    last = rows[-1]
    lock = f"locked from field {first_lock} ({first_lock / 59.94:.2f} s)" if first_lock else "NEVER locked"
    print(f"{len(rows)} fields; {lock}; {wrong} locked fields wrong; {unlocked_after} unlocked after first lock; "
          f"words {last[4]} bad {last[5]} jumps {last[6]} phase {phase_ms(int(last[7]) * TB_DIV):+.2f} ms")
    tc = int(last[3], 16)
    fr = int(last[1]) // 2
    want = f"{fr // 108000 % 24:02d}{fr // 1800 % 60:02d}{fr // 30 % 60:02d}{fr % 30:02d}"
    print(f"last tc {tc:08x} vs field/2 {want}: {'ok' if f'{tc:08x}' == want else 'MISMATCH'}")
    if wrong or not first_lock:
        raise SystemExit(1)


# JTAG: probe LTCP (chipset video_top), fields MSB first.
PROBE = [("tag_ok", 16), ("tag_bad", 16), ("peak", 16), ("phase", 20), ("jumps", 16), ("bad", 16),
         ("words", 16), ("user", 32), ("tc", 32), ("locked", 1), ("field", 24)]


def stp(*cmds: str) -> list[str]:
    r = subprocess.run([str(QUARTUS_BIN / "quartus_stp"), "-t", str(ROOT / "chipset/probe.tcl"), "DE-SoC",
                        "LTCP", *cmds], capture_output=True, text=True)
    if r.returncode:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit("quartus_stp failed")
    return [l.split()[1] for l in r.stdout.splitlines() if l.startswith("BITS ")]


def parse(bits: str) -> dict:
    out, i = {}, 0
    for name, w in PROBE:
        out[name] = int(bits[i:i + w], 2)
        i += w
    assert i == len(bits), f"probe is {len(bits)} bits, expected {i}"
    return out


def gcv2_frame() -> int | None:
    r = subprocess.run([str(ROOT / "scripts/gcv2_ctl.sh")], capture_output=True, text=True)
    for l in r.stdout.splitlines():
        if l.startswith("ltc_frame "):
            return int(l.split()[1])
    return None


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
        rate = ""
        if prev:
            dt = t - prev[0]
            rate = (f"  {(s['field'] - prev[1]['field']) / dt:6.2f} fields/s"
                    f"  {(s['words'] - prev[1]['words']) / dt:5.2f} words/s")
        print(f"{t - t0:6.1f} s  {'LOCK' if s['locked'] else '----'} field {s['field']} tc {tc_str(s['tc'])} "
              f"user {s['user']:08x}  words {s['words']} bad {s['bad']} jumps {s['jumps']} "
              f"phase {phase_ms(s['phase']):+.2f} ms "
              f"peak {s['peak']}  tag ok {s['tag_ok']} bad {s['tag_bad']}{rate}", flush=True)
        prev = (t, s)
    if args.gcv2:
        g = gcv2_frame()
        s = parse(stp("read")[-1])
        if g is not None:
            print(f"GCV2 sending frame {g}, DE1 at frame {s['field'] // 2} (read ~1 s apart: expect ~30 behind)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("word")
    p.add_argument("frame", type=int)
    p.add_argument("--user", type=lambda v: int(v, 0), default=0)
    p = sub.add_parser("stim")
    p.add_argument("--out-dir", type=Path, default=ROOT / "chipset/sim/out")
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--start", type=int, default=1234, help="LTC frame number of the first video frame")
    p.add_argument("--amp", type=int, default=4096, help="streams.c ltc_amp")
    p.add_argument("--gain", type=float, default=1.0, help="extra gain in the audio path")
    p.add_argument("--bw", type=float, default=15000, help="audio path -3 dB, Hz")
    p.add_argument("--noise", type=float, default=0, help="rms, codec LSBs")
    p.add_argument("--audio-delay", type=float, default=1.2, help="ms, GCV2 to DE1 codec")
    p.add_argument("--video-delay", type=float, default=1.1, help="ms, GCV2 field start to DE1 field_start")
    p.add_argument("--dropout", action="append", default=[], metavar="START_MS:LEN_MS")
    p.add_argument("--splice", action="append", default=[], metavar="FRAME:DELTA",
                   help="from video frame FRAME on, labels jump by DELTA (a tape edit)")
    p.add_argument("--seed", type=int, default=1)
    p = sub.add_parser("score")
    p.add_argument("--out-dir", type=Path, default=ROOT / "chipset/sim/out")
    p = sub.add_parser("check")
    p.add_argument("--count", type=int, default=1)
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--reset", action="store_true", help="clear the DE1's LTC counters first")
    p.add_argument("--gcv2", action="store_true", help="also read the GCV2's frame over SWD")
    args = ap.parse_args()
    {"word": cmd_word, "stim": cmd_stim, "score": cmd_score, "check": cmd_check}[args.cmd](args)


if __name__ == "__main__":
    main()
