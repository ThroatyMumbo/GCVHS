#!/usr/bin/env python3
"""IR remote codes off the DE1's receiver (chipset ir_rx, JTAG instance IRRX).

Commands:
  watch            print every frame received: decoded protocol and raw mark/space timings (us)
  learn NAME...    prompt for each button in turn and save its first frame to a JSON map
  decode US...     decode a raw timing list given on the command line (mark first)
  send BUTTON...   send mapped buttons from the DE1's emitter (ir_tx), and report what its receiver saw
  shell            prompt for buttons to send over one open JTAG session (no start-up per send)
  time BUTTON[:S]  send each button and log what the DE1 sees for S seconds (default 3): lock, field lengths,
                   tag line, LTC, VITC and position tags; real deck or GCV2 alike
  time --check     the standard sequence from a deck that is off, with the expected numbers alongside
"""
from __future__ import annotations

import argparse
import json
import os
import pty
import re
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
QUARTUS_BIN = Path(os.environ.get("QUARTUS_BIN", ""))   # unset: quartus_stp on PATH
MAP = ROOT / "tools/vcr_ir.json"

CARRIER_HZ = {"NEC": 38000, "NECext": 38000, "Samsung32": 38000, "JVC": 38000, "Kaseikyo": 36700,
              "Sony": 40000, "RC5": 36000}


def near(x: int, t: int, tol: float = 0.3) -> bool:
    return abs(x - t) <= tol * t


def pd_bits(d: list[int], i: int, n: int, mark: int, zero: int, one: int) -> int | None:
    """n pulse-distance bits from d[i], LSB first, then a stop mark."""
    if len(d) < i + 2 * n + 1:
        return None
    v = 0
    for k in range(n):
        m, s = d[i + 2 * k], d[i + 2 * k + 1]
        if not near(m, mark):
            return None
        if near(s, one):
            v |= 1 << k
        elif not near(s, zero):
            return None
    return v if near(d[i + 2 * n], mark) else None


def dec_nec(d: list[int]) -> dict | None:
    if len(d) == 3 and near(d[0], 9000) and near(d[1], 2250) and near(d[2], 560):
        return {"protocol": "NEC", "repeat": True}
    if len(d) != 67:
        return None
    for proto, hm, hs in (("NEC", 9000, 4500), ("Samsung32", 4500, 4500)):
        if near(d[0], hm, 0.2) and near(d[1], hs, 0.2):
            v = pd_bits(d, 2, 32, 560, 560, 1690)
            if v is None:
                return None
            a, na, c, nc = v & 255, v >> 8 & 255, v >> 16 & 255, v >> 24
            if proto == "Samsung32":
                return {"protocol": proto, "address": a, "command": c, "check": c ^ nc == 255}
            if a ^ na == 255:
                return {"protocol": "NEC", "address": a, "command": c, "check": c ^ nc == 255}
            return {"protocol": "NECext", "address": v & 0xFFFF, "command": c, "check": c ^ nc == 255}
    return None


def dec_jvc(d: list[int]) -> dict | None:
    if len(d) == 35 and near(d[0], 8400, 0.2) and near(d[1], 4200, 0.2):
        v, rep = pd_bits(d, 2, 16, 526, 526, 1578), False
    elif len(d) == 33:
        v, rep = pd_bits(d, 0, 16, 526, 526, 1578), True
    else:
        return None
    if v is None:
        return None
    return {"protocol": "JVC", "address": v & 255, "command": v >> 8, "repeat": rep}


def dec_kaseikyo(d: list[int]) -> dict | None:
    if len(d) != 99 or not (near(d[0], 3456, 0.2) and near(d[1], 1728, 0.2)):
        return None
    v = pd_bits(d, 2, 48, 432, 432, 1296)
    if v is None:
        return None
    return {"protocol": "Kaseikyo", "vendor": v & 0xFFFF, "bytes": [v >> 8 * k & 255 for k in range(2, 6)]}


def dec_sony(d: list[int]) -> dict | None:
    if not (near(d[0], 2400, 0.2) and near(d[1], 600)) or len(d) not in (25, 31, 41):
        return None
    n = (len(d) - 1) // 2
    v = 0
    for k in range(n):
        m = d[2 + 2 * k]
        if k < n - 1 and not near(d[3 + 2 * k], 600):
            return None
        if near(m, 1200):
            v |= 1 << k
        elif not near(m, 600):
            return None
    out = {"protocol": "Sony", "bits": n, "command": v & 127}
    if n == 20:
        out.update(address=v >> 7 & 31, extended=v >> 12)
    else:
        out["address"] = v >> 7
    return out


def dec_rc5(d: list[int]) -> dict | None:
    halves = [0]
    for k, t in enumerate(d):
        h = round(t / 889)
        if h not in (1, 2) or not near(t, 889 * h, 0.35):
            return None
        halves += [1 - k % 2] * h
    if len(halves) % 2:
        halves.append(0)
    if len(halves) != 28:
        return None
    bits = []
    for a, b in zip(halves[::2], halves[1::2]):
        if a == b:
            return None
        bits.append(b)
    v = int("".join(map(str, bits)), 2)
    return {"protocol": "RC5", "toggle": v >> 11 & 1, "address": v >> 6 & 31,
            "command": (v & 63) | (0 if v >> 12 & 1 else 64)}


def dec_generic(d: list[int]) -> dict | None:
    if len(d) < 9:
        return None
    body = d[2:] if d[0] > 2 * statistics.median(d[::2]) else d
    marks, spaces = body[::2], body[1::2]
    if max(marks) < 1.6 * min(marks) and max(spaces) > 1.6 * min(spaces):
        kind, cells = "pulse-distance", spaces
    elif max(spaces) < 1.6 * min(spaces) and max(marks) > 1.6 * min(marks):
        kind, cells = "pulse-width", marks
    else:
        return None
    thr = (min(cells) + max(cells)) / 2
    bits = [int(c > thr) for c in cells]
    v = sum(b << k for k, b in enumerate(bits))
    out = {"protocol": kind, "bits": len(bits), "value": f"{v:#x}"}
    if body is not d:
        out["header"] = d[:2]
    return out


def decode(d: list[int]) -> dict:
    for f in (dec_nec, dec_jvc, dec_kaseikyo, dec_sony, dec_rc5, dec_generic):
        r = f(d)
        if r:
            if r["protocol"] in CARRIER_HZ:
                r["carrier_hz"] = CARRIER_HZ[r["protocol"]]
            return r
    return {"protocol": "unknown"}


def describe(r: dict) -> str:
    def v(k, x):
        if isinstance(x, bool):
            return f"{k}={'ok' if x else 'BAD'}" if k == "check" else (k if x else "")
        if isinstance(x, int) and k not in ("bits", "toggle", "carrier_hz"):
            return f"{k}=0x{x:02x}"
        if isinstance(x, list) and k == "bytes":
            return f"{k}=" + " ".join(f"{b:02x}" for b in x)
        return f"{k}={x}"
    return " ".join([r["protocol"]] + [s for k, x in r.items() if k != "protocol" and (s := v(k, x))])


# Probe IRRX, MSB first: idle, over, len[8:0], seq[15:0], t_start[31:0] (us), page[3:0], 16 entries
# (entry 0 lowest).
def parse_frame(pages: list[str]) -> tuple[dict, list[int]]:
    b = pages[0]
    hdr = {"idle": int(b[0]), "over": int(b[1]), "len": int(b[2:11], 2), "seq": int(b[11:27], 2),
           "t": int(b[27:59], 2)}
    d = []
    for p, bits in enumerate(pages):
        assert len(bits) == 319, f"probe is {len(bits)} bits, expected 319"
        if int(bits[59:63], 2) != p:
            raise ValueError(f"page {p} read back as {int(bits[59:63], 2)}")
        data = int(bits[63:], 2)
        d += [data >> 16 * k & 0xFFFF for k in range(16)]
    return hdr, d[:hdr["len"]]


ANSI = re.compile(r"\x1b\[[0-9;]*m")


def quartus_tcl(script: str, *args: str):
    """quartus_stp on a pty: on a pipe it holds back all its output until it exits."""
    m, s = pty.openpty()
    p = subprocess.Popen([str(QUARTUS_BIN / "quartus_stp"), "-t", str(ROOT / "chipset" / script), "DE-SoC",
                          *args], stdin=subprocess.DEVNULL, stdout=s, stderr=s)
    os.close(s)

    def lines():
        with os.fdopen(m, "r", errors="replace") as f:
            try:
                for line in f:
                    line = ANSI.sub("", line).rstrip()
                    if "rror" in line or line.startswith(("no ", "probe ", "unknown")):
                        sys.stderr.write(line + "\n")
                    yield line
            except OSError:             # EIO once the child has gone
                return
    return p, lines()


def frames():
    p, lines = quartus_tcl("irctl.tcl", "watch")
    last = None
    try:
        for line in lines:
            w = line.split()
            if not w:
                continue
            if w[0] == "READY":
                print(f"listening (receiver idles {'high' if w[2] == '1' else 'low'})", flush=True)
                last = int(w[4])
            elif w[0] == "TORN":
                print(f"frame {w[1]} overwritten while reading", flush=True)
            elif w[0] == "FRAME":
                hdr, d = parse_frame(w[1:])
                if len(d) < 3:          # lone 25-180 us pulses from the receiver between presses
                    continue
                missed = (hdr["seq"] - last - 1) & 0xFFFF if last is not None else 0
                last = hdr["seq"]
                yield hdr, d, missed
        raise SystemExit(f"quartus_stp exited ({p.wait()})")
    finally:
        p.kill()


def cmd_watch(args) -> None:
    t_prev = None
    for hdr, d, missed in frames():
        r = decode(d)
        extra = f"  +{(hdr['t'] - t_prev) % 2**32 / 1000:.1f} ms" if t_prev is not None else ""
        t_prev = hdr["t"]
        extra += f"  ({missed} missed)" if missed else ""
        extra += "  (over 256 entries, truncated)" if hdr["over"] else ""
        print(f"#{hdr['seq']:<5} {describe(r)}{extra}")
        if args.raw or r["protocol"] in ("unknown", "pulse-distance", "pulse-width"):
            print(f"       raw {len(d)}: {' '.join(map(str, d))}")
        sys.stdout.flush()


def cmd_learn(args) -> None:
    codes = json.loads(args.out.read_text()) if args.out.exists() else {}
    it = frames()
    for name in args.names:
        print(f"press {name}", flush=True)
        for hdr, d, _ in it:
            r = decode(d)
            if r.get("repeat") or hdr["over"]:
                continue
            codes[name] = r if "carrier_hz" in r else {**r, "raw": d}
            args.out.write_text(json.dumps(codes, indent=1) + "\n")
            print(f"  {name}: {describe(r)}", flush=True)
            break
    it.close()
    print(f"saved {len(args.names)} to {args.out}")


def cmd_decode(args) -> None:
    print(describe(decode(args.us)))


def payload(code: dict) -> int:
    if code.get("protocol") != "Kaseikyo":
        raise SystemExit(f"chipset ir_tx only sends Kaseikyo, not {code.get('protocol')}")
    return code["vendor"] | sum(b << 8 * (k + 2) for k, b in enumerate(code["bytes"]))


class Link:
    """One quartus_stp session running chipset/irctl.tcl: the first send pays its ~0.6 s start-up."""

    def __init__(self):
        self.dir = tempfile.TemporaryDirectory()
        fifo = Path(self.dir.name) / "cmds"
        os.mkfifo(fifo)
        self.p, self.lines = quartus_tcl("irctl.tcl", str(fifo))
        self.until("READY")
        self.cmds = open(fifo, "w", buffering=1)

    def until(self, word: str) -> list[str]:
        for line in self.lines:
            if line.startswith(word):
                return line.split()
        raise SystemExit(f"quartus_stp exited ({self.p.wait()})")

    def send(self, name: str, v: int) -> None:
        self.cmds.write(f"send {v:048b}\n")
        self.until("SENT")
        print(f"sent {name} ({v:012x})", flush=True)
        w = self.until("RX")
        if w[1] == "0":
            print("  loopback: the DE1's receiver saw nothing", flush=True)
            return
        hdr, d = parse_frame(w[2:])
        print(f"  loopback: {w[1]} frame(s), last {describe(decode(d))}", flush=True)

    def timed(self, v: int, secs: float) -> list:
        """Send without waiting for the loopback, then (t since the send, VSTA, TAGW, LTCP, PTAG, VITC) samples."""
        self.cmds.write(f"tx {v:048b}\nlog {int(secs * 1000)}\n")
        t0 = int(self.until("SENT")[1])
        samples = []
        for line in self.lines:
            w = line.split()
            if w and w[0] == "DONE":
                return samples
            if w and w[0] == "S" and len(w) == 7:
                samples.append(((int(w[1]) - t0) / 1000, *(parse_probe(b, lay) for b, lay in zip(w[2:], PROBES))))
        raise SystemExit(f"quartus_stp exited ({self.p.wait()})")

    def close(self) -> None:
        self.cmds.write("quit\n")
        self.cmds.close()
        self.p.wait()
        self.dir.cleanup()


def lookup(codes: dict, name: str) -> int | None:
    key = next((k for k in codes if k.lower() == name.lower()), None)
    if key is None:
        print(f"{name} not mapped: {', '.join(codes)}")
        return None
    return payload(codes[key])


def cmd_send(args) -> None:
    codes = json.loads(args.map.read_text())
    vs = [lookup(codes, b) for b in args.buttons]
    if None in vs:
        raise SystemExit(1)
    link = Link()
    for b, v in zip(args.buttons, vs):
        link.send(b, v)
    link.close()


def cmd_shell(args) -> None:
    codes = json.loads(args.map.read_text())
    link = Link()
    print(f"buttons: {', '.join(codes)}; empty line or ^D quits")
    while True:
        try:
            name = input("ir> ").strip()
        except EOFError:
            break
        if not name:
            break
        v = lookup(codes, name)
        if v is not None:
            link.send(name, v)
    link.close()


# chipset probes logged by `time`, fields MSB first (scripts/de1_ctl.sh, tools/ltc.py, ptag.py, vitc.py).
VSTA = [("codec_err", 1), ("aud_fill", 9), ("aud_underruns", 16), ("aud_overruns", 16), ("aud_peak", 16),
        ("bist_ok", 1), ("sd_lat", 3), ("bist_counts", 50), ("status1", 8), ("cfg_done", 1), ("i2c_err", 1),
        ("aud_stream", 3), ("aud_rate", 16), ("aud_no_pilot", 16), ("aud_cell0", 11), ("aud_pilot", 12),
        ("raw", 1), ("comb_off", 1), ("slot_shown", 2), ("stream", 4), ("tag_bad", 16), ("tag_seen", 1),
        ("tag_scheme", 4), ("tag_count", 8), ("fields_per_sec", 8), ("lines_f0", 10), ("lines_f1", 10)]
TAGW = [("seq", 8), ("line", 5), ("count", 8)]
LTCP = [("tag_ok", 16), ("tag_bad", 16), ("peak", 16), ("phase", 20), ("jumps", 16), ("bad", 16),
        ("words", 16), ("user", 32), ("tc", 32), ("locked", 1), ("field", 24)]
PTAG = [("reads", 16), ("fields_read", 16), ("fields_agreed", 16), ("fields_conflict", 16), ("seq", 8),
        ("line", 5), ("at", 7), ("age", 6), ("pos", 16), ("field_reads", 5), ("agreed", 16)]
VITC = [("reads", 16), ("seq", 8), ("line", 5), ("age", 6), ("mark", 1), ("tc", 32), ("pos", 24)]
PROBES = [VSTA, TAGW, LTCP, PTAG, VITC]

# Measured on the real deck: s after the send, per button and the state it was pressed in.
EXPECT = {
    ("POWER", "off"): "lock 0.3-0.4",
    ("PLAY", "stop"): "first tag on line 10 1.7-1.8 (1.78 SP)",
    ("PAUSE", "play"): "VITC stops ~0.3, LTC unlock 0.56, lines 264/262, tag on 11/12 ~11/s, ptag 41-43/s",
    ("PLAY", "pause"): "LTC relock 0.67, tag settles on line 10 1.37-1.39",
    ("FF", "play"): "LTC unlock 0.56, lines 252/253, tag on 6/7 ~10/s, ptag 45/s stepping 7",
    ("PLAY", "cue"): "LTC relock 0.99",
    ("REW", "play"): "estimate: lines 272/273, tag on 13/14, stepping -7",
    ("PLAY", "review"): "estimate: as PLAY from cue",
    ("STOP", "play"): "tags gone 0.09-0.16, ~100 ms lock blip",
    ("POWER", "stop"): "not measured",
}
NEXT = {("POWER", "off"): "stop", ("PLAY", "stop"): "play", ("PAUSE", "play"): "pause", ("PLAY", "pause"): "play",
        ("PAUSE", "pause"): "play", ("FF", "play"): "cue", ("REW", "play"): "review", ("PLAY", "cue"): "play",
        ("PLAY", "review"): "play", ("FF", "stop"): "wind", ("REW", "stop"): "wind", ("STOP", "wind"): "stop"}
SEQUENCE = [("POWER", 2), ("PLAY", 4), ("PAUSE", 3), ("PLAY", 3), ("FF", 3), ("PLAY", 3), ("REW", 3), ("PLAY", 3),
            ("STOP", 2), ("FF", 2), ("STOP", 1.5), ("PLAY", 4), ("STOP", 2), ("POWER", 2)]


def parse_probe(bits: str, layout) -> dict:
    out, i = {}, 0
    for name, w in layout:
        out[name] = int(bits[i:i + w], 2)
        i += w
    return out


def changes(samples, get) -> list:
    out, last = [], None
    for s in samples:
        v = get(s)
        if v != last:
            out.append((s[0], v))
            last = v
    return out


def timing_report(samples) -> list[str]:
    if not samples:
        return ["no samples"]
    rows = []
    rows.append("lock    " + ", ".join(f"{v} @{t:.2f}" for t, v in changes(samples, lambda s: s[1]["status1"] & 1)))
    lines = changes(samples, lambda s: (s[1]["lines_f0"], s[1]["lines_f1"]))
    rows.append("lines   " + ", ".join(f"{a}/{b} @{t:.2f}" for t, (a, b) in lines[:6]) + (" ..." if len(lines) > 6 else ""))

    hits = [(b[0], b[2]["line"]) for a, b in zip(samples, samples[1:]) if b[2]["seq"] != a[2]["seq"]]
    end = samples[-1][0]
    if hits:
        run = len(hits)
        while run > 0 and hits[run - 1][1] == 10:
            run -= 1
        settled = f", settled on 10 @{hits[run][0]:.2f}" if 0 < run < len(hits) else ""
        tail = [h for h in hits if h[0] > end / 2]
        hist = {}
        for _, ln in tail:
            hist[ln] = hist.get(ln, 0) + 1
        l10 = next((t for t, ln in hits if ln == 10), None)
        rows.append(f"tag     first @{hits[0][0]:.2f} line {hits[0][1]}, last @{hits[-1][0]:.2f}"
                    + (f", first on 10 @{l10:.2f}" if l10 is not None else "") + settled
                    + f"; 2nd half {len(tail) / (end / 2):.0f}/s on " + " ".join(f"L{k}:{v}" for k, v in sorted(hist.items())))
    else:
        rows.append("tag     none")

    rows.append("ltc     " + ", ".join(f"{'locked' if v else 'unlocked'} @{t:.2f}"
                                      for t, v in changes(samples, lambda s: s[3]["locked"])))
    vr = [b[0] for a, b in zip(samples, samples[1:]) if b[5]["reads"] != a[5]["reads"]]
    rows.append(f"vitc    {len(vr)} read changes" + (f", first @{vr[0]:.2f} last @{vr[-1]:.2f}" if vr else ""))

    half = [s for s in samples if s[0] > end / 2]
    if len(half) > 1:
        a, b = half[0], half[-1]
        agreed = (b[4]["fields_agreed"] - a[4]["fields_agreed"]) & 0xFFFF
        steps = []
        for x, y in zip(half, half[1:]):
            if (y[4]["fields_agreed"] - x[4]["fields_agreed"]) & 0xFFFF == 1:
                d = (y[4]["agreed"] - x[4]["agreed"]) & 0xFFFF
                steps.append(d - 0x10000 if d > 0x8000 else d)
        common = max(set(steps), key=steps.count) if steps else None
        rows.append(f"ptag    2nd half {agreed / (b[0] - a[0]):.0f} agreed/s, pos {b[4]['agreed']}"
                    + (f", step per field {common}" if common is not None else ""))
    return rows


def cmd_time(args) -> None:
    codes = json.loads(args.map.read_text())
    if args.check:
        keys = SEQUENCE
    else:
        keys = [(k.split(":")[0], float(k.split(":")[1]) if ":" in k else 3.0) for k in args.buttons]
    if not keys:
        raise SystemExit("time: buttons or --check")
    vs = [lookup(codes, k) for k, _ in keys]
    if None in vs:
        raise SystemExit(1)
    state = "off"
    link = Link()
    try:
        for (k, secs), v in zip(keys, vs):
            k = k.upper()
            samples = link.timed(v, secs)
            print(f"{k} (from {state}), {len(samples) / secs:.0f} samples/s" if args.check else k, flush=True)
            if args.check and (k, state) in EXPECT:
                print(f"  expect  {EXPECT[k, state]}")
            for r in timing_report(samples):
                print("  " + r, flush=True)
            state = NEXT.get((k, state), "stop" if k == "STOP" else "off" if k == "POWER" else state)
    finally:
        link.close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("watch")
    p.add_argument("--raw", action="store_true", help="print raw timings for decoded frames too")
    p = sub.add_parser("learn")
    p.add_argument("names", nargs="+")
    p.add_argument("--out", type=Path, default=MAP)
    p = sub.add_parser("decode")
    p.add_argument("us", type=int, nargs="+")
    p = sub.add_parser("send")
    p.add_argument("buttons", nargs="+")
    p.add_argument("--map", type=Path, default=MAP)
    p = sub.add_parser("shell")
    p.add_argument("--map", type=Path, default=MAP)
    p = sub.add_parser("time")
    p.add_argument("buttons", nargs="*")
    p.add_argument("--check", action="store_true")
    p.add_argument("--map", type=Path, default=MAP)
    args = ap.parse_args()
    try:
        {"watch": cmd_watch, "learn": cmd_learn, "decode": cmd_decode, "send": cmd_send,
         "shell": cmd_shell, "time": cmd_time}[args.cmd](args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
