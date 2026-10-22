#!/usr/bin/env python3
"""Data tape: a game image on video lines, GCV2 (streams.c, src 2) to DE1.

Commands:
  bake   build the flash blob vhs_streams sends (flash_rp2350.sh data), from a payload or the mock game
  mock   write the mock 256 KiB game image
  stim   write luma test lines and expected results for the Questa testbench (chipset/sim)
  check  read the DE1's loader over JTAG and compare it with the blob; --dump reads every packet back
"""
from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
import time
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
FLASH_OFFSET = 0xF00000                 # streams.c DATA_OFFSET; clips.py stops short of it
FLASH_END = 16 << 20
MAGIC = 0x44534856                      # "VHSD"
VERSION = 1
HDR_SIZE = 256
WIDTHS = (4, 5, 6, 7)                   # DAC samples per cell; the DE1's class is w - 4
DATA_LEAD = 16
DATA_SPAN = 720
RUNIN = 5
IDX_HEADER, IDX_PARITY = 0xFFF0, 0xFF00
MAX_PACKETS = 32768
ROWS = 240
ACTIVE = 752                            # DAC samples of active video (streams.c 2 * W)
DATA_ROWS = 230                         # streams.c ctl.data_rows default
GROUPS = 8                              # ... and ctl.data_parity
MOCK_SIZE = 256 << 10

DAC_HZ = 315e6 / 22
DE1_HZ = 13.5e6
DE1_PER_DAC = DE1_HZ / DAC_HZ
DE1_X0 = 6.7                            # DE1 luma sample of DAC sample 0 (from audio_slicer's FIRST)


def line_bytes(w: int) -> int:
    return (DATA_SPAN // w - RUNIN) // 8


def data_bytes(w: int) -> int:
    return line_bytes(w) - 4


def packets(length: int, w: int) -> int:
    return -(-length // data_bytes(w))


def crc16(b: bytes) -> int:
    c = 0xFFFF
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = (c << 1 ^ 0x1021 if c & 0x8000 else c << 1) & 0xFFFF
    return c


def keystream(row: int, nbits: int) -> np.ndarray:
    s = (row & 0xFF) << 8 | (~row & 0xFF)
    out = np.empty(nbits, np.uint8)
    for i in range(nbits):
        k = s & 1
        s = s >> 1 ^ (0xB400 if k else 0)
        out[i] = k
    return out


def padded(payload: bytes, w: int) -> bytes:
    return payload.ljust(packets(len(payload), w) * data_bytes(w), b"\0")


def image_params(payload: bytes, w: int) -> tuple[int, int, int]:
    """(n, crc32 of the padded image, session) at width w."""
    p = padded(payload, w)
    crc = zlib.crc32(p)
    return len(p) // data_bytes(w), crc, zlib.crc32(p + bytes([w])) & 0xFF


class Encoder:
    """What streams.c data_line sends, row by row, for one width and row count."""

    def __init__(self, payload: bytes, w: int, rows: int = DATA_ROWS, groups: int = GROUPS, s: int = 0):
        self.w, self.rows, self.k, self.s = w, rows, groups, s
        self.D, self.B = data_bytes(w), line_bytes(w)
        self.image = padded(payload, w)
        self.n, self.crc, self.session = image_params(payload, w)
        self.last = rows - groups - 1       # last data row
        self.parity = [bytearray(self.D) for _ in range(groups)]

    def packet(self, row: int) -> bytes | None:
        D = self.D
        if row >= self.rows:
            return None
        if row == 0:
            self.parity = [bytearray(D) for _ in range(self.k)]
            idx = IDX_HEADER + self.k - 1
            data = struct.pack("<BBHI", self.session, self.rows, self.n & 0xFFFF, self.crc).ljust(D, b"\0")
        elif row > self.last:
            p = row - self.last - 1
            idx, data = IDX_PARITY + p, bytes(self.parity[p])
        else:
            idx = self.s % self.n
            self.s += 1
            data = self.image[idx * D:(idx + 1) * D]
            g = self.parity[(row - 1) % self.k]
            for i, b in enumerate(data):
                g[i] ^= b
        body = struct.pack(">H", idx) + data
        return body + struct.pack(">H", crc16(body))

    def bits(self, row: int, pk: bytes) -> np.ndarray:
        b = np.unpackbits(np.frombuffer(pk, np.uint8))
        return b ^ keystream(row, len(b))


def render_ire(bits: np.ndarray | None, w: int, black: float, white: float, lead: int = DATA_LEAD) -> np.ndarray:
    """One line's active video in IRE per DAC sample, as streams.c draws it."""
    s = np.full(ACTIVE, black, np.float64)
    if bits is None:
        return s
    cells = np.concatenate([[1, 1, 1, 1, 0], bits])
    s[lead:lead + len(cells) * w] = np.where(np.repeat(cells, w) != 0, white, black)
    return s


def to_de1(ire: np.ndarray, rng: np.random.Generator, bw_mhz: float, noise: float, jitter: float,
           shift: float) -> np.ndarray:
    """The ADV7180's 720 luma samples of a line: zero-order hold, Gaussian low-pass, resampled, noisy."""
    up = 16
    fine = np.repeat(ire, up)
    sigma = 0.1325 / (bw_mhz * 1e6) * DAC_HZ * up      # -3 dB at bw for a Gaussian
    k = np.arange(-int(4 * sigma) - 1, int(4 * sigma) + 2)
    g = np.exp(-0.5 * (k / sigma) ** 2)
    fine = np.convolve(fine, g / g.sum(), mode="same")
    walk = np.cumsum(rng.normal(0, jitter / np.sqrt(720), 720)) if jitter else np.zeros(720)
    pos = (np.arange(720) - DE1_X0 + walk + shift) / DE1_PER_DAC
    y_ire = np.interp(pos * up, np.arange(len(fine)), fine, left=ire[0], right=ire[-1])
    y = 16 + (y_ire - 7.5) * 219 / 92.5 + rng.normal(0, noise, 720)
    return np.clip(np.round(y), 1, 254).astype(np.uint8)


def mock_image(size: int = MOCK_SIZE) -> bytes:
    """A stand-in game: header, pseudo-random 'code', and runs of 00 and FF that the scrambler must break up."""
    rng = np.random.default_rng(0x56485347)
    body = bytearray(rng.integers(0, 256, size - 64, dtype=np.uint8).tobytes())
    body[0x8000:0xC000] = bytes(0x4000)
    body[0x20000:0x21000] = b"\xFF" * 0x1000
    text = b"VHSGame mock payload. " * 64
    body[0x100:0x100 + len(text)] = text
    hdr = struct.pack("<8sIIII", b"VHSGAME\0", 1, size, 0x40, zlib.crc32(body)).ljust(32, b"\0")
    return (hdr + b"MOCK GAME".ljust(32, b"\0")) + bytes(body)


PROGRAMMES = {"clips": 1, "sd": 3}     # streams.c ctl.src after the leader
# Test faults: header loads and verifies but the BIOS rejects it; crc never verifies at the DE1 (TAPE READ ERROR).
FAULTS = ("header", "crc")


def blob(payload: bytes, leader: int = 0, programme: str | None = None, fault: str | None = None) -> bytes:
    if len(payload) > FLASH_END - FLASH_OFFSET - HDR_SIZE:
        raise SystemExit("payload does not fit in flash after DATA_OFFSET")
    if fault == "header":
        payload = bytes([payload[0] ^ 0xFF]) + payload[1:]
    params = [image_params(payload, w) for w in WIDTHS]
    if fault == "crc":
        params = [(n, crc ^ 1, ses) for n, crc, ses in params]
    if any(n > MAX_PACKETS for n, _, _ in params):
        raise SystemExit(f"payload needs more than {MAX_PACKETS} packets at some width")
    body = payload.ljust(-(-len(payload) // 4) * 4, b"\0")
    total = int(np.frombuffer(body, "<u4").sum(dtype=np.uint64)) & 0xFFFFFFFF
    prog = 0x80000000 | PROGRAMMES[programme] if programme else 0
    hdr = struct.pack("<4I4I4I4BII", MAGIC, VERSION, len(payload), total,
                      *(n for n, _, _ in params), *(c for _, c, _ in params), *(s for _, _, s in params), leader,
                      prog)
    return hdr.ljust(HDR_SIZE, b"\0") + body


def read_blob(path: Path) -> bytes:
    b = path.read_bytes()
    magic, version, length = struct.unpack_from("<3I", b)
    if magic != MAGIC or version != VERSION:
        raise SystemExit(f"{path}: not a data tape blob")
    return b[HDR_SIZE:HDR_SIZE + length]


def cmd_bake(args) -> None:
    payload = args.payload.read_bytes() if args.payload else mock_image()
    b = blob(payload, args.leader, args.programme, args.fault)
    args.out.write_bytes(b)
    print(f"{args.out}: {len(payload)} byte payload, leader {args.leader or 'the GCV2 default'}, "
          f"programme {args.programme or 'the GCV2 default'}{f', FAULT {args.fault}' if args.fault else ''}")
    for w in WIDTHS:
        n, crc, ses = image_params(payload, w)
        fields = n / (DATA_ROWS - GROUPS - 1)
        print(f"  w={w}: {line_bytes(w)} B/line, {data_bytes(w)} data, {n} packets, crc32 {crc:08x}, "
              f"session {ses:02x}, one pass {fields:.0f} fields = {fields / 59.94:.2f} s")


def cmd_mock(args) -> None:
    args.out.write_bytes(mock_image())


def cmd_stim(args) -> None:
    rng = np.random.default_rng(args.seed)
    payload = mock_image()[:args.size]
    w = args.width
    enc = Encoder(payload, w, args.rows, args.groups)
    lines = []
    drop_left, drops = 0, 0
    for f in range(args.fields):
        for row in range(ROWS):
            pk = enc.packet(row)
            bits = enc.bits(row, pk) if pk is not None else None
            lead = DATA_LEAD + (int(rng.integers(-args.skew, args.skew + 1)) if args.skew else 0)
            ire = render_ire(bits, w, 7.5, 90.0, lead)
            if pk is not None and (drop_left or rng.random() < args.drop):
                drop_left = drop_left - 1 if drop_left else args.burst - 1
                drops += 1
                a = int(rng.integers(16, 600))
                ire[a:a + int(rng.integers(60, 360))] = 100.0 if rng.random() < 0.5 else 7.5
            lines.append(to_de1(ire, rng, args.bw, args.noise, args.jitter, rng.uniform(-0.5, 0.5)))
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    np.savetxt(out / "stim.hex", np.concatenate(lines), fmt="%02x")
    img = np.frombuffer(enc.image, np.uint8)
    np.savetxt(out / "image.hex", img, fmt="%02x")
    (out / "params.vh").write_text(
        f"localparam integer FIELDS = {args.fields};\nlocalparam integer N = {enc.n};\n"
        f"localparam integer D = {enc.D};\nlocalparam [31:0] CRC = 32'h{enc.crc:08x};\n"
        f"localparam [7:0] SESSION = 8'h{enc.session:02x};\nlocalparam integer ROWS_R = {args.rows};\n"
        f"localparam integer GROUPS = {args.groups};\n"
        f"localparam integer CLS = {w - 4};\n")
    print(f"{out}: {args.fields} fields, w={w}, n={enc.n}, D={enc.D}, crc {enc.crc:08x}, "
          f"{drops} dropped lines, {enc.s} packets sent")


# JTAG: probe DTAP (chipset video_top), fields MSB first.
PROBE = [("peek_idx", 15), ("peek", 136), ("session", 8), ("cls", 2), ("rows", 8), ("groups", 4), ("n", 16),
         ("have", 16), ("crc_exp", 32), ("crc_got", 32), ("have_session", 1), ("complete", 1), ("verified", 1),
         ("verify_ok", 1), ("lines_ok", 24), ("lines_bad", 24), ("lines_none", 24), ("recovered", 16),
         ("dupes", 16), ("data_fields", 16), ("load_fields", 16), ("lost", 8), ("drops", 8), ("misseq", 16)]
QUARTUS_BIN = Path(os.environ.get("QUARTUS_BIN", ""))   # unset: quartus_stp on PATH


def stp(script: str, *args: str) -> list[str]:
    """BITS lines of quartus_stp -t chipset/SCRIPT DE-SoC ARGS..."""
    r = subprocess.run([str(QUARTUS_BIN / "quartus_stp"), "-t", str(ROOT / "chipset" / script), "DE-SoC", *args],
                       capture_output=True, text=True)
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
    out["peek"] = out["peek"].to_bytes(17, "big")
    return out


def cmd_check(args) -> None:
    payload = read_blob(args.blob)
    if args.reset:
        stp("probe.tcl", "DTAP", "src 1000000000000000", "src 0000000000000000")
        print("loader reset")
    t0, s = time.monotonic(), None
    while True:
        s = parse(stp("probe.tcl", "DTAP", "read")[-1])
        w = WIDTHS[s["cls"]]
        n, crc, ses = image_params(payload, w)
        pct = 100 * s["have"] / s["n"] if s["n"] else 0
        print(f"{time.monotonic() - t0:6.1f} s  w={w} K={s['groups']} session {s['session']:02x} "
              f"have {s['have']}/{s['n']} ({pct:.1f}%)  lines ok {s['lines_ok']} bad {s['lines_bad']} "
              f"none {s['lines_none']}  recovered {s['recovered']} dupes {s['dupes']} misseq {s['misseq']}  "
              f"{'DROPPED LINES ' + str(s['drops']) + '  ' if s['drops'] else ''}"
              f"data fields {s['data_fields']}{'  LOST WRITES ' + str(s['lost']) if s['lost'] else ''}", flush=True)
        if not args.watch or s["verified"]:
            break
        time.sleep(args.interval)
    match = s["have_session"] and (s["session"], s["n"], s["crc_exp"]) == (ses, n, crc)
    print(f"header    {'matches the blob' if match else 'DIFFERS from the blob'} "
          f"(expect session {ses:02x}, n {n}, crc32 {crc:08x}; got {s['session']:02x}, {s['n']}, {s['crc_exp']:08x})")
    if s["complete"]:
        print(f"complete  after {s['load_fields']} data fields ({s['load_fields'] / 59.94:.2f} s)")
    if s["verified"]:
        print(f"verify    crc32 {s['crc_got']:08x}: {'OK' if s['verify_ok'] else 'MISMATCH'}")
    if args.dump:
        D = data_bytes(w)
        image = padded(payload, w)
        cnt = s["n"] if args.count is None else min(args.count, s["n"])
        bad = 0
        rows = stp("dtap.tcl", str(args.first), str(cnt))
        for bits in rows:
            p = parse(bits)
            i = p["peek_idx"]
            if p["peek"][:D] != image[i * D:(i + 1) * D]:
                bad += 1
                if bad <= 10:
                    print(f"  packet {i}: got {p['peek'][:D].hex()} want {image[i * D:(i + 1) * D].hex()}")
        print(f"dump      {len(rows)} packets read back, {bad} differ from the payload")
        if len(rows) != cnt:
            raise SystemExit("dump incomplete")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("bake")
    p.add_argument("payload", type=Path, nargs="?", help="game image (default: the mock)")
    p.add_argument("-o", "--out", type=Path, required=True)
    p.add_argument("--leader", type=int, default=0, help="leader fields on the GCV2 tape; 0 keeps its 600")
    p.add_argument("--programme", choices=PROGRAMMES, help="what follows the leader, even with an SD tape in")
    p.add_argument("--fault", choices=FAULTS, help="bake a bad image to test the BIOS error paths")
    p.set_defaults(fn=cmd_bake)
    p = sub.add_parser("mock")
    p.add_argument("-o", "--out", type=Path, required=True)
    p.set_defaults(fn=cmd_mock)
    p = sub.add_parser("stim")
    p.add_argument("--out-dir", type=Path, default=ROOT / "chipset/sim/out")
    p.add_argument("--size", type=int, default=6000, help="payload bytes (a prefix of the mock)")
    p.add_argument("--width", type=int, default=6, choices=WIDTHS)
    p.add_argument("--rows", type=int, default=DATA_ROWS)
    p.add_argument("--groups", type=int, default=GROUPS, choices=range(1, 9))
    p.add_argument("--fields", type=int, default=8)
    p.add_argument("--drop", type=float, default=0.0, help="chance a data line starts a dropout")
    p.add_argument("--burst", type=int, default=1)
    p.add_argument("--bw", type=float, default=3.0, help="luma bandwidth, MHz")
    p.add_argument("--noise", type=float, default=2.0, help="rms, luma codes")
    p.add_argument("--skew", type=int, default=0, help="+- DAC samples of line position")
    p.add_argument("--jitter", type=float, default=0.0, help="rms timing walk over a line, DE1 samples")
    p.add_argument("--seed", type=int, default=1)
    p.set_defaults(fn=cmd_stim)
    p = sub.add_parser("check")
    p.add_argument("--blob", type=Path, default=ROOT / "vcr_sim/build/datatape.bin")
    p.add_argument("--reset", action="store_true", help="clear the loader first")
    p.add_argument("--watch", action="store_true", help="poll until the image is verified")
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--dump", action="store_true", help="read every packet back and compare")
    p.add_argument("--first", type=int, default=0)
    p.add_argument("--count", type=int)
    p.set_defaults(fn=cmd_check)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
