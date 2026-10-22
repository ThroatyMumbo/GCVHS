#!/usr/bin/env python3
"""Game CPU <-> DE1 link packets: codec, Questa stimulus for sim/tb_cpu.v and its scoring."""
import argparse
import json
import struct
from pathlib import Path

from link_proto import CRC_INIT, CRC_POLY, DN, REGS, STATUS_BYTES as SB, SYNC, TAP_BYTES, UP

ROOT = Path(__file__).resolve().parent.parent

REG_WR, FB_WR, FLIP, PAL_WR, AUD, IMG_RD, ECHO = (DN[k] for k in ("REG_WR", "FB_WR", "FLIP", "PAL_WR", "AUD",
                                                                 "IMG_RD", "ECHO"))
STATUS, AUD_REQ, IMG_DATA, TAPE_AUD, ECHO_UP = (UP[k] for k in ("STATUS", "AUD_REQ", "IMG_DATA", "TAPE_AUD", "ECHO"))
A = {r.name: r.addr for r in REGS}
TAP_LINES = 12


def tap_line(t: int) -> bytes:
    """What tb_cpu.v sends as tap line t."""
    return bytes((t * 11 + j * 3 + 1) & 0xFF for j in range(TAP_BYTES))


def crc16(data: bytes, crc: int = CRC_INIT) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def packet(typ: int, payload: bytes = b"", pad: bool = False) -> bytes:
    body = struct.pack("<BH", typ, len(payload)) + payload
    p = bytes([SYNC]) + body + struct.pack("<H", crc16(body))
    return p + b"\x00" * (-len(p) % 4) if pad else p


def parse(stream: bytes):
    """Yield (type, payload, crc_ok) for every A5-framed packet; skips padding and garbage."""
    i = 0
    while i + 6 <= len(stream):
        if stream[i] != SYNC:
            i += 1
            continue
        typ, n = struct.unpack_from("<BH", stream, i + 1)
        if i + 6 + n > len(stream):
            break
        body = stream[i + 1:i + 4 + n]
        (crc,) = struct.unpack_from("<H", stream, i + 4 + n)
        ok = crc == crc16(body)
        yield typ, bytes(stream[i + 4:i + 4 + n]), ok
        i = i + 6 + n if ok else i + 1


def reg_wr(pairs) -> bytes:
    return packet(REG_WR, b"".join(struct.pack("<BI", a, v) for a, v in pairs))


def img_byte(idx: int, sel: int) -> int:
    """What tb_cpu.v's slot model returns for byte sel of slot idx."""
    return (idx * 7 + sel * 13 + 1) & 0xFF


def status_vec() -> bytes:
    return bytes((i * 3 + 5) & 0xFF for i in range(SB))


def cmd_stim(args) -> None:
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    regs = [(A["OWN"], 1), (A["VIDEO"], 0x105), (A["AUDIO"], 0x6B), (A["FX"], 0x2AA), (A["COMB"], 1),
            (A["HUD"], 0x2345678), (A["OVERLAY"], 1), (A["LOADER"], 0), (A["IR_LO"], 0xDEADBEEF),
            (A["IR_HI"], 0x1234), (A["TAP"], 1), (A["WIN_PAN"], 0xFE08ABCD)]
    fb = bytes((i * 5) & 0xFF for i in range(300))
    pal = [(10, 20, 30, 0), (40, 50, 60, 1)]
    aud = [(i * 100 - 3000, -i * 77) for i in range(40)]
    echo = bytes(range(37))
    bad = bytearray(packet(ECHO, b"never echoed"))
    bad[8] ^= 0x40
    fb_cut = bytes((i * 3 + 1) & 0xFF for i in range(46))
    fb_slow = bytes((i * 9 + 2) & 0xFF for i in range(100))
    pal256 = [((i * 7) & 0xFF, (i * 13) & 0xFF, 255 - i, i & 1) for i in range(256)]
    bad_pal = bytearray(packet(PAL_WR, bytes([0, 1, 1, 2, 3, 0])))
    bad_pal[6] ^= 0x01
    aud_big = [((i * 37) & 0x7FFF, -((i * 53) & 0x7FFF)) for i in range(2068)]
    late = [(A["AUDIO"], 0xD5), (A["FX"], 0x1F0)]
    stall_regs = [(A["HUD"], 0x1234567), (A["OVERLAY"], 0), (A["TAP"], 0)]

    def aud_pk(frames):
        return packet(AUD, struct.pack("<H", len(frames)) + b"".join(struct.pack("<hh", l, r) for l, r in frames))

    stream = []                         # (byte, pause in us before it, hold the audio drain)

    def put(data, pause=0, hold=False):
        stream.extend((b, pause if k == 0 else 0, hold and k == 0) for k, b in enumerate(data))

    def put_slow(data, at, pause):
        stream.extend((b, pause if k == at else 0, False) for k, b in enumerate(data))

    for d in [b"\x00\x13",                                      # line noise before the first packet
              reg_wr(regs[:8]), reg_wr(regs[8:]),
              packet(REG_WR, struct.pack("<BI", A["VIDEO"], 0xFF) + b"\x09"),   # 6 bytes: not whole registers
              packet(FB_WR, struct.pack("<I", 76790) + fb[:20]),            # runs past the end of the buffer
              packet(FB_WR, struct.pack("<I", 1000) + fb),
              packet(PAL_WR, bytes([3, len(pal)]) + b"".join(bytes(e) for e in pal)),
              aud_pk(aud),
              bytes(bad),
              packet(ECHO, echo),
              packet(IMG_RD, struct.pack("<HH", 7, 3)),
              packet(FLIP),
              packet(0x55, b"xyz"),                             # unknown type: longer than it may be
              packet(ECHO, b"after")]:
        put(d)
    # Packets the CPU abandons (the next comes after more than the 10 us timeout), then ones that must all land.
    put(packet(FB_WR, struct.pack("<I", 2000) + bytes(300))[:8] + fb_cut)
    put(b"\xA5\x01", pause=12)
    put(packet(REG_WR, struct.pack("<BI", A["IR_HI"], 0x777))[:7], pause=12)
    put(reg_wr(late), pause=12)
    put_slow(packet(FB_WR, struct.pack("<I", 3000) + fb_slow), 60, 8)     # an 8 us pause inside: kept
    put(packet(ECHO, b"abandon-ok"))
    put(packet(PAL_WR, bytes([0, 0]) + b"".join(bytes(e) for e in pal256)))
    put(bytes(bad_pal))
    # The audio drain held so the parser stalls on aud_full for over 10 us, with register writes queued behind.
    put(aud_pk(aud_big[:1024]), hold=True)
    put(aud_pk(aud_big[1024:2048]))
    put(aud_pk(aud_big[2048:]))
    for r in stall_regs:
        put(reg_wr([r]))
    put(packet(ECHO, b"post-stall"))

    (out / "cpu_dn.hex").write_text("".join(f"{h << 16 | p << 8 | b:05x}\n" for b, p, h in stream))
    (out / "cpu_params.vh").write_text(f"localparam integer DN_BYTES = {len(stream)};\n"
                                       f"localparam integer TAP_LINES = {TAP_LINES};\n")
    final = dict(regs)
    final.update(late)
    final.update(stall_regs)
    truth = {"regs": sorted(final.items()),
             "fb": [[1000 + i, b] for i, b in enumerate(fb)] + [[76790 + i, fb[i]] for i in range(10)]
             + [[2000 + i, b] for i, b in enumerate(fb_cut)] + [[3000 + i, b] for i, b in enumerate(fb_slow)],
             "pal": [[3 + i, list(e)] for i, e in enumerate(pal)] + [[i, list(e)] for i, e in enumerate(pal256)],
             "pal_bursts": [len(pal), len(pal256)], "aud": aud + aud_big,
             "echo": [echo.hex(), b"after".hex(), b"abandon-ok".hex(), b"post-stall".hex()], "img": [7, 3],
             "packets": 21, "crc_errs": 2, "len_errs": 5, "ir_sends": 1, "reg_gap": 32, "stall": 2000}
    (out / "cpu_truth.json").write_text(json.dumps(truth))
    print(f"{out}: {len(stream)} DN bytes")


def cmd_score(args) -> None:
    out = args.out_dir
    t = json.loads((out / "cpu_truth.json").read_text())
    fails = []

    def check(name, got, want):
        if got != want:
            fails.append(f"{name}: got {got}, want {want}")

    lines = (out / "cpu_out.txt").read_text().split("\n")
    fb, pal, pal_cyc, aud, regs, up, cnt = [], [], [], [], {}, bytearray(), {}
    for ln in lines:
        f = ln.split()
        if not f:
            continue
        if f[0] == "FB":
            fb.append([int(f[1], 16), int(f[2], 16)])
        elif f[0] == "PAL":
            v = int(f[2], 16)
            pal.append([int(f[1], 16), [v >> 16 & 0xFF, v >> 8 & 0xFF, v & 0xFF, v >> 24]])
            pal_cyc.append(int(f[3]))
        elif f[0] == "AUD":
            v = int(f[1], 16)
            aud.append([struct.unpack("<h", struct.pack("<H", v & 0xFFFF))[0],
                        struct.unpack("<h", struct.pack("<H", v >> 16))[0]])
        elif f[0] == "UP":
            up.append(int(f[1], 16))
        elif f[0] == "REG":
            regs[int(f[1], 16)] = int(f[2], 16)
        elif f[0] == "CNT":
            cnt = {"packets": int(f[1]), "crc_errs": int(f[2]), "len_errs": int(f[3]), "tap_drops": int(f[4])}
        elif f[0] in ("IRS", "RWGAP", "STALL"):
            cnt[f[0]] = int(f[1])
    check("fb", sorted(fb), sorted(t["fb"]))
    check("pal", pal, t["pal"])
    runs = [1] if pal_cyc else []
    for a, b in zip(pal_cyc, pal_cyc[1:]):
        if b == a + 1:
            runs[-1] += 1
        else:
            runs.append(1)
    check("pal bursts (entries on consecutive clocks)", runs, t["pal_bursts"])
    check("ir sends", cnt.get("IRS"), t["ir_sends"])
    if cnt.get("RWGAP", 0) < t["reg_gap"]:
        fails.append(f"reg_wr pulses {cnt.get('RWGAP')} clocks apart, want >= {t['reg_gap']}")
    if cnt.get("STALL", 0) < t["stall"]:
        fails.append(f"longest aud_full stall {cnt.get('STALL')} clocks, want >= {t['stall']} (past the DN timeout)")
    check("aud", aud, [list(a) for a in t["aud"]])
    mask = {r.addr: r.mask for r in REGS}
    for a, v in t["regs"]:
        check(f"reg {a:02x}", regs.get(a), v & mask[a])
    for k in ("packets", "crc_errs", "len_errs"):
        check(k, cnt.get(k), t[k])
    pk = list(parse(bytes(up)))
    bad = [p for p in pk if not p[2]]
    check("up CRC errors", len(bad), 0)
    echoes = [p[1].hex() for p in pk if p[0] == ECHO_UP]
    check("echo", echoes, t["echo"])
    first, count = t["img"]
    imgs = [(struct.unpack_from("<H", p[1])[0], p[1][2:]) for p in pk if p[0] == IMG_DATA]
    want = [(i, bytes(img_byte(i, s) for s in range(17))) for i in range(first, first + count)]
    check("img", imgs, want)
    st = [p[1] for p in pk if p[0] == STATUS]
    check("status", st, [status_vec()])
    reqs = [struct.unpack("<H", p[1])[0] for p in pk if p[0] == AUD_REQ]
    steps = [(b - a) & 0xFFFF for a, b in zip([0] + reqs, reqs)]
    if not reqs or any(s not in (0, 512) for s in steps) or steps[0] != 512 or 0 not in steps:
        fails.append(f"aud_req totals (grants of 512, repeated while unmet): {reqs}")
    taps = [p[1] for p in pk if p[0] == TAPE_AUD]
    sent = [tap_line(t) for t in range(TAP_LINES)]
    check("tap spaced lines", taps[:TAP_LINES - 3], sent[:TAP_LINES - 3])
    if any(x not in sent[TAP_LINES - 3:] for x in taps[TAP_LINES - 3:]) or len(taps) + cnt.get("tap_drops", -1) != TAP_LINES:
        fails.append(f"tap burst: {len(taps)} lines, {cnt.get('tap_drops')} dropped, of {TAP_LINES}")
    print(f"UP: {len(pk)} packets ({len(reqs)} AUD_REQ to {reqs[-1] if reqs else 0} frames, {steps.count(0)} repeats, "
          f"{len(imgs)} IMG_DATA, {len(echoes)} ECHO, {len(st)} STATUS, "
          f"{len(taps)} TAPE_AUD, {cnt.get('tap_drops')} dropped)")
    print(f"DN: {cnt.get('packets')} packets, {cnt.get('crc_errs')} CRC errors, {cnt.get('len_errs')} length errors "
          f"or abandoned, {len(aud)} audio frames, PAL bursts {runs}, reg_wr pulses >= {cnt.get('RWGAP')} clocks apart, longest aud_full stall "
          f"{cnt.get('STALL')} clocks")
    for f in fails:
        print("FAIL", f)
    print("PASS" if not fails else f"{len(fails)} failures")
    raise SystemExit(1 if fails else 0)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(required=True)
    p = sub.add_parser("stim")
    p.add_argument("--out-dir", type=Path, default=ROOT / "chipset/sim/out")
    p.set_defaults(fn=cmd_stim)
    p = sub.add_parser("score")
    p.add_argument("--out-dir", type=Path, default=ROOT / "chipset/sim/out")
    p.set_defaults(fn=cmd_score)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
