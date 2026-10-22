#!/usr/bin/env python3
"""Game CPU <-> DE1 link protocol tables: writes cpu/src/link_proto.h and
chipset/src/cpu/link_proto.vh; --check fails if either is stale."""
import argparse
import sys
from pathlib import Path
from typing import NamedTuple, Optional

ROOT = Path(__file__).resolve().parent.parent
C_OUT = ROOT / "cpu/src/link_proto.h"
V_OUT = ROOT / "chipset/src/cpu/link_proto.vh"

SYNC = 0xA5
CRC_POLY, CRC_INIT = 0x1021, 0xFFFF     # CRC-16/CCITT-FALSE over type, length and payload

DN = {"REG_WR": 0x01, "FB_WR": 0x02, "FLIP": 0x03, "PAL_WR": 0x04, "AUD": 0x05, "IMG_RD": 0x06, "ECHO": 0x07}
UP = {"STATUS": 0x81, "AUD_REQ": 0x82, "IMG_DATA": 0x83, "TAPE_AUD": 0x84, "ECHO": 0x87}

FB_CHUNK, AUD_MAX, ECHO_MAX, REG_MAX, PAL_MAX = 4096, 1024, 256, 8, 256
OUT_W, OUT_H = 720, 480
FB_W, FB_H, FB_X0 = 320, 240, 40
STATUS_BYTES, TAP_BYTES = 64, 22

# Longest DN payloads, bytes; REG_WR also has to be whole 5-byte registers.
DN_MAX = {"REG_WR": 5 * REG_MAX, "FB_WR": 4 + FB_CHUNK, "FLIP": 0, "PAL_WR": 2 + 4 * PAL_MAX,
          "AUD": 2 + 4 * AUD_MAX, "IMG_RD": 4, "ECHO": ECHO_MAX}
# UP payloads: IMG_DATA is this header and then the slot's d bytes.
UP_LEN = {"STATUS": STATUS_BYTES, "AUD_REQ": 2, "IMG_DATA": 2, "TAPE_AUD": TAP_BYTES}


class Reg(NamedTuple):
    name: str
    addr: int
    mask: int               # value bits the DE1 keeps
    reset: Optional[int]    # after a link reset, and while the CPU doesn't own the controls; None: not reset
    doc: str


REGS = [
    Reg("OWN", 0x00, 0x1, 0, "bit 0: the CPU owns the DE1's controls"),
    Reg("VIDEO", 0x01, 0x1FF, 0, "3:0 stream, 4 mosaic, 5 flip test, 8 raw"),
    Reg("AUDIO", 0x02, 0xFF, 0,
        "2:0 stream, 3 own stream, 4 show strip, 5 line-in off, 6 tape audio off, 7 mu-law expand"),
    Reg("FX", 0x03, 0x3FF, 0, "8:0 effects, 9 HUD"),
    Reg("COMB", 0x04, 0x1, 0, "bit 0: comb filters off"),
    Reg("HUD", 0x05, 0x3FFFFFF, 0, "3:0 hp, 5:4 lives, 25:6 score (BCD)"),
    Reg("OVERLAY", 0x06, 0x1, 0, "bit 0: overlay on"),
    Reg("LOADER", 0x07, 0x1, 0, "bit 0: hold the data loader in reset"),
    Reg("IR_LO", 0x08, 0xFFFFFFFF, None, "IR payload [31:0]"),
    Reg("IR_HI", 0x09, 0xFFFF, None, "IR payload [47:32]; writing sends"),
    Reg("WIN_POS", 0x0A, 0x1FF03FF, 0, "video window, output pixels: 9:0 x, 24:16 y"),
    Reg("WIN_SIZE", 0x0B, 0x1FF03FF, OUT_H << 16 | OUT_W, "9:0 w, 24:16 h; 0 hides the video"),
    Reg("WIN_HSTEP", 0x0C, 0xFFFFF, 1 << 16, "Q16 full-screen source pixels per output pixel: 720 / w"),
    Reg("WIN_VSTEP", 0x0D, 0xFFFFF, (240 << 16) // OUT_H, "Q16 display rows per output line: 240 / h"),
    Reg("OV_HWIN", 0x0E, 0x3FF03FF, 2 * FB_W << 16 | FB_X0, "overlay columns, output pixels: 9:0 x, 25:16 w"),
    Reg("OV_HSTEP", 0x0F, 0x1FFFFFF, 1 << 15,
        "15:0 Q16 framebuffer pixels per output pixel, 24:16 first framebuffer x"),
    Reg("TAP", 0x10, 0x1, 0, "bit 0: send TAPE_AUD for every line with in-line audio"),
    Reg("WIN_PAN", 0x11, 0x1FFFFFF, 0,
        "15:0 Q16 pan into the wrap period, 23:16 Q8 inset per side of the region, 24 wrap on"),
]
REG = {r.name: r for r in REGS}

# STATUS flags bytes 5 and 9, bit 0 first.
ST0 = [("FRONT", ""), ("TAG_SEEN", ""), ("FROZEN", ""), ("RAW", ""), ("OWN", ""), ("IN_LOCK", ""),
       ("LTC_LOCKED", ""), ("AUD_SEEN", "")]
ST1 = [("DT_SESSION", ""), ("DT_COMPLETE", ""), ("DT_VERIFIED", ""), ("DT_OK", ""), ("COMB_OFF", ""),
       ("MOSAIC", ""), ("DATA_LIVE", "a good data line in the last two fields"),
       ("BLUE", "the picture is blue-back: deck stopped, winding or empty")]

C_STRUCTS = """\
typedef struct __attribute__((packed)) {
    uint32_t frame;         // output frames
    uint8_t flips;          // overlay flips applied
    uint8_t flags0;         // ST0_*
    uint8_t tag_scheme;     // {t_log2[1:0], layout[1:0]}: T = 1 << t_log2; layout full/halves/quads/lines
    uint8_t tag_count;      // bit 7: 240p; field count
    uint8_t stream_aud;     // 3:0 video stream shown, 6:4 audio stream played
    uint8_t flags1;         // ST1_*
    uint8_t dt_session;
    uint8_t dt_d;           // data bytes per tape slot
    uint16_t dt_n;          // slots in the image
    uint16_t dt_have;
    uint32_t dt_crc;        // CRC-32 of the image padded to n * d
    uint32_t ltc_field;     // 2 * frame + field, while ltc locked
    uint32_t ltc_tc;        // BCD hhmmssff
    uint32_t ltc_user;
    uint32_t vitc_pos;      // 2 * frame + field mark
    uint32_t vitc_tc;
    uint8_t vitc_age;       // fields since the last good VITC word (63 = never)
    uint8_t ptag_age;
    uint16_t ptag_agreed;   // low 16 bits of the position, pause and cue
    uint16_t aud_rate;      // in-line audio samples per second
    uint16_t ir_seq;        // IR frames received
    uint16_t dn_packets;
    uint16_t dn_crc_errs;
    uint16_t dn_len_errs;
    uint16_t up_packets;
    uint16_t cpu_aud_under; // CPU audio frames played as silence
    uint16_t tag_bad;
    uint8_t key_resets;     // KEY[0] presses since the DE1 was configured, 1-255; 0 none
    uint8_t reserved;
    uint16_t tap_drops;     // TAPE_AUD lines dropped with the last still waiting
} vhs_status_t;

_Static_assert(sizeof(vhs_status_t) == LINK_STATUS_BYTES, "vhs_status_t is the DE1's STATUS payload");

// TAPE_AUD: one stored line's in-line audio cells, raw sums of 6 luma samples (audio_slicer.v).
#define TAP_LOCKED 0x01000000u
typedef struct __attribute__((packed)) {
    uint32_t field;         // 23:0 LTC field the line was stored in; TAP_LOCKED while LTC is locked
    uint8_t row;            // stored line within the field
    uint8_t pad;
    uint16_t cell[8];
} vhs_tap_t;

_Static_assert(sizeof(vhs_tap_t) == LINK_TAP_BYTES, "vhs_tap_t is the DE1's TAPE_AUD payload");
"""

BANNER = "Generated by tools/link_proto.py: edit its tables and rerun it, never this file."


def c_header() -> str:
    o = [f"// {BANNER}",
         "// Game CPU <-> DE1 link packets, registers and STATUS layout (chipset/src/cpu/).",
         "#ifndef LINK_PROTO_H", "#define LINK_PROTO_H", "", "#include <stdint.h>", "",
         f"#define LINK_SYNC 0x{SYNC:02X}",
         f"#define LINK_CRC_POLY 0x{CRC_POLY:04X}",
         f"#define LINK_CRC_INIT 0x{CRC_INIT:04X}", "", "enum {"]
    o += [f"    DN_{k} = 0x{v:02X}," for k, v in DN.items()]
    o += [f"    UP_{k} = 0x{v:02X}," for k, v in UP.items()]
    o += ["};", "",
          f"#define LINK_FB_CHUNK   {FB_CHUNK}u",
          f"#define LINK_AUD_MAX    {AUD_MAX}u",
          f"#define LINK_ECHO_MAX   {ECHO_MAX}u",
          f"#define LINK_REG_MAX    {REG_MAX}u", "", "enum {"]
    o += [f"    {f'REG_{r.name} = 0x{r.addr:02X},':<24}// {r.doc}" for r in REGS]
    o += ["};", "", "// Register values after a link reset, and while the CPU doesn't own the controls."]
    o += [f"#define {f'REG_{r.name}_RESET':<24}0x{r.reset:08X}u" for r in REGS if r.reset is not None]
    o += ["",
          f"#define LINK_OUT_W  {OUT_W}     // output raster; the {FB_W}x{FB_H} framebuffer is doubled into x "
          f"{FB_X0}-{FB_X0 + 2 * FB_W - 1}",
          f"#define LINK_OUT_H  {OUT_H}",
          f"#define LINK_FB_W   {FB_W}",
          f"#define LINK_FB_H   {FB_H}",
          f"#define LINK_FB_X0  {FB_X0}", "",
          f"#define LINK_STATUS_BYTES {STATUS_BYTES}",
          f"#define LINK_TAP_BYTES    {TAP_BYTES}", ""]
    for pre, bits in (("ST0", ST0), ("ST1", ST1)):
        for i, (n, doc) in enumerate(bits):
            line = f"#define {f'{pre}_{n}':<16}0x{1 << i:02X}"
            o.append(f"{line}    // {doc}" if doc else line)
        o.append("")
    o += [C_STRUCTS.rstrip("\n"), "", "#endif", ""]
    return "\n".join(o)


def v_header() -> str:
    def lp(width, items):
        return [f"localparam [{width - 1}:0] {n} = {width}'h{v:0{(width + 3) // 4}X};" for n, v in items]

    o = [f"// {BANNER}", "// Link protocol constants; include inside a module.",
         *lp(8, [("LINK_SYNC", SYNC)]),
         *lp(16, [("LINK_CRC_POLY", CRC_POLY), ("LINK_CRC_INIT", CRC_INIT)]),
         *lp(8, [(f"DN_{k}", v) for k, v in DN.items()]),
         *lp(8, [(f"UP_{k}", v) for k, v in UP.items()]),
         *[f"localparam integer DN_{k}_MAX = {v};" for k, v in DN_MAX.items()],
         *[f"localparam integer UP_{k}_LEN = {v};" for k, v in UP_LEN.items()],
         f"localparam integer LINK_REG_MAX = {REG_MAX};",
         *lp(8, [(f"REG_{r.name}", r.addr) for r in REGS]),
         *lp(32, [(f"REG_{r.name}_RESET", r.reset) for r in REGS if r.reset is not None]),
         f"localparam integer LINK_OUT_W = {OUT_W}, LINK_OUT_H = {OUT_H};",
         f"localparam integer LINK_FB_W = {FB_W}, LINK_FB_H = {FB_H}, LINK_FB_X0 = {FB_X0};",
         f"localparam integer LINK_STATUS_BYTES = {STATUS_BYTES}, LINK_TAP_BYTES = {TAP_BYTES};", ""]
    return "\n".join(o)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true", help="fail if a generated file differs, write nothing")
    args = ap.parse_args()
    stale = []
    for path, text in ((C_OUT, c_header()), (V_OUT, v_header())):
        if path.exists() and path.read_text() == text:
            continue
        stale.append(path.relative_to(ROOT))
        if not args.check:
            path.write_text(text)
    if args.check and stale:
        sys.exit("stale, run tools/link_proto.py: " + ", ".join(map(str, stale)))
    print(("written: " + ", ".join(map(str, stale))) if stale else "up to date")


if __name__ == "__main__":
    main()
