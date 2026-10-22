#!/usr/bin/env python3
"""Bakes video clips into the flash image vhs_streams plays: one clip per stream, T=2 x quads."""
from __future__ import annotations

import argparse
import struct
import subprocess
import wave
from pathlib import Path

import numpy as np

FLASH_SIZE = 0xF00000                   # clips end here: datatape.py FLASH_OFFSET follows
CLIPS_OFFSET = 0x40000                  # flash offset; streams.c CLIPS_OFFSET
HDR_SIZE = 4096
MAGIC = 0x43534856                      # "VHSC"
VERSION = 1
SD_MAGIC = 0x54534856                   # "VHST": streams.c SDT_MAGIC
SD_FIRST = 2048                         # first field's sector: 1 MiB in

T_LOG2 = 1
LAYOUT_QUADS = 2
NSTREAMS = 8
AUD_N = 8
W_PIC = 376 - (2 + AUD_N * 6 + 4 + 2)   # streams.c W - AUD_W(8)
ROWS = 240
HS_BLANK = 4                            # last rows of each field left black: the deck's head switch lands there
PIC_ROWS = ROWS - HS_BLANK
QW, QH = W_PIC // 2, ROWS // 2
PAIRS = ROWS // 2
PAIR_BYTES = 2 * AUD_N + W_PIC + W_PIC // 4
FIELD_BYTES = PAIRS * PAIR_BYTES

FIELD_HZ = 315e6 / 22 / 910 / 262.5
AUD_HZ = PIC_ROWS * FIELD_HZ
FRAME_RATE = "30000/1001"               # one frame per 2 fields

IRE_BLACK, IRE_WHITE = 7.5, 100.0
SPAN = IRE_WHITE - IRE_BLACK
U_MAX = SPAN * 0.492 * 0.886
V_MAX = SPAN * 0.877 * 0.701
C_GAMMA = 1.6                           # finer chroma steps near grey

# Ordered dither over (row % 4, (x // 4) % 2): coarse horizontally so none of it lands near 3.58 MHz.
DITHER = (np.array([[0, 6], [4, 2], [1, 7], [5, 3]]) + 0.5) / 8


def tables() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    y = IRE_BLACK + SPAN * np.arange(16) / 15
    s = np.minimum(np.arange(16) - 7, 7)
    c = np.sign(s) * (np.abs(s) / 7) ** C_GAMMA
    return y, c * U_MAX, c * V_MAX


def ffmpeg_video(path: Path, frames: int, qh: int) -> np.ndarray:
    """Fitted into the quad's top qh rows; the rest stay black."""
    vf = (f"fps={FRAME_RATE},scale={QW}:{qh}:force_original_aspect_ratio=decrease,"
          f"pad={QW}:{qh}:-1:-1,pad={QW}:{QH}:0:0,format=rgb24")
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", str(path), "-an", "-vf", vf,
                          "-frames:v", str(frames), "-f", "rawvideo", "-"],
                         check=True, capture_output=True).stdout
    v = np.frombuffer(raw, np.uint8).reshape(-1, QH, QW, 3)
    if len(v) < frames:
        v = np.concatenate([v, np.repeat(v[-1:], frames - len(v), 0)])
    return v[:frames].astype(np.float32) / 255


MU = 255.0                              # compand=True: chipset audio_slicer expands it (REG_AUDIO bit 7)


def mu_compress(x: np.ndarray) -> np.ndarray:
    return np.sign(x) * np.log1p(MU * np.abs(x)) / np.log1p(MU)


def mu_rom(out: Path, full: int = 5930) -> None:
    """audio_slicer's expander: 257 points over |compressed| 0..1, as samples of the same full scale."""
    y = np.arange(257) / 256
    e = np.round(full * np.expm1(y * np.log1p(MU)) / MU).astype(int)
    out.write_text(f"// tools/clips.py mu_rom(): mu {MU:g}, full scale {full}; do not edit\n" +
                   "".join(f"{v:04x}\n" for v in e))


def ffmpeg_audio(path: Path, samples: int, rng: np.random.Generator, compand: bool = False) -> np.ndarray:
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", str(path), "-vn", "-ac", "1",
                          "-ar", str(round(AUD_HZ)), "-f", "s16le", "-"],
                         check=True, capture_output=True).stdout
    a = np.frombuffer(raw, "<i2").astype(np.float32)[:samples]
    a = np.pad(a, (0, samples - len(a)))
    peak = np.max(np.abs(a))
    a = a / peak * 120 if peak > 0 else a
    if compand:
        a = mu_compress(a / 127) * 127
    a += rng.random(samples) - rng.random(samples)
    return np.clip(np.round(a), -127, 127).astype(np.int8)


def encode_clip(rgb: np.ndarray, ytab, utab, vtab) -> tuple[np.ndarray, np.ndarray]:
    """rgb (frames, h, w, 3) -> luma levels (frames, h, w), chroma bytes (frames, h/2, w/4)."""
    f, h, w = rgb.shape[:3]
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    u = SPAN * 0.492 * (b - y)
    v = SPAN * 0.877 * (r - y)

    rows, cols = np.arange(h)[:, None] % 4, (np.arange(w)[None, :] // 4) % 2
    lev = np.floor(y * 15 + DITHER[rows, cols])
    lev = np.clip(lev, 0, 15).astype(np.uint8)

    ub = u.reshape(f, h // 2, 2, w // 4, 4).mean(axis=(2, 4))
    vb = v.reshape(f, h // 2, 2, w // 4, 4).mean(axis=(2, 4))
    ui = np.argmin(np.abs(ub[..., None] - utab[:15]), axis=-1)
    vi = np.argmin(np.abs(vb[..., None] - vtab[:15]), axis=-1)
    return lev, (ui | vi << 4).astype(np.uint8)


def bake(luma: list, chroma: list, audio: np.ndarray, fields: int) -> bytes:
    """Lays the clips out as the fields carry them: field f shows frame f >> 1 of clips (f & 1) * 4 + q."""
    L = np.zeros((fields, ROWS, W_PIC), np.uint8)
    C = np.zeros((fields, PAIRS, W_PIC // 4), np.uint8)
    for f in range(fields):
        for q in range(4):
            k, fr = (f & 1) * 4 + q, f >> 1
            r0, x0 = (q >> 1) * QH, (q & 1) * QW
            L[f, r0:r0 + QH, x0:x0 + QW] = luma[k][fr]
            C[f, r0 // 2:(r0 + QH) // 2, x0 // 4:(x0 + QW) // 4] = chroma[k][fr]
    lp = (L[..., 0::2] | L[..., 1::2] << 4).reshape(fields, PAIRS, W_PIC)
    a = np.zeros((AUD_N, fields, ROWS), np.int8)
    a[..., :PIC_ROWS] = audio.reshape(AUD_N, fields, PIC_ROWS)
    a = a.reshape(AUD_N, fields, PAIRS, 2).transpose(1, 2, 3, 0).reshape(fields, PAIRS, 2 * AUD_N)
    rec = np.concatenate([a.view(np.uint8), lp, C], axis=-1)
    assert rec.shape[-1] == PAIR_BYTES
    return rec.tobytes()


def header(fields: int, payload: bytes, ytab, utab, vtab) -> bytes:
    total = int(np.frombuffer(payload, "<u4").sum(dtype=np.uint64)) & 0xFFFFFFFF
    q = lambda t: [int(round(x * 256)) for x in t]
    h = struct.pack("<10I48i", MAGIC, VERSION, T_LOG2, LAYOUT_QUADS, W_PIC, PIC_ROWS, fields, PAIR_BYTES,
                    AUD_N, total, *q(ytab), *q(utab), *q(vtab))
    return h.ljust(HDR_SIZE, b"\0")


def write_sd(path: Path, fields: int, payload: bytes, ytab, utab, vtab) -> None:
    """streams.c SD tape: header in sector 0, field f's records from SD_FIRST + f * field_sectors."""
    field_sectors = -(-FIELD_BYTES // 512)
    q = lambda t: [int(round(x * 256)) for x in t]
    h = struct.pack("<12I48i", SD_MAGIC, 1, T_LOG2, LAYOUT_QUADS, W_PIC, PIC_ROWS, fields, PAIR_BYTES, AUD_N,
                    field_sectors, SD_FIRST, 0, *q(ytab), *q(utab), *q(vtab))
    pad = b"\0" * (field_sectors * 512 - FIELD_BYTES)
    with path.open("wb") as f:
        f.write(h.ljust(SD_FIRST * 512, b"\0"))
        for k in range(fields):
            f.write(payload[k * FIELD_BYTES:(k + 1) * FIELD_BYTES] + pad)


def to_rgb(Y: np.ndarray, U: np.ndarray, V: np.ndarray) -> np.ndarray:
    y = (Y - IRE_BLACK) / SPAN
    b = y + U / (SPAN * 0.492)
    r = y + V / (SPAN * 0.877)
    g = (y - 0.299 * r - 0.114 * b) / 0.587
    return (np.clip(np.stack([r, g, b], -1), 0, 1) * 255).astype(np.uint8)


def preview(blob: bytes, out: Path) -> None:
    from PIL import Image
    out.mkdir(parents=True, exist_ok=True)
    hdr = struct.unpack_from("<10I48i", blob)
    fields = hdr[6]
    ytab, utab, vtab = (np.array(hdr[10 + 16 * i:26 + 16 * i]) / 256 for i in range(3))
    rec = np.frombuffer(blob, np.uint8, offset=HDR_SIZE).reshape(fields, PAIRS, PAIR_BYTES)
    lb = rec[..., 2 * AUD_N:2 * AUD_N + W_PIC].reshape(fields, ROWS, W_PIC // 2)
    lev = np.stack([lb & 15, lb >> 4], -1).reshape(fields, ROWS, W_PIC)
    cb = np.repeat(np.repeat(rec[..., 2 * AUD_N + W_PIC:], 4, axis=2), 2, axis=1)
    img = to_rgb(ytab[lev], utab[cb & 15], vtab[cb >> 4])

    Image.fromarray(img[0]).resize((W_PIC * 2, ROWS * 2), Image.NEAREST).save(out / "field0.png")
    picks = np.linspace(0, fields // 2 - 1, 6).astype(int)
    for k in range(NSTREAMS):
        q, ph = k & 3, k >> 2
        r0, x0 = (q >> 1) * QH, (q & 1) * QW
        tiles = [img[2 * fr + ph, r0:r0 + QH, x0:x0 + QW] for fr in picks]
        Image.fromarray(np.concatenate(tiles, 1)).resize((QW * 12, QH * 2), Image.NEAREST) \
            .save(out / f"stream{k}.png")

        a = rec[..., np.arange(2) * AUD_N + k].reshape(fields, ROWS)[:, :PIC_ROWS].reshape(-1).view(np.int8)
        with wave.open(str(out / f"stream{k}.wav"), "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(round(AUD_HZ))
            w.writeframes((a.astype("<i2") << 8).tobytes())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("videos", type=Path, help="directory of clips named 1, 2, ...: clip 1 is stream 0; other files are ignored")
    ap.add_argument("-o", "--out", type=Path, required=True)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--preview", type=Path, help="write per-stream PNGs and WAVs decoded from the blob")
    ap.add_argument("--sd", action="store_true", help="write a raw SD card image (ctl.src 3) instead: no length limit")
    args = ap.parse_args()

    if not args.videos.is_dir():
        ap.error(f"{args.videos} missing: put {NSTREAMS} clips in it named 1 to {NSTREAMS} (any ffmpeg format)")
    clips = sorted((p for p in args.videos.iterdir() if p.is_file() and p.stem.isdigit()), key=lambda p: int(p.stem))
    if len(clips) != NSTREAMS:
        ap.error(f"need {NSTREAMS} clips, found {len(clips)}")
    fit = (FLASH_SIZE - CLIPS_OFFSET - HDR_SIZE) // FIELD_BYTES
    fields = int(round(args.seconds * FIELD_HZ / 4)) * 4
    if not args.sd:
        fields = min(fields, fit // 4 * 4)

    ytab, utab, vtab = tables()
    rng = np.random.default_rng(1)
    luma, chroma, audio = [], [], []
    for k, p in enumerate(clips):
        bottom = (k & 3) >> 1               # quads 2 and 3 end on the field's last rows
        l, c = encode_clip(ffmpeg_video(p, fields // 2, QH - bottom * HS_BLANK), ytab, utab, vtab)
        luma.append(l)
        chroma.append(c)
        audio.append(ffmpeg_audio(p, fields * PIC_ROWS, rng))
    payload = bake(luma, chroma, np.stack(audio), fields)
    if args.sd:
        write_sd(args.out, fields, payload, ytab, utab, vtab)
        print(f"{args.out}: SD image, {fields} fields ({fields / FIELD_HZ:.2f} s), "
              f"{args.out.stat().st_size / 2**20:.1f} MiB")
        return
    blob = header(fields, payload, ytab, utab, vtab) + payload
    args.out.write_bytes(blob)
    print(f"{args.out}: {fields} fields ({fields / FIELD_HZ:.2f} s), {len(blob) / 2**20:.2f} MiB "
          f"of {(FLASH_SIZE - CLIPS_OFFSET) / 2**20:.2f}")
    for k, p in enumerate(clips):
        print(f"  stream {k}: {p.name}")
    if args.preview:
        preview(blob, args.preview)


if __name__ == "__main__":
    main()
