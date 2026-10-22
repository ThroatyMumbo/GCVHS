#!/usr/bin/env python3
"""Test pattern (Y/U/V in IRE) and the exact 4fsc NTSC field waveform the RP2350 plays out."""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

FS_DAC = 315e6 / 22
SPL = 910
LINES = 262
HALF = SPL // 2
SYNC_LEN = 67
EQ_PULSE = 33
ACTIVE_START = 136
ACTIVE_LEN = 753
ACTIVE_LINE0 = 21
ROWS = 240
LINE_HZ = FS_DAC / SPL

BURST_START = SYNC_LEN + 9
BURST_LEN = 36
BURST_IRE = 20.0
SIN4 = np.array([0.0, 1.0, 0.0, -1.0])
COS4 = np.array([1.0, 0.0, -1.0, 0.0])

IRE_BLACK = 7.5
IRE_WHITE = 100.0
LUMA_SPAN = IRE_WHITE - IRE_BLACK

STAIR_ROWS = (12, 60)
STAIR_COLS = (48, 704)
STAIR_STEPS = 8
MB_ROWS = (66, 106)
MB_REF_WHITE = (48, 96)
MB_REF_BLACK = (96, 144)
MB_PACKETS = [(152, 282, 0.5e6), (292, 422, 1.0e6), (432, 562, 1.5e6), (572, 702, 2.0e6)]
MB_MEAN, MB_AMP = 50.0, 40.0
BAR_ROWS = (112, 160)
BAR_RGB = [(0.75, 0.75, 0.75), (0.75, 0.75, 0), (0, 0.75, 0.75), (0, 0.75, 0),
           (0.75, 0, 0.75), (0.75, 0, 0), (0, 0, 0.75), (0, 0, 0)]
BAR_NAMES = ["grey", "yellow", "cyan", "green", "magenta", "red", "blue", "black"]
HUE_ROWS = (164, 184)
HUE_Y, HUE_SAT = 50.0, 30.0
GEOM_ROWS = (190, 232)


def ire_to_code(ire):
    return np.clip(np.round((np.asarray(ire, float) + 40.0) / 180.0 * 63.0), 0, 63).astype(np.uint8)


def code_to_ire(code):
    return np.asarray(code, float) * 180.0 / 63.0 - 40.0


def rgb_to_yuv_ire(r: float, g: float, b: float) -> tuple[float, float, float]:
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return IRE_BLACK + LUMA_SPAN * y, LUMA_SPAN * 0.492 * (b - y), LUMA_SPAN * 0.877 * (r - y)


def yuv_ire_to_rgb(y, u, v) -> np.ndarray:
    yy = (np.asarray(y) - IRE_BLACK) / LUMA_SPAN
    b = yy + np.asarray(u) / LUMA_SPAN / 0.492
    r = yy + np.asarray(v) / LUMA_SPAN / 0.877
    g = (yy - 0.299 * r - 0.114 * b) / 0.587
    return np.clip(np.stack([r, g, b], axis=-1), 0.0, 1.0)


def stair_levels() -> np.ndarray:
    return np.linspace(IRE_BLACK, IRE_WHITE, STAIR_STEPS)


def stair_patch(i: int) -> tuple[int, int]:
    w = (STAIR_COLS[1] - STAIR_COLS[0]) // STAIR_STEPS
    return STAIR_COLS[0] + i * w, STAIR_COLS[0] + (i + 1) * w


bar_patch = stair_patch


def hue_cols() -> tuple[int, int]:
    return STAIR_COLS


def hue_at(c: np.ndarray) -> np.ndarray:
    c0, c1 = hue_cols()
    return (np.asarray(c, float) - c0) / (c1 - c0) * 2 * np.pi


def pattern_yuv() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    y = np.full((ROWS, ACTIVE_LEN), IRE_BLACK)
    u = np.zeros_like(y)
    v = np.zeros_like(y)
    w = IRE_WHITE

    for i, lvl in enumerate(stair_levels()):
        c0, c1 = stair_patch(i)
        y[STAIR_ROWS[0]:STAIR_ROWS[1], c0:c1] = lvl

    r0, r1 = MB_ROWS
    y[r0:r1, MB_REF_WHITE[0]:MB_REF_WHITE[1]] = w
    for c0, c1, f in MB_PACKETS:
        n = np.arange(c1 - c0)
        y[r0:r1, c0:c1] = MB_MEAN + MB_AMP * np.sin(2 * np.pi * f * n / FS_DAC)

    r0, r1 = BAR_ROWS
    for i, rgb in enumerate(BAR_RGB):
        c0, c1 = bar_patch(i)
        y[r0:r1, c0:c1], u[r0:r1, c0:c1], v[r0:r1, c0:c1] = rgb_to_yuv_ire(*rgb)

    r0, r1 = HUE_ROWS
    c0, c1 = hue_cols()
    h = hue_at(np.arange(c0, c1))
    y[r0:r1, c0:c1] = HUE_Y
    u[r0:r1, c0:c1] = HUE_SAT * np.cos(h)
    v[r0:r1, c0:c1] = HUE_SAT * np.sin(h)

    g0, g1 = GEOM_ROWS
    for c in range(48, 705, 82):
        y[g0:g1, c:c + 6] = 60.0
    for r in range(g0, g1):
        c = 48 + round((r - g0) * (704 - 48 - 8) / (g1 - g0 - 1))
        y[r, c:c + 8] = w
    y[g0 + 19:g0 + 23, 316:436] = w
    y[g0 + 4:g1 - 4, 373:379] = w
    y[g1 - 16:g1 - 4, 60:120] = w
    y[g1 - 10:g1 - 4, 640:660] = w

    for r in range(16, ROWS - 8, 16):
        y[r:r + 2, 8:40] = w
        u[r:r + 2, 8:40] = v[r:r + 2, 8:40] = 0
    for sl in (np.s_[0:3, :], np.s_[ROWS - 3:, :], np.s_[:, 0:8], np.s_[:, ACTIVE_LEN - 8:]):
        y[sl], u[sl], v[sl] = w, 0, 0
    return y, u, v


def pattern_ire() -> np.ndarray:
    return pattern_yuv()[0]


def field_ire() -> np.ndarray:
    """910x262 composite in IRE before DAC quantisation; 910 = 227.5 cycles, so phase flips each line."""
    wave = np.zeros((LINES, SPL))
    for ln in range(LINES):
        if ln <= 2 or 6 <= ln <= 8:
            wave[ln, 0:EQ_PULSE] = -40
            wave[ln, HALF:HALF + EQ_PULSE] = -40
        elif 3 <= ln <= 5:
            wave[ln, 0:HALF - SYNC_LEN] = -40
            wave[ln, HALF:SPL - SYNC_LEN] = -40
        else:
            wave[ln, 0:SYNC_LEN] = -40
    phase = (np.arange(LINES)[:, None] * SPL + np.arange(SPL)[None, :]) % 4
    sin_p, cos_p = SIN4[phase], COS4[phase]
    burst = np.s_[9:LINES, BURST_START:BURST_START + BURST_LEN]
    wave[burst] = -BURST_IRE * sin_p[burst]
    y, u, v = pattern_yuv()
    act = np.s_[ACTIVE_LINE0:ACTIVE_LINE0 + ROWS, ACTIVE_START:ACTIVE_START + ACTIVE_LEN]
    wave[act] = y + u * sin_p[act] + v * cos_p[act]
    return wave


def field_wave() -> np.ndarray:
    return ire_to_code(field_ire())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--emit", type=Path, help="write the 910x262 field as raw DAC bytes")
    ap.add_argument("--png", type=Path, help="write the ideal pattern as an RGB PNG")
    a = ap.parse_args()
    if a.emit:
        a.emit.parent.mkdir(parents=True, exist_ok=True)
        wave = field_wave()
        assert wave.size % 4 == 0
        a.emit.write_bytes(wave.tobytes())
    if a.png:
        from PIL import Image
        Image.fromarray((yuv_ire_to_rgb(*pattern_yuv()) * 255).astype(np.uint8)).save(a.png)


if __name__ == "__main__":
    main()
