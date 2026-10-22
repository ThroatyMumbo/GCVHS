#!/usr/bin/env python3
"""The GCV2's SD asset store and flash schedule (vcr_sim/src/tape_store.c):
write a store image (dd it to the card once), write a schedule (flash_rp2350.sh sched=FILE), list either."""
from __future__ import annotations

import argparse
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

STORE_MAGIC = 0x41534856                # "VHSA"
SCHED_MAGIC = 0x53534856                # "VHSS"
SCHED_FLASH = 0x10030000                # tape_store.h SCHED_OFFSET
SCHED_MAX = 0x10000
DIR_ENTRY = struct.Struct("<24s7I12x")  # tape_store.c dir_entry_t
SCHED_HDR = struct.Struct("<8I48i")
SCHED_ENTRY = struct.Struct("<24sBB2xIIiIIII")
ALIGN = 4                               # sectors: tape_store.c LINE_SECT
AUDIO, VIDEO = 0, 1
FOREVER = 0xFFFFFFFF


@dataclass
class Asset:
    name: str
    kind: int
    data: bytes
    units: int
    unit_bytes: int = 1
    stride: int = 0                     # sectors a frame
    w: int = 0
    h: int = 0


@dataclass
class Entry:
    name: str
    kind: int
    target: int                         # audio cell, or video stream
    first: int                          # SD fields [first, end)
    end: int = FOREVER
    offset: int = 0                     # asset unit at field first, row 0
    win0: int = 0
    win1: int = 0                       # 0: the whole asset
    loop0: int = 0
    loop1: int = 0                      # nonzero: units from loop1 on repeat [loop0, loop1)


def audio_asset(name: str, samples: np.ndarray) -> Asset:
    """int8 in-line audio samples, one per picture row, as the cells carry them (mu-law or linear)."""
    a = np.asarray(samples, np.int8)
    return Asset(name, AUDIO, a.tobytes(), len(a))


def video_asset(name: str, luma: np.ndarray, chroma: np.ndarray) -> Asset:
    """Frames for one region: luma levels (n, h, w) 0-15, chroma bytes (n, h/2, w/4) as clips.encode_clip makes."""
    n, h, w = luma.shape
    lp = (luma[..., 0::2] | luma[..., 1::2] << 4).astype(np.uint8).reshape(n, -1)
    frames = np.concatenate([lp, chroma.astype(np.uint8).reshape(n, -1)], axis=1)
    stride = -(-frames.shape[1] // 512)
    padded = np.zeros((n, stride * 512), np.uint8)
    padded[:, :frames.shape[1]] = frames
    return Asset(name, VIDEO, padded.tobytes(), n, frames.shape[1], stride, w, h)


def write_store(path: Path, assets: list[Asset]) -> list[dict]:
    names = [a.name for a in assets]
    if len(set(names)) != len(names) or any(len(n.encode()) > 24 for n in names):
        raise SystemExit("asset names must be unique and at most 24 bytes")
    dir_sectors = -(-len(assets) // 8)
    at = -(-(1 + dir_sectors) // ALIGN) * ALIGN
    rows, blobs = [], []
    for a in assets:
        sectors = -(-len(a.data) // 512)
        rows.append(dict(name=a.name, kind=a.kind, first=at, units=a.units, unit_bytes=a.unit_bytes,
                         stride=a.stride, w=a.w, h=a.h))
        blobs.append((at, a.data))
        at += -(-sectors // ALIGN) * ALIGN
    with path.open("wb") as f:
        f.write(struct.pack("<4I", STORE_MAGIC, 1, len(assets), 1).ljust(512, b"\0"))
        d = b"".join(DIR_ENTRY.pack(r["name"].encode(), r["kind"], r["first"], r["units"], r["unit_bytes"],
                                    r["stride"], r["w"], r["h"]) for r in rows)
        f.write(d.ljust(dir_sectors * 512, b"\0"))
        for first, data in blobs:
            f.seek(first * 512)
            f.write(data)
        f.truncate(at * 512)
    return rows


def read_store(path: Path) -> list[dict]:
    with path.open("rb") as f:
        magic, version, n, dir_first = struct.unpack("<4I", f.read(16))
        if magic != STORE_MAGIC or version != 1:
            raise SystemExit(f"{path}: not an asset store")
        f.seek(dir_first * 512)
        raw = f.read(n * DIR_ENTRY.size)
    keys = ("name", "kind", "first", "units", "unit_bytes", "stride", "w", "h")
    out = []
    for i in range(n):
        v = DIR_ENTRY.unpack_from(raw, i * DIR_ENTRY.size)
        out.append(dict(zip(keys, (v[0].rstrip(b"\0").decode(), *v[1:]))))
    return out


def write_schedule(path: Path, t_log2: int, layout: int, width: int, rows: int, aud_n: int, tabs,
                   entries: list[Entry]) -> None:
    q = lambda t: [int(round(x * 256)) for x in t]
    b = SCHED_HDR.pack(SCHED_MAGIC, 1, t_log2, layout, width, rows, aud_n, len(entries),
                       *q(tabs[0]), *q(tabs[1]), *q(tabs[2]))
    b += b"".join(SCHED_ENTRY.pack(e.name.encode(), e.kind, e.target, e.first, e.end, e.offset,
                                   e.win0, e.win1, e.loop0, e.loop1) for e in entries)
    if len(b) > SCHED_MAX:
        raise SystemExit(f"schedule is {len(b)} bytes; flash holds {SCHED_MAX}")
    path.write_bytes(b)


def read_schedule(path: Path) -> tuple[tuple, list[Entry]]:
    b = path.read_bytes()
    h = SCHED_HDR.unpack_from(b)
    if h[0] != SCHED_MAGIC:
        raise SystemExit(f"{path}: not a schedule")
    es = []
    for i in range(h[7]):
        v = SCHED_ENTRY.unpack_from(b, SCHED_HDR.size + i * SCHED_ENTRY.size)
        es.append(Entry(v[0].rstrip(b"\0").decode(), *v[1:]))
    return h[2:7], es


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ls", help="an asset store image's directory").add_argument("image", type=Path)
    sub.add_parser("sched", help="a schedule's entries, checked against a store").add_argument("sched", type=Path)
    ap.add_argument("--store", type=Path)
    args = ap.parse_args()
    if args.cmd == "ls":
        for r in read_store(args.image):
            kind = "audio" if r["kind"] == AUDIO else f"video {r['w']}x{r['h']}"
            print(f"{r['name']:24} {kind:14} {r['units']:9} units  sector {r['first']}")
        return
    geom, es = read_schedule(args.sched)
    names = {r["name"] for r in read_store(args.store)} if args.store else None
    print("t_log2 {} layout {} width {} rows {} cells {}".format(*geom))
    for e in es:
        miss = "  MISSING" if names is not None and e.name not in names else ""
        end = "forever" if e.end == FOREVER else e.end
        print(f"{'audio' if e.kind == AUDIO else 'video'} {e.target}  {e.name:24} fields {e.first}-{end} "
              f"offset {e.offset} window {e.win0}-{e.win1} loop {e.loop0}-{e.loop1}{miss}")


if __name__ == "__main__":
    main()
