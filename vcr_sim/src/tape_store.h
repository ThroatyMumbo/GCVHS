// SD card as an asset store, played by a schedule in flash (tools/tapestore.py). Core 0 only.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define STORE_MAGIC  0x41534856         // "VHSA": the card's sector 0
#define SCHED_MAGIC  0x53534856         // "VHSS": the schedule at flash SCHED_OFFSET
#define SCHED_OFFSET 0x30000            // 64 KiB up to CLIPS_OFFSET: flash_rp2350.sh sched=FILE

typedef struct {                        // the tape the schedule builds, as sdt_hdr_t has it
    uint32_t t_log2, layout, width, rows, aud_n;
    int32_t y_tab[16], u_tab[16], v_tab[16];
} store_geom_t;

typedef struct {
    uint32_t assets, entries, dead;     // dead: schedule entries naming no asset, or the wrong size
    uint32_t reads, line_misses, frame_reads, fails;
} store_stats_t;
extern store_stats_t store_stats;

// sector0: the card's first sector, already read. False unless it is a store and flash holds a schedule for it.
bool store_init(const uint8_t *sector0, store_geom_t *geom);
// Builds field f's CLIP_PAIRS row-pair records into rec, reading the card as it goes: call again while 0;
// 1 done, -1 a read failed.
int store_compose(uint8_t *rec, uint32_t f);
void store_abort(void);
