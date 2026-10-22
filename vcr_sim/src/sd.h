// microSD over 4-bit SDIO on the cart bus pins: CLK GP16, CMD GP17, DAT0-3 GP18-21. Core 0 only.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define SD_RAW_BLOCK 520                // 512 data bytes, then each line's CRC-16 as 8 bytes

typedef struct {
    uint32_t reads, crc_errors, timeouts, cmd_errors;
    uint32_t stage;                     // last command sd_init sent, 100 once up
    uint32_t resp;                      // its response's content bits
    uint32_t fail;                      // last failed command: index | 0x100 no response, | 0x200 bad CRC or index
} sd_stats_t;
extern sd_stats_t sd_stats;

bool sd_init(uint32_t fast_div);        // sys clocks per SD clock once up (>= 13 keeps it under 25 MHz)
// Blocks land at raw + 512 k once sd_read_poll has returned 1; raw holds blocks * SD_RAW_BLOCK bytes.
bool sd_read_start(uint32_t sector, uint32_t blocks, uint8_t *raw);
int  sd_read_poll(void);                // 0 in progress, 1 done and CRC-clean, -1 failed
bool sd_read(uint32_t sector, uint32_t blocks, uint8_t *raw);  // blocking
uint32_t sd_status_test(uint32_t n);    // CMD13 n times: how many got no clean response
