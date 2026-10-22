// microSD over 4-bit SDIO: PIO1 runs the CMD and DAT lines against a PWM clock; a multi-block read DMAs
// raw blocks (data plus CRCs) and is CRC-checked and compacted in place a few blocks per poll.
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "sd.h"
#include "sdio.pio.h"

#define CMD_PIN (CLK_PIN + 1)
#define DAT0_PIN (CLK_PIN + 2)
#define NIBBLES (2 * SD_RAW_BLOCK)
#define CHECK_PER_POLL 8
enum { R_NONE, R_48, R_48_NOCRC, R_136 };
enum { RD_IDLE, RD_DATA, RD_BUSY, RD_CHECK };

sd_stats_t sd_stats;
static PIO const pio = pio1;
static PIO const pio_rx = pio0;         // the two programs overfill one PIO; pio0 has the DAC (sm0) and I2S (sm1)
#define SM_RX 3
static uint sm_cmd, off_cmd, off_rx, dma;
static bool hw_up, ccs;
static uint32_t gap_us;                 // 16 clocks: a card wants 8 after a response before the next command
static uint32_t rca;
static uint16_t crc16_tab[256];
static uint32_t lane[256];              // a byte's two bits per DAT line: line k's at the top of byte k

static int rd_state = RD_IDLE;
static uint8_t *rd_raw;
static uint32_t rd_blocks, rd_k, rd_t0;
static bool rd_bad;

static void clock_set(uint32_t sys_per_clk) {
    uint slice = pwm_gpio_to_slice_num(CLK_PIN);
    uint32_t div = (sys_per_clk + 255) / 256, top = sys_per_clk / div;
    gap_us = 16 * div * top / (clock_get_hz(clk_sys) / 1000000) + 1;
    pwm_set_enabled(slice, false);
    pwm_set_clkdiv_int_frac(slice, (uint8_t)div, 0);
    pwm_set_wrap(slice, (uint16_t)(top - 1));
    pwm_set_chan_level(slice, pwm_gpio_to_channel(CLK_PIN), (uint16_t)(top / 2));
    pwm_set_counter(slice, 0);
    pwm_set_enabled(slice, true);
}

static void sm_reset(PIO p, uint sm, uint off, bool out) {
    pio_sm_set_enabled(p, sm, false);
    pio_sm_clear_fifos(p, sm);
    pio_sm_restart(p, sm);
    if (out) pio_sm_set_consecutive_pindirs(p, sm, CMD_PIN, 1, false);
    pio_sm_exec(p, sm, pio_encode_jmp(off));
    pio_sm_set_enabled(p, sm, true);
}

static void cmd_reset(void) { sm_reset(pio, sm_cmd, off_cmd + sdio_cmd_offset_start, true); }
static void rx_reset(void) { sm_reset(pio_rx, SM_RX, off_rx + sdio_rx_offset_start, false); }

static void tables(void) {
    for (int i = 0; i < 256; i++) {
        uint16_t d = (uint16_t)(i << 8);
        for (int k = 0; k < 8; k++) d = d & 0x8000 ? (uint16_t)(d << 1 ^ 0x1021) : (uint16_t)(d << 1);
        crc16_tab[i] = d;
        uint32_t l = 0;
        for (int k = 0; k < 4; k++)
            l |= ((uint32_t)(i >> (4 + k) & 1) << 1 | (uint32_t)(i >> k & 1)) << (8 * k + 6);
        lane[i] = l;
    }
}

static uint8_t crc7(const uint8_t *p, int n) {
    uint8_t c = 0;
    while (n--) {
        uint8_t x = *p++;
        for (int b = 7; b >= 0; b--) {
            bool in = ((x >> b) ^ (c >> 6)) & 1;
            c = (uint8_t)((c << 1) & 0x7F);
            if (in) c ^= 0x09;
        }
    }
    return c;
}

static bool hw_setup(void) {
    tables();
    gpio_set_function(CLK_PIN, GPIO_FUNC_PWM);
    gpio_set_drive_strength(CLK_PIN, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_slew_rate(CLK_PIN, GPIO_SLEW_RATE_FAST);
    for (uint p = CMD_PIN; p < DAT0_PIN + 4; p++) {
        gpio_pull_up(p);
        pio_gpio_init(pio, p);
    }
    int oc = pio_add_program(pio, &sdio_cmd_program), orx = pio_add_program(pio_rx, &sdio_rx_program);
    if (oc < 0 || orx < 0) return false;    // no room: the SDK returns -1 rather than panicking
    off_cmd = (uint)oc;
    off_rx = (uint)orx;
    sm_cmd = (uint)pio_claim_unused_sm(pio, true);
    pio_sm_claim(pio_rx, SM_RX);

    pio_sm_config c = sdio_cmd_program_get_default_config(off_cmd);
    sm_config_set_out_pins(&c, CMD_PIN, 1);
    sm_config_set_set_pins(&c, CMD_PIN, 1);
    sm_config_set_in_pins(&c, CMD_PIN);
    sm_config_set_jmp_pin(&c, CMD_PIN);
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_in_shift(&c, false, true, 32);
    pio_sm_set_pins_with_mask(pio, sm_cmd, 1u << CMD_PIN, 1u << CMD_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm_cmd, CMD_PIN, 1, false);
    pio_sm_init(pio, sm_cmd, off_cmd + sdio_cmd_offset_start, &c);

    c = sdio_rx_program_get_default_config(off_rx);
    sm_config_set_in_pins(&c, DAT0_PIN);
    sm_config_set_jmp_pin(&c, DAT0_PIN);
    sm_config_set_in_shift(&c, false, true, 32);
    pio_sm_set_consecutive_pindirs(pio_rx, SM_RX, DAT0_PIN, 4, false);
    pio_sm_init(pio_rx, SM_RX, off_rx + sdio_rx_offset_start, &c);
    pio_sm_set_enabled(pio, sm_cmd, true);
    pio_sm_set_enabled(pio_rx, SM_RX, true);
    dma = (uint)dma_claim_unused_channel(true);
    hw_up = true;
    return true;
}

// out: the 32 content bits of a 48-bit response; unused for R_NONE and R_136.
static bool cmd(int idx, uint32_t arg, int kind, uint32_t *out) {
    sd_stats.stage = (uint32_t)idx;
    uint8_t b[5] = {(uint8_t)(0x40 | idx), (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg};
    uint64_t frame = (uint64_t)b[0] << 40 | (uint64_t)arg << 8 | (uint64_t)crc7(b, 5) << 1 | 1;
    int bits = kind == R_NONE ? 0 : kind == R_136 ? 136 : 48, words = bits ? (bits - 1 + 31) / 32 : 0;
    pio_sm_put(pio, sm_cmd, (uint32_t)(frame >> 16));
    pio_sm_put(pio, sm_cmd, (uint32_t)(frame & 0xFFFF) << 16 | (uint32_t)(bits ? bits - 2 : 0));
    uint32_t r[5], t0 = time_us_32();
    for (int i = 0; i < words; i++) {
        while (pio_sm_is_rx_fifo_empty(pio, sm_cmd))
            if (time_us_32() - t0 > 5000) {
                cmd_reset();
                sd_stats.cmd_errors++;
                sd_stats.fail = (uint32_t)idx | 0x100;
                return false;
            }
        r[i] = pio_sm_get(pio, sm_cmd);
    }
    busy_wait_us(gap_us);
    if (kind == R_NONE) {
        while (!pio_sm_is_tx_fifo_empty(pio, sm_cmd)) tight_loop_contents();
        busy_wait_us(1000);
        return true;
    }
    if (kind == R_136) return true;
    uint64_t v = (uint64_t)r[0] << 15 | (r[1] & 0x7FFF);    // response bits 46..0
    uint32_t content = (uint32_t)(v >> 8);
    if (out) *out = content;
    sd_stats.resp = content;
    if (kind == R_48_NOCRC) return true;
    uint8_t rb[5] = {(uint8_t)(v >> 40 & 0x3F), (uint8_t)(content >> 24), (uint8_t)(content >> 16),
                     (uint8_t)(content >> 8), (uint8_t)content};
    if ((int)(v >> 40 & 0x3F) != idx || crc7(rb, 5) != (v >> 1 & 0x7F)) {
        sd_stats.cmd_errors++;
        sd_stats.fail = (uint32_t)idx | 0x200;
        return false;
    }
    return true;
}

static bool wait_ready(uint32_t timeout_us) {
    busy_wait_us(2);
    uint32_t t0 = time_us_32();
    while (!gpio_get(DAT0_PIN))
        if (time_us_32() - t0 > timeout_us) return false;
    return true;
}

static bool init_once(uint32_t fast_div) {
    rd_state = RD_IDLE;
    dma_channel_abort(dma);
    clock_set(clock_get_hz(clk_sys) / 400000 + 1);
    cmd_reset();
    rx_reset();
    busy_wait_ms(2);                    // well over the 74 clocks a card wants before CMD0

    uint32_t r, ocr = 0;
    cmd(0, 0, R_NONE, NULL);
    bool v2 = cmd(8, 0x1AA, R_48, &r) && (r & 0xFFF) == 0x1AA;
    uint32_t t0 = time_us_32();
    for (;;) {
        if (!cmd(55, 0, R_48, &r) || !cmd(41, v2 ? 0x40FF8000 : 0x00FF8000, R_48_NOCRC, &ocr)) return false;
        if (ocr & 0x80000000) break;
        if (time_us_32() - t0 > 1000000) return false;
        busy_wait_ms(5);
    }
    ccs = v2 && (ocr & 0x40000000);
    if (!cmd(2, 0, R_136, NULL) || !cmd(3, 0, R_48, &r)) return false;
    rca = r & 0xFFFF0000;
    if (!cmd(7, rca, R_48, &r) || !wait_ready(100000)) return false;
    if (!cmd(55, rca, R_48, &r) || !cmd(6, 2, R_48, &r)) return false;    // 4-bit bus
    if (!cmd(16, 512, R_48, &r)) return false;
    clock_set(fast_div < 13 ? 13 : fast_div);
    sd_stats.stage = 100;
    return true;
}

// A card already up now and then misses a command of the bring-up; the whole sequence goes again.
bool sd_init(uint32_t fast_div) {
    if (!hw_up && !hw_setup()) return false;
    for (int i = 0; i < 3; i++)
        if (init_once(fast_div)) return true;
    return false;
}

bool sd_read_start(uint32_t sector, uint32_t blocks, uint8_t *raw) {
    if (rd_state != RD_IDLE || blocks == 0) return false;
    pio_sm_put(pio_rx, SM_RX, blocks - 1);
    pio_sm_put(pio_rx, SM_RX, NIBBLES - 1);
    dma_channel_config c = dma_channel_get_default_config(dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_bswap(&c, true);     // first nibble to the lowest address
    channel_config_set_dreq(&c, pio_get_dreq(pio_rx, SM_RX, false));
    dma_channel_configure(dma, &c, raw, &pio_rx->rxf[SM_RX], blocks * SD_RAW_BLOCK / 4, true);
    uint32_t r;
    if (!cmd(18, ccs ? sector : sector * 512, R_48, &r) || (r & 0xFFF80000)) {
        dma_channel_abort(dma);
        rx_reset();
        return false;
    }
    rd_raw = raw;
    rd_blocks = blocks;
    rd_t0 = time_us_32();
    rd_state = RD_DATA;
    return true;
}

// Byte k: DAT line k's bits of these four bytes, in the order sent.
static inline uint32_t lanes(const uint8_t *p) {
    return lane[p[0]] | lane[p[1]] >> 2 | lane[p[2]] >> 4 | lane[p[3]] >> 6;
}

static bool block_ok(const uint8_t *b) {
    uint32_t c[4] = {0, 0, 0, 0};
    for (int i = 0; i < 512; i += 4) {
        uint32_t x = lanes(b + i);
        for (int k = 0; k < 4; k++)
            c[k] = (c[k] << 8 ^ crc16_tab[(c[k] >> 8 ^ x >> 8 * k) & 0xFF]) & 0xFFFF;
    }
    uint32_t hi = lanes(b + 512), lo = lanes(b + 516);
    for (int k = 0; k < 4; k++)
        if (c[k] != ((hi >> 8 * k & 0xFF) << 8 | (lo >> 8 * k & 0xFF))) return false;
    return true;
}

static void stop(void) {
    uint32_t r;
    cmd(12, 0, R_48, &r);
}

int sd_read_poll(void) {
    switch (rd_state) {
    case RD_DATA:
        if (dma_channel_is_busy(dma)) {
            if (time_us_32() - rd_t0 < 250000) return 0;
            dma_channel_abort(dma);
            rx_reset();
            stop();
            wait_ready(250000);
            sd_stats.timeouts++;
            rd_state = RD_IDLE;
            return -1;
        }
        stop();
        rd_t0 = time_us_32();
        rd_state = RD_BUSY;
        return 0;
    case RD_BUSY:
        if (time_us_32() - rd_t0 < 2) return 0;
        if (!gpio_get(DAT0_PIN)) {
            if (time_us_32() - rd_t0 < 250000) return 0;
            sd_stats.timeouts++;
            rd_state = RD_IDLE;
            return -1;
        }
        rd_k = 0;
        rd_bad = false;
        rd_state = RD_CHECK;
        return 0;
    case RD_CHECK:
        for (int n = 0; n < CHECK_PER_POLL && rd_k < rd_blocks; n++, rd_k++) {
            const uint8_t *b = rd_raw + rd_k * SD_RAW_BLOCK;
            if (!block_ok(b)) rd_bad = true;
            memmove(rd_raw + rd_k * 512, b, 512);
        }
        if (rd_k < rd_blocks) return 0;
        rd_state = RD_IDLE;
        sd_stats.reads++;
        if (rd_bad) {
            sd_stats.crc_errors++;
            return -1;
        }
        return 1;
    default:
        return -1;
    }
}

uint32_t sd_status_test(uint32_t n) {
    uint32_t bad = 0, r;
    for (uint32_t i = 0; i < n; i++)
        if (!cmd(13, rca, R_48, &r)) bad++;
    return bad;
}

bool sd_read(uint32_t sector, uint32_t blocks, uint8_t *raw) {
    if (!sd_read_start(sector, blocks, raw)) return false;
    int r;
    while ((r = sd_read_poll()) == 0) tight_loop_contents();
    return r > 0;
}
