// VHS console BIOS: the tape screen (tape_ui.c) until the DE1 has read the game image off the data leader,
// then the image over the link and the game, as GameConsoleV2's src/main.c launch_game does.
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"

#include "video/video.h"
#include "video/render3d.h"
#include "video/psram.h"
#include "audio/audio.h"
#include "engine/engine.h"
#include "input/gamepad.h"
#include "cart/cart.h"
#include "cart/cart_format.h"
#include "fb_draw.h"
#include "link.h"
#include "tape_ui.h"
#include "vhs_api.h"
#include "video_link.h"

#define TAPE_OFFSET     0x00200000u     // the whole tape image, for cart_read
#define READ_CHUNK      1024u
#define SLOT_MAX        17u             // the most data bytes a tape slot holds
#define IMG_READ_MS     200             // per READ_CHUNK slots, then a retry
#define IMG_TRIES       4
#define PRESENT_MS      100
#define ECHO_MS         20
#define WATCHDOG_MS     3000            // the BIOS feeds it every frame; off while a game runs

enum { STAGE_INIT, STAGE_TAPE, STAGE_LOADING, STAGE_RUNNING, STAGE_TEST };

// Live control and counters over SWD (arm-none-eabi-nm for the address). ctl.mode link tests: 1 ECHO soak
// + moving pattern + tone, 2 ECHO only, 3 random frames only; drive_ma/fast: DN pads.
volatile struct {
    uint32_t magic, mode, stage, status_seq;
    uint32_t echo_ok, echo_bad, echo_timeout, presents, present_timeouts;
    uint32_t load_slots, load_retries, load_fails;
    uint32_t up_packets, up_crc_errs, dn_packets, aud_frames;
    uint32_t de1_dn_crc_errs, de1_dn_len_errs, de1_aud_under;
    uint32_t drive_ma, fast;
    uint32_t audio_off, echo_us_last, echo_us_max;
    uint32_t games, dn_drops;               // games launched, DN packets dropped with the DE1 away
    uint32_t up_resyncs, up_bytes;          // UP receiver restarts; bytes received
} ctl = {.magic = 0x55504356u, .drive_ma = 2};   // "VCPU"; 4 mA and up corrupts DN (2026-09-29)

void audio_link_attach(bool on);        // audio_link.c

static uint32_t core1_stack[2048];
static uint8_t chunk[READ_CHUNK * SLOT_MAX];
static cart_header_t hdr;
static const uint8_t *image;            // the whole game image, copied off the tape into PSRAM
static uint32_t image_bytes;

static uint8_t *psram(uint32_t off) {
    return (uint8_t *)(uintptr_t)(PSRAM_XIP_BASE + off);
}

static void update_ctl(void) {
    link_stats_t ls;
    vhs_status_t s;
    uint32_t seq;
    link_get_stats(&ls);
    ctl.up_packets = ls.up_packets;
    ctl.up_crc_errs = ls.up_crc_errs;
    ctl.dn_packets = ls.dn_packets;
    ctl.aud_frames = ls.aud_frames;
    ctl.dn_drops = ls.dn_drops;
    ctl.up_resyncs = ls.up_resyncs;
    ctl.up_bytes = ls.up_bytes;
    if (link_status(&s, &seq)) {
        ctl.status_seq = seq;
        ctl.de1_dn_crc_errs = s.dn_crc_errs;
        ctl.de1_dn_len_errs = s.dn_len_errs;
        ctl.de1_aud_under = s.cpu_aud_under;
    }
}

// API slots that differ from the GCV2 BIOS.

static void api_vsync(void) {
    video_wait_vsync();
    gamepad_poll();
}

static void api_video_init(int mode) {
    video_init((video_mode_t)mode);
}

static void api_asset_read(uint32_t offset, void *buf, uint32_t len) {
    uint8_t *d = buf;
    for (uint32_t i = 0; i < len; i++)
        d[i] = offset + i < hdr.asset_size ? psram(CART_ASSET_PSRAM_OFFSET)[offset + i] : 0x00;
}

static const void *api_asset_ptr(uint32_t offset) {
    return psram(CART_ASSET_PSRAM_OFFSET + offset);
}

static void api_cart_read(uint32_t offset, void *buf, uint32_t len) {
    uint8_t *d = buf;
    uint32_t base = cart_asset_base(&hdr);
    for (uint32_t i = 0; i < len; i++) {
        uint32_t at = base + offset + i;
        d[i] = at < image_bytes ? image[at] : 0xFF;
    }
}

static void api_coproc_submit(uint32_t cmd, const void *in, uint32_t in_len) {
    (void)cmd, (void)in, (void)in_len;
}

static int api_coproc_fetch(void *out, uint32_t out_len) {
    (void)out, (void)out_len;
    return -1;
}

static const void *api_sample_bank(void) {
    return NULL;
}

static bool api_status(vhs_status_t *out) {
    return link_status(out, NULL);
}

static const vhs_api_t api = {
    .gcv2 = {
        .video_init = api_video_init,
        .framebuffer = video_get_framebuffer,
        .vsync = api_vsync,
        .tilemap_render = tilemap_render,
        .sprite_render = sprite_layer_render,
        .art_color = art_color,
        .art_to_pixels = art_to_pixels,
        .render3d_init = render3d_init,
        .render3d_set_projection = render3d_set_projection,
        .render3d_set_light = render3d_set_light,
        .render3d_set_dualcore = render3d_set_dualcore,
        .render3d_begin = render3d_begin,
        .render3d_draw = render3d_draw,
        .render3d_end = render3d_end,
        .r3d_model_identity = r3d_model_identity,
        .r3d_model_euler = r3d_model_euler,
        .r3d_model_mul = r3d_model_mul,
        .r3d_sphere = r3d_sphere,
        .gamepad_read = gamepad_read,
        .audio_play_note = audio_play_note,
        .audio_noise = audio_noise,
        .audio_stop = audio_stop,
        .asset_read = api_asset_read,
        .asset_ptr = api_asset_ptr,
        .coproc_submit = api_coproc_submit,
        .coproc_fetch = api_coproc_fetch,
        .printf = printf,
        .time_us = time_us_64,
        .sleep_ms = sleep_ms,
        .audio_scope_read = audio_scope_read,
        .gamepad_read_port = gamepad_read_port,
        .cart_read = api_cart_read,
        .audio_set_fill = audio_set_fill_callback,
        .audio_stop_all = audio_stop_all,
        .audio_sample_bank = api_sample_bank,
        .audio_out_latency = audio_out_latency_frames,
    },
    .magic = VHS_API_MAGIC,
    .status = api_status,
    .reg_write = link_reg_write,
    .ir_send = link_ir_send,
    .palette = link_palette,
    .video_window = video_window,
    .video_fullscreen = video_fullscreen,
    .tape_tap = link_set_tap,
};

// Image bytes [at, at + len) into dst, via whole tape slots.
static bool read_image(uint32_t at, uint32_t len, uint8_t *dst, unsigned d) {
    uint32_t first = at / d, last = (at + len - 1) / d;
    for (uint32_t s = first; s <= last; s += READ_CHUNK) {
        uint32_t n = last - s + 1 < READ_CHUNK ? last - s + 1 : READ_CHUNK;
        int tries = 0;
        watchdog_update();
        while (!link_img_read((uint16_t)s, (uint16_t)n, chunk, d, IMG_READ_MS)) {
            ctl.load_retries++;
            if (++tries == IMG_TRIES)
                return false;
        }
        ctl.load_slots += n;
        for (uint32_t k = 0; k < n * d; k++) {
            uint32_t b = (s * d) + k;
            if (b >= at && b < at + len)
                dst[b - at] = chunk[k];
        }
    }
    return true;
}

// Validate the image at `image` and copy its code to GAMERAM and its assets to PSRAM. NULL, or why not.
static const char *install_image(void) {
    const char *err = NULL;
    memcpy(&hdr, image, sizeof hdr);
    if (!cart_header_valid(&hdr, image_bytes, CART_GAME_RAM_SIZE, &err)) {
        printf("load: bad header: %s\n", err ? err : "?");
        return "THE GAME HEADER IS NOT VALID";
    }
    printf("load: \"%.32s\" code %lu assets %lu\n", hdr.title, (unsigned long)hdr.code_size,
           (unsigned long)hdr.asset_size);
    const uint8_t *code = image + CART_CODE_OFFSET;
    if ((hdr.flags & CART_FLAG_CODE_CRC32) && cart_crc32(code, hdr.code_size) != hdr.code_crc32) {
        printf("load: code CRC mismatch\n");
        return "THE GAME CODE IS CORRUPT";
    }
    if (hdr.entry_offset & 1u) {        // cart_header_valid has the rest; Thumb code starts on a halfword
        printf("load: odd entry %lu\n", (unsigned long)hdr.entry_offset);
        return "THE GAME HAS NO VALID ENTRY";
    }
    if (hdr.asset_size > TAPE_OFFSET - CART_ASSET_PSRAM_OFFSET) {   // the assets would run into the image they come from
        printf("load: assets %lu bytes\n", (unsigned long)hdr.asset_size);
        return "THE GAME IS TOO BIG FOR THIS CONSOLE";
    }
    memset((void *)(uintptr_t)CART_GAME_RAM_BASE, 0, CART_GAME_RAM_SIZE);
    memcpy((void *)(uintptr_t)CART_GAME_RAM_BASE, code, hdr.code_size);
    if (hdr.asset_size)
        memcpy(psram(CART_ASSET_PSRAM_OFFSET), image + cart_asset_base(&hdr), hdr.asset_size);
    return NULL;
}

static const char *load_tape(const vhs_status_t *s) {
    unsigned d = s->dt_d;
    uint32_t bytes = (uint32_t)s->dt_n * d;
    if (d == 0 || d > SLOT_MAX || s->dt_n == 0) {
        printf("load: %u-byte slots, %u slots\n", d, s->dt_n);
        return "THE GAME TAPE FORMAT IS NOT SUPPORTED";
    }
    if (psram_get_size() < TAPE_OFFSET + bytes) {
        printf("load: %u-byte slots, %lu bytes, PSRAM %u\n", d, (unsigned long)bytes, (unsigned)psram_get_size());
        return "THE GAME IS TOO BIG FOR THIS CONSOLE";
    }
    if (!read_image(0, bytes, psram(TAPE_OFFSET), d)) {
        printf("load: image read failed\n");
        return "THE VIDEO UNIT DID NOT SEND THE GAME";
    }
    image = psram(TAPE_OFFSET);
    image_bytes = bytes;
    return install_image();
}

// The DE1 as tape_ui_enter left it, but on stream 0 full screen with the reset palette; no watchdog in the game.
// The caller's tape_ui_enter takes the DE1 back after.
static void launch_game(void) {
    printf("launch: \"%.32s\" -> 0x%08lX\n", hdr.title, (unsigned long)(CART_GAME_RAM_BASE + hdr.entry_offset));
    video_fullscreen();
    video_palette_reset();
    video_overlay_default();
    link_reg_write(REG_VIDEO, 0);
    link_reg_write(REG_OWN, 1);
    ctl.stage = STAGE_RUNNING;
    ctl.games++;
    game_entry_t entry = (game_entry_t)((CART_GAME_RAM_BASE + hdr.entry_offset) | 1u);
    watchdog_disable();
    entry(&api.gcv2);
    watchdog_enable(WATCHDOG_MS, true);
    link_set_tap(NULL);
    audio_set_fill_callback(NULL);
    audio_stop_all();
    printf("launch: game returned\n");
}

static uint32_t rng = 0x12345678u;
static uint32_t xorshift(void) {
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng;
}

static void test_step(unsigned mode) {
    static uint32_t frame;
    uint8_t *fb = video_get_framebuffer();
    if (mode == 1) {
        for (int y = 0; y < VIDEO_HEIGHT; y++)
            for (int x = 0; x < VIDEO_WIDTH; x++)
                fb[y * VIDEO_WIDTH + x] = y < 40 ? (uint8_t)(x * 8 / VIDEO_WIDTH * 32 + 31) : 0;
        fill_rect(fb, (int)(frame * 2 % (VIDEO_WIDTH - 16)), 60, 16, 120, RGB332(255, 255, 0));
        fill_rect(fb, 0, 200, VIDEO_WIDTH, 1, RGB332(255, 255, 255));
    } else if (mode == 3) {
        for (int i = 0; i < VIDEO_WIDTH * VIDEO_HEIGHT; i++)
            fb[i] = (uint8_t)xorshift();
    }
    frame++;
    if (mode != 2) {
        if (link_present(fb, PRESENT_MS))
            ctl.presents++;
        else
            ctl.present_timeouts++;
    }
    if (mode == 3)
        return;

    uint8_t buf[LINK_ECHO_MAX];
    for (int k = 0; k < 16; k++) {
        for (unsigned i = 0; i < sizeof buf; i++)
            buf[i] = (uint8_t)xorshift();
        uint64_t t0 = time_us_64();
        int r = link_echo(buf, sizeof buf, ECHO_MS);
        uint32_t us = (uint32_t)(time_us_64() - t0);
        ctl.echo_us_last = us;
        if (us > ctl.echo_us_max)
            ctl.echo_us_max = us;
        if (r > 0)
            ctl.echo_ok++;
        else if (r < 0)
            ctl.echo_bad++;
        else
            ctl.echo_timeout++;
    }
}

// Every frame, in the tape screen too: the SWD counters and controls, and whether a link test was asked for.
static bool test_mode(void) {
    static uint32_t drive, fast;
    update_ctl();
    audio_link_attach(!ctl.audio_off);
    if (ctl.drive_ma != drive || ctl.fast != fast) {
        drive = ctl.drive_ma;
        fast = ctl.fast;
        link_set_drive(drive, fast);
    }
    return ctl.mode >= 1 && ctl.mode <= 3;
}

// KEY[0] on the DE1 resets the whole console, a game included; on core 1.
static void key_reset(void) {
    watchdog_reboot(0, 0, 0);
}

int main(void) {
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(3);
    set_sys_clock_khz(315000, true);
    stdio_init_all();
    printf("\n=== VHS console BIOS ===\n");
    size_t ps = psram_init(PSRAM_CS_PIN);
    printf("PSRAM: %u bytes\n", (unsigned)ps);
    for (uint p = 40; p <= 41; p++)
        gpio_pull_up(p);                // controller data: the bare PGA2350 has no external pull-up
    gamepad_init();
    link_init();
    link_set_key_reset(key_reset);
    multicore_launch_core1_with_stack(link_core1_main, core1_stack, sizeof core1_stack);
    audio_init();
    if (watchdog_caused_reboot())
        printf("BIOS: restarted by the watchdog\n");
    watchdog_enable(WATCHDOG_MS, true);
    tape_ui_enter();

    const char *err = NULL, *detail = NULL;
    uint32_t last_mode = 0;
    for (;;) {
        watchdog_update();
        if (test_mode()) {
            if (last_mode != 1 && ctl.mode == 1)
                audio_play_test_tone(1000);
            last_mode = ctl.mode;
            ctl.stage = STAGE_TEST;
            test_step(ctl.mode);
            continue;
        }
        if (last_mode) {
            audio_set_fill_callback(NULL);
            tape_ui_enter();
        }
        last_mode = 0;

        ctl.stage = STAGE_TAPE;
        if (!tape_ui_run(err, detail, test_mode))
            continue;
        err = detail = NULL;
        ctl.stage = STAGE_LOADING;
        tape_ui_show("LOADING GAME", "");
        vhs_status_t s;
        link_status(&s, NULL);
        const char *why = load_tape(&s);
        if (why) {
            ctl.load_fails++;
            err = "GAME LOAD FAILED";
            detail = why;
            continue;
        }
        launch_game();
        tape_ui_enter();
    }
}
