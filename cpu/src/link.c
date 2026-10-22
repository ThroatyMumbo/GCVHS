// DE1 link driver: core 1 parses UP off link_phy.c, feeds audio, replays registers after a DE1 reset and runs
// core 0's requests.
#include "link.h"

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "link_phy.h"

#define AUD_BLOCK 256u
#define IMG_MAX   4096u

#define AUD_CREDIT_MAX 1536u            // the most the DE1 can have unmet: it grants 512 while queued + owed < 1024

#define UP_STALL_US   300000            // no STATUS this long: restart the UP receiver

static volatile uint32_t st_time, st_ms;    // time_us_32 and ms since boot of the last STATUS
static const uint8_t *volatile present_fb;  // the caller's frame; it waits until the flip

static link_stats_t stats;

// STATUS under a sequence lock: odd while core 1 writes it.
static volatile uint32_t st_seq;
static vhs_status_t st;

// Core 0 -> core 1 requests, one at a time.
enum { RQ_NONE, RQ_REGS, RQ_PAL, RQ_PRESENT, RQ_IMG, RQ_ECHO };
static volatile int rq_kind;
static uint8_t rq_buf[4 + 256 * 4];
static unsigned rq_len;

static volatile uint16_t img_first, img_count, img_got;
static uint8_t *volatile img_dst;
static volatile unsigned img_d;
static uint32_t img_seen[IMG_MAX / 32];

static uint8_t echo_want[LINK_ECHO_MAX];
static volatile unsigned echo_len;
static volatile int echo_result;            // 0 waiting, 1 match, -1 mismatch

static volatile link_audio_fill_fn aud_fill;
static uint32_t aud_credit;
static uint16_t aud_granted;            // AUD_REQ's running total, mod 2^16
static volatile link_tap_fn tap_fn;
static volatile link_key_reset_fn key_reset_fn;

// Every register and palette entry written, resent when STATUS shows the DE1 was reset or reloaded (its frame
// count went back, or OWN is gone) or lost a packet. IR_LO/IR_HI are not kept: writing IR_HI sends a frame.
#define SHADOW_REGS (REG_WIN_PAN + 1)
#define SHADOW_KEEP (((1u << SHADOW_REGS) - 1) & ~(1u << REG_IR_LO | 1u << REG_IR_HI))
static volatile uint32_t shadow[SHADOW_REGS], shadow_set;
static uint8_t pal_shadow[256][4];
static volatile uint32_t pal_set[256 / 32];
static volatile bool replay;
static uint32_t last_frame, dn_errs;
static uint8_t key_resets;              // STATUS key_resets: a new nonzero count calls key_reset_fn
static bool key_seen;

static void up_packet(uint8_t type, const uint8_t *p, unsigned n) {
    stats.up_packets++;
    switch (type) {
    case UP_STATUS:
        if (n != sizeof st)
            break;
        st_seq++;
        __dmb();
        memcpy(&st, p, sizeof st);
        __dmb();
        st_seq++;
        st_time = time_us_32();
        st_ms = to_ms_since_boot(get_absolute_time());
        if (st.key_resets && st.key_resets != key_resets && key_seen) {
            link_key_reset_fn fn = key_reset_fn;
            if (fn)
                fn();
        }
        key_resets = st.key_resets;
        key_seen = true;
        // Also when the DE1 lost OWN (writes sent while KEY[0] held it in reset vanish) or dropped a packet.
        uint32_t errs = (uint32_t)st.dn_crc_errs + st.dn_len_errs;
        if (st.frame < last_frame || phy_dn_lost() || errs != dn_errs ||
            (shadow[REG_OWN] & 1 && !(st.flags0 & ST0_OWN)))
            replay = true;
        phy_dn_found();
        dn_errs = errs;
        last_frame = st.frame;
        break;
    case UP_AUD_REQ:
        if (n == 2) {
            uint16_t total = (uint16_t)(p[0] | p[1] << 8);
            aud_credit += (uint16_t)(total - aud_granted);
            aud_granted = total;
            if (aud_credit > AUD_CREDIT_MAX)    // also absorbs the jump when a reset DE1 restarts its total
                aud_credit = AUD_CREDIT_MAX;
        }
        break;
    case UP_IMG_DATA: {
        uint8_t *dst = img_dst;
        if (n < 2 || !dst)
            break;
        unsigned slot = p[0] | p[1] << 8, k = slot - img_first;
        if (slot < img_first || k >= img_count || n - 2 < img_d || (img_seen[k / 32] >> (k % 32) & 1))
            break;
        memcpy(dst + k * img_d, p + 2, img_d);
        img_seen[k / 32] |= 1u << (k % 32);
        img_got++;
        break;
    }
    case UP_TAPE_AUD: {
        link_tap_fn fn = tap_fn;
        if (fn && n == sizeof(vhs_tap_t))
            fn((const vhs_tap_t *)p);
        break;
    }
    case UP_ECHO:
        echo_result = n == echo_len && memcmp(p, echo_want, n) == 0 ? 1 : -1;
        break;
    }
}

// Longest payload of each UP type, or -1: a false sync then costs one header, not a long bogus payload.
static int up_max(uint8_t type) {
    switch (type) {
    case UP_STATUS: return sizeof(vhs_status_t);
    case UP_AUD_REQ: return 2;
    case UP_IMG_DATA: return 2 + 32;
    case UP_TAPE_AUD: return sizeof(vhs_tap_t);
    case UP_ECHO: return LINK_ECHO_MAX;
    default: return -1;
    }
}

static enum { P_HUNT, P_TYPE, P_L0, P_L1, P_PAY, P_C0, P_C1 } ps;

static void up_restart(void) {
    phy_up_restart();
    ps = P_HUNT;
}

// Parse whatever UP bytes have landed.
static void up_poll(void) {
    static uint8_t type, buf[LINK_ECHO_MAX + 1];
    static unsigned len, i;
    static uint16_t crc;
    const uint8_t *p;
    unsigned n;
    while ((n = phy_up_take(&p)) != 0)
        for (const uint8_t *end = p + n; p < end; p++) {
            uint8_t b = *p;
            switch (ps) {
            case P_HUNT:
                if (b == LINK_SYNC) {
                    ps = P_TYPE;
                    crc = LINK_CRC_INIT;
                }
                break;
            case P_TYPE:
                type = b;
                crc = phy_crc16(crc, &b, 1);
                ps = P_L0;
                break;
            case P_L0:
                len = b;
                crc = phy_crc16(crc, &b, 1);
                ps = P_L1;
                break;
            case P_L1:
                len |= b << 8;
                crc = phy_crc16(crc, &b, 1);
                i = 0;
                ps = (int)len > up_max(type) ? P_HUNT : len ? P_PAY : P_C0;
                break;
            case P_PAY:
                buf[i++] = b;
                crc = phy_crc16(crc, &b, 1);
                if (i == len)
                    ps = P_C0;
                break;
            case P_C0:
                buf[len] = b;       // spare byte past the payload
                ps = P_C1;
                break;
            case P_C1:
                if ((buf[len] | b << 8) == crc)
                    up_packet(type, buf, len);
                else
                    stats.up_crc_errs++;
                ps = P_HUNT;
                break;
            }
        }
}

static void audio_service(void) {
    static int16_t blk[AUD_BLOCK * 2];
    link_audio_fill_fn fill = aud_fill;
    if (!fill)
        aud_credit = 0;                 // nothing banked while detached, so a reattach can't flood the link
    if (!fill || aud_credit < AUD_BLOCK)
        return;
    fill(blk, AUD_BLOCK);
    uint16_t hdr = AUD_BLOCK;
    phy_send(DN_AUD, &hdr, 2, blk, sizeof blk);
    aud_credit -= AUD_BLOCK;
    stats.aud_frames += AUD_BLOCK;
}

static void replay_regs(void) {
    uint8_t buf[LINK_REG_MAX * 5];
    unsigned n = 0;
    replay = false;
    for (uint8_t a = 0; a < SHADOW_REGS; a++) {
        if (!(shadow_set >> a & 1))
            continue;
        uint32_t v = shadow[a];
        buf[5 * n] = a;
        memcpy(buf + 5 * n + 1, &v, 4);
        if (++n == LINK_REG_MAX) {
            phy_send(DN_REG_WR, buf, 5 * n, NULL, 0);
            n = 0;
        }
    }
    if (n)
        phy_send(DN_REG_WR, buf, 5 * n, NULL, 0);
    for (unsigned a = 0; a < 256;) {
        unsigned b = a;
        while (b < 256 && pal_set[b / 32] >> (b % 32) & 1)
            b++;
        if (b == a) {
            a++;
            continue;
        }
        uint8_t hdr[2] = {(uint8_t)a, (uint8_t)(b - a)};
        phy_send(DN_PAL_WR, hdr, 2, pal_shadow[a], (b - a) * 4);
        a = b;
    }
}

static void service(void) {
    static uint32_t up_check;
    uint32_t now = time_us_32();
    if (now - st_time > UP_STALL_US && now - up_check > UP_STALL_US) {
        up_check = now;
        up_restart();
    }
    up_poll();
    if (replay)
        replay_regs();
    audio_service();
}

static void run_request(void) {
    __dmb();                            // rq_kind was seen: now rq_buf, rq_len and the rest are core 0's
    switch (rq_kind) {
    case RQ_REGS:
        phy_send(DN_REG_WR, rq_buf, rq_len, NULL, 0);
        break;
    case RQ_PAL:
        phy_send(DN_PAL_WR, rq_buf, rq_len, NULL, 0);
        break;
    case RQ_PRESENT:
        for (uint32_t off = 0; off < LINK_FB_W * LINK_FB_H; off += LINK_FB_CHUNK) {
            uint32_t n = LINK_FB_W * LINK_FB_H - off < LINK_FB_CHUNK ? LINK_FB_W * LINK_FB_H - off : LINK_FB_CHUNK;
            phy_send(DN_FB_WR, &off, 4, present_fb + off, n);
            service();
        }
        phy_send(DN_FLIP, NULL, 0, NULL, 0);
        stats.presents++;
        break;
    case RQ_IMG: {
        uint16_t a[2] = {img_first, img_count};
        phy_send(DN_IMG_RD, a, 4, NULL, 0);
        break;
    }
    case RQ_ECHO:
        phy_send(DN_ECHO, echo_want, echo_len, NULL, 0);
        break;
    }
    __dmb();
    rq_kind = RQ_NONE;
}

void link_core1_main(void) {
    for (;;) {
        service();
        if (rq_kind != RQ_NONE)
            run_request();
    }
}

void link_init(void) {
    phy_init();
}

static void post(int kind) {
    __dmb();
    rq_kind = kind;
    while (rq_kind != RQ_NONE)
        tight_loop_contents();
}

bool link_status(vhs_status_t *out, uint32_t *seq) {
    uint32_t s;
    do {
        s = st_seq;
        __dmb();
        memcpy(out, &st, sizeof st);
        __dmb();
    } while ((s & 1) || s != st_seq);
    if (seq)
        *seq = s / 2;
    return s != 0;
}

uint32_t link_status_age_ms(void) {
    if (st_seq == 0)
        return UINT32_MAX;
    return to_ms_since_boot(get_absolute_time()) - st_ms;
}

bool link_wait_status(vhs_status_t *out, uint32_t timeout_ms) {
    uint32_t s0 = st_seq;
    absolute_time_t end = make_timeout_time_ms(timeout_ms);
    while (st_seq == s0 || (st_seq & 1))
        if (time_reached(end))
            return false;
    return link_status(out, NULL);
}

void link_regs_write(const link_reg_t *regs, unsigned n) {
    if (n == 0 || n > LINK_REG_MAX)
        return;
    while (rq_kind != RQ_NONE)
        tight_loop_contents();
    for (unsigned k = 0; k < n; k++) {
        uint8_t addr = regs[k].addr;
        rq_buf[5 * k] = addr;
        memcpy(rq_buf + 5 * k + 1, &regs[k].val, 4);
        if (addr < SHADOW_REGS && (SHADOW_KEEP >> addr & 1)) {
            shadow[addr] = regs[k].val;
            shadow_set |= 1u << addr;
        }
    }
    rq_len = 5 * n;
    post(RQ_REGS);
}

void link_reg_write(uint8_t addr, uint32_t val) {
    link_regs_write(&(link_reg_t){addr, val}, 1);
}

void link_ir_send(uint64_t payload) {
    link_regs_write((const link_reg_t[]){{REG_IR_LO, (uint32_t)payload},
                                         {REG_IR_HI, (uint32_t)(payload >> 32) & 0xFFFF}}, 2);
}

void link_palette(uint8_t first, unsigned count, const uint8_t (*rgbt)[4]) {
    if (count == 0 || count > 256 - first)
        return;
    memcpy(pal_shadow[first], rgbt, count * 4);
    for (unsigned k = first; k < first + count; k++)
        pal_set[k / 32] |= 1u << (k % 32);
    rq_buf[0] = first;
    rq_buf[1] = (uint8_t)count;
    memcpy(rq_buf + 2, rgbt, count * 4);
    rq_len = 2 + count * 4;
    post(RQ_PAL);
}

bool link_present(const uint8_t *fb, uint32_t timeout_ms) {
    vhs_status_t s;
    bool have = link_status(&s, NULL);
    uint8_t want = (uint8_t)(s.flips + 1);
    present_fb = fb;
    post(RQ_PRESENT);
    if (!have)
        return false;
    absolute_time_t end = make_timeout_time_ms(timeout_ms);
    do {
        link_status(&s, NULL);
        if (s.flips == want)
            return true;
    } while (!time_reached(end));
    return false;
}

bool link_img_read(uint16_t first, uint16_t count, uint8_t *dst, unsigned d, uint32_t timeout_ms) {
    if (count == 0 || count > IMG_MAX)
        return false;
    memset(img_seen, 0, sizeof img_seen);
    img_first = first;
    img_count = count;
    img_d = d;
    img_got = 0;
    img_dst = dst;
    post(RQ_IMG);
    absolute_time_t end = make_timeout_time_ms(timeout_ms);
    while (img_got < count)
        if (time_reached(end))
            break;
    bool ok = img_got == count;
    img_dst = NULL;
    return ok;
}

int link_echo(const uint8_t *buf, unsigned len, uint32_t timeout_ms) {
    if (len == 0 || len > LINK_ECHO_MAX)
        return -1;
    memcpy(echo_want, buf, len);
    echo_len = len;
    echo_result = 0;
    post(RQ_ECHO);
    absolute_time_t end = make_timeout_time_ms(timeout_ms);
    while (echo_result == 0)
        if (time_reached(end))
            return 0;
    return echo_result;
}

void link_set_audio_fill(link_audio_fill_fn fill) {
    aud_fill = fill;
}

// Clearing: the register write is a round trip through core 1, so no callback is still running on return.
void link_set_tap(link_tap_fn fn) {
    tap_fn = fn;
    link_reg_write(REG_TAP, fn != NULL);
}

void link_set_key_reset(link_key_reset_fn fn) {
    key_reset_fn = fn;
}

void link_get_stats(link_stats_t *out) {
    phy_stats_t p;
    phy_get_stats(&p);
    *out = stats;
    out->dn_packets = p.dn_packets;
    out->dn_bytes = p.dn_bytes;
    out->dn_drops = p.dn_drops;
    out->up_resyncs = p.up_resyncs;
    out->up_bytes = p.up_bytes;
}
