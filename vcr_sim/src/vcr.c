// VCR emulation: Kaseikyo frames off the IR receiver on core 0, and the deck's
// transport stepped once per output field on core 1. Timings are in fields, counted from the decoded frame.
#include "vcr.h"
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#define LAG 4                           // Kaseikyo frame (~56 ms) + end gap: the delays below run from the send's start
#define F(ms) ((int)(((ms) * 5994 + 50000) / 100000))

// Deck timings: M measured, D derived, E estimate.
#define ADV_ACQ       F(170)            // the ADV7180 raises IN_LOCK ~0.17 s after a signal appears (bench)
#define POWER_ON      (F(350) - ADV_ACQ - LAG)  // M: lock 0.3-0.4 s after POWER
#define POWER_OFF     (F(200) - LAG)    // E
#define THREAD        (F(1780) - LAG)   // M: SP, n=1
#define PLAY_SKIP     52                // M: fields past the STOP position when PLAY brings the picture back
#define PLAY_HIFI     F(500)            // E: Hi-Fi after the picture, from stop
#define TRICK_AT      (F(300) - LAG)    // M: VITC stops ~0.3 s after PAUSE or FF
#define RES_A         F(350)            // M (SLP): the tag slides for ~0.35 s after PLAY from pause
#define RES_END_P     (F(1380) - LAG)   // M: tag back on line 10, 1.37-1.39 s
#define RES_HIFI_P    (F(670) - LAG - 4)    // D: LTC relock 0.67 s, less the two words it takes
#define RES_END_C     (F(1400) - LAG)   // E
#define RES_HIFI_C    (F(990) - LAG - 4)    // D: LTC relock 0.99 s after PLAY from cue
#define STOP_AT       (F(120) - LAG)    // M: tags gone 0.09-0.16 s after STOP
#define BLIP_N        F(200)           // D: ~100 ms IN_LOCK blip; the ADV7180 drops it ~0.24 s into a gap, regains it ~0.17 s after
#define WIND_AT       (F(500) - LAG)    // E
#define WIND_STOP_N   F(500)            // E
#define WIND_SPEED    60                // E: T-120 end to end in ~2 min
#define CUE_SPEED     7                 // M: SP (21 in SLP)
#define PAUSE_TIMEOUT F(300000)         // E: 5 min
#define LOAD_N        F(1500)           // E: cassette drawn in and seated
#define EJECT_N       F(1500)           // E: unthreaded and pushed out
#define HOLD_MS       250               // same code again within this is a held button, not a press
#define GAP_US        8000

volatile uint32_t vcr_ir_frames, vcr_ir_bad, vcr_ir_last;
bool vcr_fast_wind;

// IR: the ISR timestamps edges; vcr_ir_service turns them into mark/space durations and decodes.
#define EV_N 256
static uint ir_pin;
static volatile uint32_t ev_t[EV_N];
static volatile uint8_t ev_l[EV_N];
static volatile uint32_t ev_head;
static uint32_t ev_tail;

// Commands from core 0 (IR and front panel) to core 1.
#define MB_N 8
static volatile uint8_t mb_cmd[MB_N];
static volatile uint32_t mb_head, mb_tail;

void vcr_post(int code) {
    if (mb_head - mb_tail >= MB_N) return;
    mb_cmd[mb_head % MB_N] = (uint8_t)code;
    __dmb();
    mb_head = mb_head + 1;
}

static void ir_isr(uint gpio, uint32_t events) {
    (void)events;
    if (gpio != ir_pin) return;
    uint32_t i = ev_head % EV_N;
    ev_t[i] = time_us_32();
    ev_l[i] = (uint8_t)gpio_get(gpio);
    __dmb();
    ev_head = ev_head + 1;
}

void vcr_ir_init(unsigned pin) {
    ir_pin = pin;
    gpio_init(pin);
    gpio_pull_up(pin);
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, ir_isr);
}

static bool near(uint32_t x, uint32_t t, uint32_t pct) {
    return x * 100 >= t * (100 - pct) && x * 100 <= t * (100 + pct);
}

// tools/ir.py dec_kaseikyo: header, 48 pulse-distance bits LSB first, stop mark.
static bool kaseikyo(const uint32_t *d, int n, uint64_t *v) {
    if (n != 99 || !near(d[0], 3456, 20) || !near(d[1], 1728, 20) || !near(d[98], 432, 30)) return false;
    *v = 0;
    for (int k = 0; k < 48; k++) {
        uint32_t m = d[2 + 2 * k], s = d[3 + 2 * k];
        if (!near(m, 432, 30)) return false;
        if (near(s, 1296, 30)) *v |= 1ull << k;
        else if (!near(s, 432, 30)) return false;
    }
    return true;
}

static uint32_t dur[100];
static int nd = -1;                     // durations so far; -1 between frames
static uint32_t last_t, held_t;
static int last_lvl = 1, held_code = -1;

static void frame_done(void) {
    int n = nd;
    nd = -1;
    if (n < 3) return;                  // lone short pulses from the receiver between presses
    uint64_t v;
    uint8_t b[4];
    if (!kaseikyo(dur, n, &v)) goto bad;
    for (int k = 0; k < 4; k++) b[k] = (uint8_t)(v >> (16 + 8 * k));
    if ((v & 0xFFFF) != 0x2002 || b[0] != 0x90 || b[1] != 0 || b[3] != (b[0] ^ b[1] ^ b[2])) goto bad;
    vcr_ir_frames = vcr_ir_frames + 1;
    vcr_ir_last = 0x20020000u | b[2];
    uint32_t now = time_us_32();
    bool held = b[2] == held_code && now - held_t < HOLD_MS * 1000u;
    held_code = b[2];
    held_t = now;
    if (held) return;
    vcr_post(b[2]);
    return;
bad:
    vcr_ir_bad = vcr_ir_bad + 1;
}

void vcr_ir_service(void) {
    while (ev_tail != ev_head) {
        uint32_t i = ev_tail % EV_N, t = ev_t[i];
        int lvl = ev_l[i];
        ev_tail++;
        if (lvl == last_lvl) continue;
        if (nd >= 0) {
            if (t - last_t > GAP_US) frame_done();
            else if (nd < (int)(sizeof dur / sizeof dur[0])) dur[nd++] = t - last_t;
        }
        if (nd < 0 && lvl == 0) nd = 0; // the receiver pulls low for a mark
        last_t = t;
        last_lvl = lvl;
    }
    if (nd >= 0 && last_lvl == 1 && time_us_32() - last_t > GAP_US) frame_done();
}

// Transport, core 1 only.
#define NEVER UINT32_MAX
static volatile int st = VS_OFF;
static int until, off_video, pend = -1;
static uint32_t now, t, pend_at, hifi_at = NEVER;
static int64_t pos;
static int res_speed, res_s0, res_s1, last_shift, stop_step, last_video;
static int cass = CASS_GAME, last_cass = CASS_GAME;
static uint32_t cpos[CASS_N];           // where each cassette out of the slot was left
static bool eject_pend, rew_pend;       // eject / rewind (end of tape) once the deck has stopped

// Published by core 1 whenever the deck rests; core 0 stores it.
static vcr_save_t pub, pub_last;
static volatile uint32_t pub_seq;
static uint32_t saved_seq;

static void enter(int s, int len) {
    st = s;
    t = 0;
    until = len;
}

static void go(int s, int len) {
    pend = -1;
    enter(s, len);
}

static void later(int s) {
    pend = s;
    pend_at = now + TRICK_AT;
    hifi_at = NEVER;                    // D: LTC drops 0.56 s after the send, ltc_rx's 30-field HOLD after the mute
}

static void stop(void) {
    stop_step = st == VS_PLAY || st == VS_RESUME;
    hifi_at = NEVER;
    go(VS_STOPPING, STOP_AT);
}

// The deck has come to rest: carry out an eject or end-of-tape rewind that was waiting for it.
static void stopped(void) {
    if (eject_pend && cass != CASS_NONE) go(VS_EJECTING, EJECT_N);
    else if (rew_pend && pos > 0) go(VS_SPINUP_REV, WIND_AT);
    else go(VS_STOPPED, 0);
    eject_pend = rew_pend = false;
}

static void eject(void) {
    if (cass == CASS_NONE) return;
    switch (st) {
    case VS_OFF: eject_pend = true; go(VS_POWERING, POWER_ON); break;
    case VS_STOPPED: go(VS_EJECTING, EJECT_N); break;
    case VS_PLAY: case VS_RESUME: case VS_PAUSE: case VS_CUE_FWD: case VS_CUE_REV:
        eject_pend = true;
        stop();
        break;
    case VS_THREADING: case VS_SPINUP_FWD: case VS_SPINUP_REV: case VS_WIND_FWD: case VS_WIND_REV:
        eject_pend = true;
        go(VS_WIND_STOP, WIND_STOP_N);
        break;
    case VS_POWERING: case VS_STOPPING: case VS_BLIP: case VS_WIND_STOP: case VS_LOADING:
        eject_pend = true;
        break;
    default: break;
    }
}

static void insert(void) {
    if (cass != CASS_NONE) {
        eject();
        return;
    }
    if (st != VS_OFF && st != VS_STOPPED) return;
    cass = last_cass == CASS_GAME ? CASS_MOVIE : CASS_GAME;
    pos = cpos[cass];
    if (st == VS_OFF) go(VS_POWERING, POWER_ON);    // E: like most decks, a cassette switches it on
    else go(VS_LOADING, LOAD_N);
}

static void resume(bool from_pause) {
    go(VS_RESUME, from_pause ? RES_END_P : RES_END_C);
    res_speed = from_pause ? 2 : 1;     // M (SLP): the count steps 2 per field while the tag slides; E from cue
    res_s0 = last_shift;
    res_s1 = from_pause ? -6 : 2;       // M (SLP): tag from line 11 up to line 4; E from cue
    hifi_at = now + (uint32_t)(from_pause ? RES_HIFI_P : RES_HIFI_C);
}

void vcr_command(int c) {
    if (c == VCR_EJECT) {
        eject();
        return;
    }
    if (c == VCR_INSERT) {
        insert();
        return;
    }
    if (c == VCR_POWER) {
        if (st == VS_OFF) go(VS_POWERING, POWER_ON);
        else if (st != VS_POWERING_OFF) {
            off_video = last_video;
            hifi_at = NEVER;
            go(VS_POWERING_OFF, POWER_OFF);
        }
        return;
    }
    switch (st) {
    case VS_STOPPED:
        if (cass == CASS_NONE) break;
        if (c == VCR_PLAY) go(VS_THREADING, THREAD);
        if (c == VCR_FF) go(VS_SPINUP_FWD, WIND_AT);
        if (c == VCR_REW) go(VS_SPINUP_REV, WIND_AT);
        break;
    case VS_THREADING:
        if (c == VCR_STOP) go(VS_WIND_STOP, WIND_STOP_N);
        break;
    case VS_PLAY:
    case VS_RESUME:
        if (c == VCR_PAUSE) later(VS_PAUSE);
        if (c == VCR_FF) later(VS_CUE_FWD);
        if (c == VCR_REW) later(VS_CUE_REV);
        if (c == VCR_STOP) stop();
        break;
    case VS_PAUSE:
        if (c == VCR_PLAY || c == VCR_PAUSE) resume(true);
        if (c == VCR_FF) later(VS_CUE_FWD);
        if (c == VCR_REW) later(VS_CUE_REV);
        if (c == VCR_STOP) stop();
        break;
    case VS_CUE_FWD:
    case VS_CUE_REV:
        if (c == VCR_PLAY) resume(false);
        if (c == VCR_PAUSE) later(VS_PAUSE);
        if (c == VCR_FF && st == VS_CUE_REV) later(VS_CUE_FWD);
        if (c == VCR_REW && st == VS_CUE_FWD) later(VS_CUE_REV);
        if (c == VCR_STOP) stop();
        break;
    case VS_SPINUP_FWD:
    case VS_SPINUP_REV:
    case VS_WIND_FWD:
    case VS_WIND_REV:
        if (c == VCR_STOP) go(VS_WIND_STOP, WIND_STOP_N);
        if (c == VCR_PLAY) go(VS_THREADING, WIND_STOP_N + THREAD);
        if (c == VCR_FF && st != VS_SPINUP_FWD && st != VS_WIND_FWD) go(VS_SPINUP_FWD, WIND_AT);
        if (c == VCR_REW && st != VS_SPINUP_REV && st != VS_WIND_REV) go(VS_SPINUP_REV, WIND_AT);
        break;
    default:                            // E: keys are ignored while the mechanism is moving
        break;
    }
}

static void timer_done(void) {
    switch (st) {
    case VS_POWERING: stopped(); break;
    case VS_POWERING_OFF: enter(VS_OFF, 0); eject_pend = rew_pend = false; break;
    case VS_LOADING: stopped(); break;
    case VS_EJECTING:
        cpos[cass] = (uint32_t)pos;
        last_cass = cass;
        cass = CASS_NONE;
        pos = 0;
        enter(VS_STOPPED, 0);
        break;
    case VS_THREADING:
        pos += PLAY_SKIP;
        hifi_at = now + PLAY_HIFI;
        enter(VS_PLAY, 0);
        break;
    case VS_RESUME: enter(VS_PLAY, 0); break;
    case VS_STOPPING: enter(VS_BLIP, BLIP_N); break;
    case VS_BLIP: stopped(); break;
    case VS_SPINUP_FWD: enter(VS_WIND_FWD, 0); break;
    case VS_SPINUP_REV: enter(VS_WIND_REV, 0); break;
    case VS_WIND_STOP: stopped(); break;
    default: break;
    }
}

void vcr_field(vcr_field_t *f, int phase, bool il, bool bypass, uint32_t len) {
    now++;
    t++;
    while (mb_tail != mb_head) {
        __dmb();
        int c = mb_cmd[mb_tail % MB_N];
        mb_tail = mb_tail + 1;
        if (!bypass) vcr_command(c);
    }
    if (bypass) {
        if (st != VS_PLAY) go(VS_PLAY, 0);
        pend = -1;
        hifi_at = 0;
    }
    if (pend >= 0 && now >= pend_at) {
        enter(pend, 0);
        pend = -1;
    }
    if (until && t >= (uint32_t)until) timer_done();

    int step = 0;
    f->H = il ? 525 : 524;
    f->shift = 0;
    f->trick = TRICK_NONE;
    f->video = VID_TAPE;
    switch (st) {
    case VS_OFF:
    case VS_POWERING:
    case VS_BLIP:
        f->video = VID_NONE;
        break;
    case VS_POWERING_OFF:
        f->video = (uint8_t)off_video;
        break;
    case VS_THREADING:                  // on the blue screen, so the first tape field starts in step
        f->video = VID_BLUE;
        if (il && (phase ^ ((until - (int)t) & 1)) != (int)((pos + PLAY_SKIP) & 1)) f->H = 526;
        break;
    case VS_STOPPED:
    case VS_SPINUP_FWD:
    case VS_SPINUP_REV:
    case VS_WIND_STOP:
    case VS_LOADING:
    case VS_EJECTING:
        f->video = VID_BLUE;
        break;
    case VS_WIND_FWD:
    case VS_WIND_REV:
        f->video = VID_BLUE;
        step = vcr_fast_wind ? (int)len + 1 : WIND_SPEED;
        if (st == VS_WIND_REV) step = -step;
        break;
    case VS_PLAY:
        if (il && phase != (int)(pos & 1)) f->H = 526;  // one 263-line field brings the tape's field parity in step
        step = 1;
        break;
    case VS_STOPPING:
        step = stop_step;
        break;
    case VS_PAUSE:
        f->trick = TRICK_PAUSE;
        f->H = t & 1 ? 524 : 528;       // M: 264/262 lines
        f->shift = t & 1 ? 2 : 1;       // M: SP tag on lines 11/12
        if (t >= PAUSE_TIMEOUT) stop();
        break;
    case VS_CUE_FWD:
        f->trick = TRICK_CUE;
        f->H = 505;                     // M: 252/253 lines
        f->shift = t & 1 ? -3 : -4;     // M: SP tag on lines 6/7
        step = CUE_SPEED;
        break;
    case VS_CUE_REV:
        f->trick = TRICK_CUE;
        f->H = 545;                     // E: cue mirrored, 272/273 lines
        f->shift = t & 1 ? 4 : 3;       // E
        step = -CUE_SPEED;
        break;
    case VS_RESUME:
        if (t < (uint32_t)RES_A) {
            f->shift = res_s0 + (res_s1 - res_s0) * (int)t / RES_A;
            step = res_speed;
        } else {
            f->shift = 2;               // M (SLP): line 12 at normal speed until the snap back to line 10
            step = 1;
            if (il && phase != (int)(pos & 1)) f->H = 526;  // in step before the snap, while the tag is off line 10
        }
        break;
    default:
        break;
    }
    if (pos > len) pos = len;
    f->pos = (uint32_t)pos;
    f->cass = bypass ? CASS_GAME : (uint8_t)cass;
    f->hifi = f->video == VID_TAPE && now >= hifi_at;
    f->t = t;
    last_shift = f->shift;
    last_video = f->video;

    pos += step;
    if (pos < 0 || pos > len) {         // E: the deck stops at either end, and rewinds from the far one
        bool end = pos > len;
        pos = pos < 0 ? 0 : len;
        if (bypass) pos = 0;
        else if (st != VS_STOPPING) {
            rew_pend = end;
            if (f->video == VID_TAPE) stop();
            else stopped();
        }
    }

    if (!bypass && (st == VS_STOPPED || st == VS_OFF)) {
        vcr_save_t s = {VCR_SAVE_MAGIC, (uint32_t)cass, (uint32_t)last_cass, {0}};
        for (int k = 0; k < CASS_N; k++) s.pos[k] = cpos[k];
        if (cass != CASS_NONE) s.pos[cass] = (uint32_t)pos;
        s.pos[CASS_NONE] = 0;
        if (memcmp(&s, &pub_last, sizeof s)) {
            pub_last = s;
            pub = s;
            __dmb();
            pub_seq = pub_seq + 1;
        }
    }
}

bool vcr_save_due(vcr_save_t *out) {
    uint32_t seq = pub_seq;
    if (seq == saved_seq || (st != VS_STOPPED && st != VS_OFF)) return false;
    __dmb();
    *out = pub;
    __dmb();
    if (pub_seq != seq) return false;
    saved_seq = seq;
    return true;
}

void vcr_restore(const vcr_save_t *s) {
    if (s->magic != VCR_SAVE_MAGIC || s->cass >= CASS_N || s->last == CASS_NONE || s->last >= CASS_N) return;
    for (int k = 0; k < CASS_N; k++) cpos[k] = s->pos[k];
    cass = (int)s->cass;
    last_cass = (int)s->last;
    pos = cass != CASS_NONE ? cpos[cass] : 0;
    pub_last = *s;
}

void vcr_seek(uint32_t p) { pos = p; }

int vcr_state(void) { return st; }

int vcr_cassette(void) { return cass; }
