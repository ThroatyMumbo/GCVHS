// GCV2's audio.h over the link: the DE1 pulls samples (AUD_REQ), core 1 fills them from the game or the synth.
// The codec runs at 48.83 kHz and the synth is tuned to it; a game's own 48 kHz fill plays 1.7 % sharp.
#include "audio/audio.h"
#include "audio/audio_synth.h"

#include <string.h>
#include "link.h"

#define LINK_AUDIO_RATE 48828u  // 50 MHz / 1024
#define SCOPE_RING      1024u

static int16_t sine_lut[AUDIO_SINE_LUT_SIZE];
static audio_tone_t tone;
static audio_chip_t chip;
static volatile audio_fill_fn fill_cb;
static volatile bool enabled = true;
static int16_t scope_ring[SCOPE_RING];
static volatile uint32_t scope_w;

static void link_fill(int16_t *dst, unsigned frames) {
    audio_fill_fn cb = fill_cb;
    if (enabled && cb)
        cb(dst, frames);
    else
        memset(dst, 0, frames * 4);
    uint32_t w = scope_w;
    for (unsigned i = 0; i < frames; i++)
        scope_ring[(w + i) & (SCOPE_RING - 1u)] = dst[2u * i];
    scope_w = w + frames;
}

void audio_init(void) {
    audio_synth_build_sine(sine_lut, AUDIO_SINE_LUT_SIZE);
    audio_tone_init(&tone, sine_lut, AUDIO_SINE_LUT_SIZE, 0);
    audio_chip_init(&chip);
    link_set_audio_fill(link_fill);
}

// Off: send the DE1 no audio at all, rather than silence.
void audio_link_attach(bool on) {
    link_set_audio_fill(on ? link_fill : NULL);
}

void audio_set_fill_callback(audio_fill_fn cb) {
    fill_cb = cb;
}

// Nominal: the DE1 grants 512 frames while queued + owed < 1024 (cpu_link.v), so up to ~1535 can be queued.
uint32_t audio_out_latency_frames(void) {
    return 1024u + 256u;
}

static void tone_cb(int16_t *dst, unsigned frames) {
    audio_tone_fill(&tone, dst, frames);
}

void audio_play_test_tone(unsigned freq_hz) {
    audio_tone_set_freq(&tone, freq_hz, LINK_AUDIO_RATE);
    tone.amp = 16000;
    tone.phase = 0;
    fill_cb = tone_cb;
}

void audio_set_enabled(bool en) {
    enabled = en;
}

static void chip_cb(int16_t *dst, unsigned frames) {
    audio_chip_fill(&chip, dst, frames);
}

void audio_play_note(int ch, int freq_hz, int vol, int duration_ms) {
    audio_chip_note(&chip, ch, freq_hz, vol, duration_ms, LINK_AUDIO_RATE);
    fill_cb = chip_cb;
}

void audio_noise(int vol, int duration_ms) {
    audio_chip_note(&chip, AUDIO_CH_NOISE, 8000, vol, duration_ms, LINK_AUDIO_RATE);
    fill_cb = chip_cb;
}

void audio_set_duty(int ch, int duty) {
    audio_chip_set_duty(&chip, ch, duty);
}

void audio_stop(int ch) {
    audio_chip_stop(&chip, ch);
}

void audio_stop_all(void) {
    audio_chip_stop_all(&chip);
}

unsigned audio_scope_read(int16_t *dst, unsigned n) {
    if (n > SCOPE_RING)
        n = SCOPE_RING;
    uint32_t start = scope_w - n;
    for (unsigned i = 0; i < n; i++)
        dst[i] = scope_ring[(start + i) & (SCOPE_RING - 1u)];
    return n;
}
