// VCR emulation: the Panasonic deck's remote codes off the IR receiver, and its transport as the per-field
// picture, timing and audio the deck would send.
#ifndef VCR_H
#define VCR_H

#include <stdbool.h>
#include <stdint.h>

// Kaseikyo command byte, vendor 0x2002 (tools/vcr_ir.json).
enum { VCR_STOP = 0, VCR_REW = 2, VCR_FF = 3, VCR_PAUSE = 6, VCR_PLAY = 10, VCR_POWER = 61 };
// Front panel only (the remote has neither): eject, and put the next cassette in the slot.
enum { VCR_EJECT = 200, VCR_INSERT = 201 };

// ctl.vcr_state; keep in step with scripts/gcv2_ctl.sh.
enum {
    VS_OFF, VS_POWERING, VS_STOPPED, VS_THREADING, VS_PLAY, VS_PAUSE, VS_CUE_FWD, VS_CUE_REV, VS_RESUME,
    VS_STOPPING, VS_BLIP, VS_SPINUP_FWD, VS_SPINUP_REV, VS_WIND_FWD, VS_WIND_REV, VS_WIND_STOP, VS_POWERING_OFF,
    VS_LOADING, VS_EJECTING,
};

// ctl.vcr_cass: the game tape (data leader, then the tagged programme) or some other recording.
enum { CASS_NONE, CASS_GAME, CASS_MOVIE, CASS_N };

enum { VID_NONE, VID_BLUE, VID_TAPE };
enum { TRICK_NONE, TRICK_PAUSE, TRICK_CUE };

typedef struct {
    int H;                              // field length in half-lines
    int shift;                          // content lines below where play puts them
    uint32_t pos;                       // tape field shown
    uint8_t video, trick, cass;
    bool hifi;                          // deck audio on (else muted)
    uint32_t t;                         // fields since the state began: moves the cue noise bars
} vcr_field_t;

// What survives a GCV2 power cycle: which cassette is in and where each one is wound to.
typedef struct {
    uint32_t magic;
    uint32_t cass, last;
    uint32_t pos[CASS_N];
} vcr_save_t;
#define VCR_SAVE_MAGIC 0x56435253u

extern volatile uint32_t vcr_ir_frames, vcr_ir_bad, vcr_ir_last;
extern bool vcr_fast_wind;              // debug: winds reach the end of the tape in one field

void vcr_ir_init(unsigned pin);
void vcr_ir_service(void);              // core 0: decode finished frames, post commands to core 1
void vcr_post(int code);                // core 0: a front-panel press

// Core 0: true with the tape state to store when it changed and the deck shows no tape (flash is idle).
bool vcr_save_due(vcr_save_t *out);
void vcr_restore(const vcr_save_t *s);  // before core 1 starts

// Core 1, at every field start: `phase` 1 when the field starts mid-line.
void vcr_field(vcr_field_t *f, int phase, bool il, bool bypass, uint32_t len);
void vcr_command(int code);             // core 1: as if received now
void vcr_seek(uint32_t pos);
int vcr_state(void);
int vcr_cassette(void);

#endif
