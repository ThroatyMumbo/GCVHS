// Inside the BIOS tape screen: tape_state.c reads the tape off STATUS, tape_load.c runs LOAD GAME, tape_draw.c draws,
// tape_ui.c has the controller and the loop. Everyone else uses tape_ui.h.
#ifndef TAPE_INT_H
#define TAPE_INT_H

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include "link.h"
#include "vcr.h"

// Tape and deck; frames are output frames (60/s).
#define PLAY_ON         150             // LOAD GAME plays on from here: a rewound deck reads ~110 fields in (picture
                                        // 1.78 s after PLAY), and the leader still holds a whole pass after it
#define WIND_SPEED      60              // E: fields per field winding
#define WIND_SPINUP     30              // E: frames before a wind gets going
#define WIND_MARGIN     60              // frames past a wind's estimate before it counts as done
#define REVIEW_MAX      (5 * 60 * 60)   // fields: this close to the start, review back on the position tags
#define REVIEW_AT       100             // review until the tags pass this; PLAY lands 12-35 fields before it
#define REVIEW_BACK     120             // fields above the review's lowest tag: the deck plays forward, REW was missed
#define REVIEW_STALL    (45 * 60)       // frames without progress: the deck stalled (they can for 30 s); wind
#define UNKNOWN_POS     432000          // fields: a blind wind with no position read times a whole T-120 tape
#define PASSES          3               // rewinds before LOAD GAME takes a landing past the leader as final
#define LOCK_SETTLE     10              // frames in A_POWER before a lock counts, so a deck already on goes straight on
#define LOCK_WAIT       90              // M: lock 0.3-0.4 s after POWER
#define PICTURE_WAIT    300             // M: picture 1.78 s after PLAY from stop (1.7-4.5 s in SLP)
#define STOP_WAIT       45              // M: tape gone 0.1 s after STOP; E: a wind stops in 0.5 s
#define LOAD_WAIT       1800            // LOAD GAME gives up on a leader that never completes
#define OFF_FRAMES      30              // no lock this long is off, not STOP's ~0.1 s blip
#define BLUE_FRAMES     10
#define REWIND_FRAMES   60              // our tape past the leader, with no data, this long
#define FOREIGN_FRAMES  180             // a picture with nothing of ours on it this long
#define LOADING_HOLD    60              // no new slot this long ends a load pass
#define IDLE_FRAMES     (5 * 60 * 60)   // then the screensaver
#define STALE_MS        500             // no STATUS this long: the DE1 is gone
#define VITC_FRESH      8               // fields since a good VITC word for it to give the position
#define PTAG_FRESH      30              // fields since a position tag for the picture to count as ours
#define PTAG_NEW        8               // fields since a position tag for a review to steer by it
#define TRICK_VITC      30              // fields without VITC, with position tags, before it reads as pause or search
#define PTAG_STILL      20              // frames without a new position tag: paused, not searching
#define FF_SHOWN        600             // frames FAST FWD stays up after FF over a blue picture

typedef enum { T_WAIT, T_OFF, T_STOPPED, T_READING, T_LOADING, T_LOADED, T_BAD, T_REWIND, T_FOREIGN, T_TRICK } tape_t;
typedef enum { A_IDLE, A_POWER, A_PROBE, A_REVIEW, A_STOP, A_REW, A_STOP2, A_PLAY, A_LOAD } auto_t;
enum { M_POWER, M_REW, M_PLAY, M_PAUSE, M_FF, M_STOP, M_LOAD, M_N };

typedef struct {
    tape_t state;
    int unlock, blue, foreign, past, have_idle;
    uint16_t have, ptag;
    int ptag_dir, ptag_still;           // the last position tag change's sign, and frames since
    int last_key, key_age;
    // failed: keep the error, don't rewind by itself; reject: this pass's image is bad, loader held. Until retry.
    bool failed, reject;
    uint32_t last_pos;                  // last tape position read, fields
    int cursor, hit, hit_age;
    int idle;
    int8_t c_prev;                      // C-left/C-right, which gamepad.c reports as the right stick
    const char *err, *err_detail;
    // keyed: this step sent its key; best and moved: a review's lowest tag and the frame it was read
    struct { auto_t st; int t, tries, passes, moved; uint32_t pos, best; bool keyed; } al;
    float sx, sy, svx, svy;             // screensaver
} tape_ui_state_t;
extern tape_ui_state_t ui;              // tape_state.c

static inline int inc(int v) { return v < INT_MAX ? v + 1 : v; }

// tape_state.c
tape_t tape_classify(const vhs_status_t *s);
void tape_enter_state(tape_t t);
void tape_loader_reset(void);
void tape_retry(void);
int32_t tape_pos(const vhs_status_t *s);    // fields, or -1
bool tape_vitc_fresh(const vhs_status_t *s);
bool tape_trick(const vhs_status_t *s);
bool tape_searching(void);

// tape_load.c
extern const char *const tape_auto_msg[];
void tape_auto_go(auto_t st);
void tape_auto_step(const vhs_status_t *s);
void tape_rewind_from(uint32_t pos);
int tape_wind_frames(uint32_t pos);         // how long a rewind from pos can take

// tape_draw.c
void tape_draw(const vhs_status_t *s);
void tape_picture(bool on);

// tape_ui.c
void tape_deck_key(uint8_t cmd);

#endif
