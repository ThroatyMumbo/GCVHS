// The tape's state from STATUS each frame, and what entering a state does.
#include <stdio.h>
#include "tape_int.h"

tape_ui_state_t ui;

static const char *const tape_name[] = {"wait", "off", "stopped", "reading", "loading", "loaded", "bad crc",
                                        "past leader", "not a game tape", "pause/search"};
static const char *const READ_ERR = "TAPE READ ERROR";

// Two STATUS frames after release, so none still carries the old VERIFIED. Held in reset while rejected.
void tape_loader_reset(void) {
    vhs_status_t s = {0};
    link_reg_write(REG_LOADER, 1);
    link_wait_status(&s, 50);
    if (ui.reject) {
        ui.have = s.dt_have;
        return;
    }
    link_reg_write(REG_LOADER, 0);
    link_wait_status(&s, 50);
    link_wait_status(&s, 50);
    ui.have = s.dt_have;
}

bool tape_vitc_fresh(const vhs_status_t *s) { return s->vitc_age < VITC_FRESH; }
static bool ptag_fresh(const vhs_status_t *s) { return s->ptag_age < PTAG_FRESH; }

static bool ours(const vhs_status_t *s) {
    return (s->flags0 & (ST0_TAG_SEEN | ST0_LTC_LOCKED)) || tape_vitc_fresh(s) || ptag_fresh(s);
}

// Pause or search: position tags and no VITC, for long enough that it isn't a VITC dropout in play.
bool tape_trick(const vhs_status_t *s) {
    return !tape_vitc_fresh(s) && ptag_fresh(s) && (ui.state == T_TRICK || s->vitc_age >= TRICK_VITC);
}

bool tape_searching(void) { return ui.ptag_still < PTAG_STILL; }

// A key, a pause or search, or the deck stopped or another cassette: a new try at the tape.
void tape_retry(void) {
    ui.failed = false;
    if (ui.reject) {
        ui.reject = false;
        link_reg_write(REG_LOADER, 0);
    }
}

int32_t tape_pos(const vhs_status_t *s) {
    if (tape_vitc_fresh(s)) return (int32_t)s->vitc_pos;
    if (s->flags0 & ST0_LTC_LOCKED) return (int32_t)s->ltc_field;
    return -1;
}

tape_t tape_classify(const vhs_status_t *s) {
    bool lock = s->flags0 & ST0_IN_LOCK, blue = s->flags1 & ST1_BLUE;
    tape_t prev = ui.state;
    int past = ui.past;                 // a run of playing frames with a position and no data: any other frame ends it
    ui.past = 0;
    ui.unlock = lock ? 0 : inc(ui.unlock);
    ui.blue = lock && blue ? inc(ui.blue) : 0;
    if (ptag_fresh(s) && s->ptag_agreed != ui.ptag) {
        ui.ptag_dir = (int16_t)(s->ptag_agreed - ui.ptag) > 0 ? 1 : -1;
        ui.ptag_still = 0;
        ui.ptag = s->ptag_agreed;
    } else {
        ui.ptag_still = inc(ui.ptag_still);
    }
    bool moved = s->dt_have != ui.have;
    ui.have = s->dt_have;
    ui.have_idle = moved || (s->flags1 & ST1_DATA_LIVE) ? 0 : inc(ui.have_idle);

    if (s->flags1 & ST1_DT_VERIFIED) return s->flags1 & ST1_DT_OK ? T_LOADED : T_BAD;
    if (ui.unlock >= OFF_FRAMES) return T_OFF;
    if (!lock) return prev;
    if (ui.blue >= BLUE_FRAMES) return T_STOPPED;
    if (blue) return prev;
    if (ui.have_idle == 0 || (prev == T_LOADING && ui.have_idle < LOADING_HOLD)) {
        ui.foreign = 0;
        return T_LOADING;
    }
    // A dropout of the tag or the position keeps T_REWIND: re-entering it would rewind again.
    if (!ours(s)) {
        ui.foreign = inc(ui.foreign);
        return ui.foreign >= FOREIGN_FRAMES ? T_FOREIGN : prev == T_FOREIGN || prev == T_REWIND ? prev : T_READING;
    }
    ui.foreign = 0;
    if (tape_trick(s)) return T_TRICK;
    int32_t pos = tape_pos(s);
    if (pos >= 0) {
        ui.last_pos = (uint32_t)pos;
        ui.past = pos > PLAY_ON ? inc(past) : 0;   // the leader still: data lags the position after a review
        return ui.past >= REWIND_FRAMES || prev == T_REWIND ? T_REWIND : T_READING;
    }
    return prev == T_REWIND ? T_REWIND : T_READING;     // ours, but no VITC or LTC position yet
}

void tape_enter_state(tape_t t) {
    printf("tape: %s\n", tape_name[t]);
    if (t == T_BAD) {
        ui.err = READ_ERR;
        ui.err_detail = "REWIND THE TAPE AND PLAY IT AGAIN";
        ui.failed = ui.reject = true;   // else a small image re-verifies bad at once and T_BAD hides the deck
    }
    if (t == T_OFF || t == T_STOPPED || t == T_TRICK)
        tape_retry();
    // A fresh start for the next pass: clears a failed check and anything from another tape.
    if (t == T_OFF || t == T_STOPPED || t == T_FOREIGN || t == T_REWIND || t == T_BAD)
        tape_loader_reset();
    if (t == T_LOADING && !ui.failed)
        ui.err = NULL;
    if (t == T_REWIND || t == T_FOREIGN || t == T_BAD)
        ui.cursor = M_LOAD;
    // LOAD GAME from its rewind, unless this tape just failed; a running LOAD GAME fails here instead.
    if (t == T_REWIND && ui.al.st == A_IDLE && !ui.failed) {
        ui.err = NULL;
        ui.al.tries = ui.al.passes = 0;
        tape_rewind_from(ui.last_pos);
    }
}
