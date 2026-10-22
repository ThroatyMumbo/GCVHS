// LOAD GAME: POWER if needed, PLAY to read the position, rewind unless near the start, PLAY the leader.
#include <stdio.h>
#include "tape_int.h"

const char *const tape_auto_msg[] = {"", "TURNING THE VCR ON", "FINDING THE TAPE POSITION", "REWINDING",
                                     "REWINDING", "REWINDING", "REWINDING", "PLAYING THE TAPE", "LOADING"};

int tape_wind_frames(uint32_t pos) {
    return WIND_SPINUP + (int)(pos / WIND_SPEED) + WIND_MARGIN;
}

void tape_auto_go(auto_t st) {
    ui.al.st = st;
    ui.al.t = 0;
    ui.al.keyed = false;
    if (st != A_IDLE) printf("load: %s\n", tape_auto_msg[st]);
}

// Back to the start from field pos while the tape plays: by review when the position is known and near enough,
// else by a blind wind (a real deck's wind changes speed and stalls, so only as the last resort).
void tape_rewind_from(uint32_t pos) {
    ui.al.pos = pos;
    tape_auto_go(pos <= REVIEW_MAX ? A_REVIEW : A_STOP);
}

static void auto_fail(const char *why, const char *detail) {
    printf("load: failed: %s\n", why);
    ui.err = why;
    ui.err_detail = detail;
    ui.failed = true;
    tape_auto_go(A_IDLE);
}

void tape_auto_step(const vhs_status_t *s) {
    int32_t pos = tape_pos(s);
    bool lock = ui.unlock == 0;
    int t = ui.al.t;
    ui.al.t = inc(t);
    switch (ui.al.st) {
    case A_IDLE:
        break;
    case A_POWER:
        if (lock && t > LOCK_SETTLE) {
            tape_auto_go(A_PROBE);
            break;
        }
        if (t == 0 && ui.state == T_OFF) tape_deck_key(KEY_POWER);  // POWER toggles: never on a lock blip
        if (t > LOCK_WAIT) {
            if (++ui.al.tries < 2) ui.al.t = 0;
            else auto_fail("THE VCR DID NOT TURN ON", "CHECK THE VCR AND ITS CABLES");
        }
        break;
    case A_PROBE:
        if (ui.state == T_LOADING) {
            tape_auto_go(A_LOAD);
        } else if (pos >= 0 && (s->flags0 & ST0_IN_LOCK) && !(s->flags1 & ST1_BLUE)) {
            printf("load: tape at field %ld\n", (long)pos);
            if ((uint32_t)pos > PLAY_ON) tape_rewind_from((uint32_t)pos);
            else tape_auto_go(A_LOAD);
        } else if (!ui.al.keyed && (ui.state == T_STOPPED || ui.state == T_TRICK || ui.state == T_FOREIGN)) {
            tape_deck_key(KEY_PLAY);    // a deck just powered on reads as stopped a few frames later
            ui.al.keyed = true;
            ui.al.t = 0;
        } else if (t > PICTURE_WAIT) {
            if (ui.state == T_FOREIGN) auto_fail("THIS IS NOT A GAME TAPE", "EJECT IT AND INSERT A GAME TAPE");
            else if (ui.state == T_READING || ui.state == T_TRICK) tape_rewind_from(UNKNOWN_POS);   // no position
            else auto_fail("NO TAPE PICTURE", "IS A TAPE IN THE VCR?");
        }
        break;
    case A_REVIEW:                      // REW from play reviews; PLAY once the tags pass REVIEW_AT
        if (t == 0) {
            tape_deck_key(KEY_REW);
            ui.al.best = ui.al.pos;
            ui.al.moved = 0;
        }
        if (s->ptag_age < PTAG_NEW) {   // the low 16 bits of the position, which is under REVIEW_MAX here
            uint32_t p = s->ptag_agreed;
            if (p <= REVIEW_AT) {
                tape_auto_go(A_PLAY);
                break;
            }
            if (p < ui.al.best) {
                ui.al.best = p;
                ui.al.moved = t;
            } else if (p > ui.al.best + REVIEW_BACK && !ui.al.keyed) {
                tape_deck_key(KEY_REW);
                ui.al.keyed = true;
                ui.al.best = p;
                ui.al.moved = t;
            }
        }
        if (ui.unlock >= OFF_FRAMES || t - ui.al.moved > REVIEW_STALL) {
            printf("load: review stalled at field %lu, winding\n", (unsigned long)ui.al.best);
            ui.al.pos = ui.al.best;
            tape_auto_go(A_STOP);
        }
        break;
    case A_STOP:
    case A_STOP2:
        if (t == 0) tape_deck_key(KEY_STOP);
        if (t > STOP_WAIT) tape_auto_go(ui.al.st == A_STOP ? A_REW : A_PLAY);
        break;
    case A_REW:
        if (t == 0) tape_deck_key(KEY_REW);
        if (t > tape_wind_frames(ui.al.pos)) tape_auto_go(A_STOP2);   // the deck stops by itself at the start
        break;
    case A_PLAY:
        if (t == 0) tape_deck_key(KEY_PLAY);
        if (ui.state == T_LOADING) {
            tape_auto_go(A_LOAD);
        } else if (tape_vitc_fresh(s)) {    // landed past the leader (a wind cut short): now it's known, review on
            if (s->vitc_pos > PLAY_ON && ++ui.al.passes < PASSES) tape_rewind_from(s->vitc_pos);
            else tape_auto_go(A_LOAD);
        } else if (t > PICTURE_WAIT) {
            auto_fail("NO TAPE PICTURE", "IS A TAPE IN THE VCR?");
        }
        break;
    case A_LOAD:
        if (ui.state == T_REWIND) auto_fail("NO GAME FOUND ON THE TAPE", "IS THIS THE RIGHT TAPE?");
        else if (ui.state == T_BAD) tape_auto_go(A_IDLE);
        else if (t > LOAD_WAIT) auto_fail("THE GAME DID NOT LOAD", "REWIND THE TAPE AND TRY AGAIN");
        break;
    }
}
