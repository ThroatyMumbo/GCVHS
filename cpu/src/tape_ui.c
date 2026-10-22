// BIOS screen: the tape's state from STATUS, the deck worked over IR from the controller (a transport bar,
// direct buttons, and LOAD GAME, which powers, rewinds and plays by itself), drawn over the deck's picture.
#include "tape_ui.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "video/video.h"
#include "input/gamepad.h"
#include "tape_int.h"

static const uint8_t menu_key[M_N] = {KEY_POWER, KEY_REW, KEY_PLAY, KEY_PAUSE, KEY_FF, KEY_STOP, 0};

void tape_deck_key(uint8_t cmd) {
    link_ir_send(vcr_ir_payload(cmd));
    ui.last_key = cmd;
    ui.key_age = 0;
    printf("deck: key %d\n", cmd);
}

static void activate(int item) {
    ui.hit = item;
    ui.hit_age = 0;
    ui.err = NULL;
    tape_retry();
    if (item == M_LOAD) {
        ui.al.tries = ui.al.passes = 0;
        tape_auto_go(A_POWER);
    } else {
        tape_deck_key(menu_key[item]);
    }
}

volatile uint32_t tape_ui_press;        // bench aid over SWD: menu item + 1, as if picked and pressed A

static void pad(void) {
    gamepad_state_t p[2];
    gamepad_poll();
    gamepad_read_port(0, &p[0]);
    gamepad_read_port(1, &p[1]);
    uint16_t pr = p[0].buttons_pressed | p[1].buttons_pressed;
    uint32_t inj = tape_ui_press;
    if (inj && inj <= M_N) {
        tape_ui_press = 0;
        ui.cursor = (int)inj - 1;
        pr |= BTN_A;
    }
    int8_t c = (int8_t)(p[0].right_x ? p[0].right_x : p[1].right_x);
    bool c_left = c < 0 && ui.c_prev >= 0, c_right = c > 0 && ui.c_prev <= 0;
    ui.c_prev = c;
    if (!pr && !c_left && !c_right) return;
    ui.idle = 0;
    if (ui.al.st != A_IDLE) {           // any button takes the deck back
        printf("load: cancelled\n");
        tape_auto_go(A_IDLE);
        return;
    }
    if (pr & BTN_DPAD_LEFT) ui.cursor = (ui.cursor + M_N - 1) % M_N;
    if (pr & BTN_DPAD_RIGHT) ui.cursor = (ui.cursor + 1) % M_N;
    if (pr & BTN_A) activate(ui.cursor);
    if (pr & BTN_B) activate(M_STOP);
    if (pr & BTN_X) activate(M_PAUSE);              // Z
    if (pr & BTN_START) activate(M_POWER);
    if ((pr & BTN_LB) || c_left) activate(M_REW);
    if ((pr & BTN_RB) || c_right) activate(M_FF);
}

bool tape_ui_run(const char *error, const char *detail, bool (*poll)(void)) {
    vhs_status_t s;
    bool gone = false;
    ui.state = T_WAIT;
    ui.unlock = ui.blue = ui.foreign = ui.past = 0;
    ui.have_idle = LOADING_HOLD;
    ui.cursor = M_LOAD;
    ui.idle = 0;
    ui.hit = -1;
    tape_auto_go(A_IDLE);
    ui.failed = ui.reject = error != NULL;  // the same image would fail the same way: no reload, no rewind
    tape_loader_reset();                // only an image read off the tape from here on counts
    ui.err = error;
    ui.err_detail = detail;
    for (;;) {
        watchdog_update();
        if (poll && poll())
            return false;
        if (link_status_age_ms() > STALE_MS) {
            if (!gone) printf("link: no STATUS from the DE1\n");
            gone = true;
            link_wait_status(&s, 100);
            continue;
        }
        if (gone) {
            printf("link: DE1 back\n");
            gone = false;
            tape_ui_enter();
            tape_loader_reset();
        }
        link_status(&s, NULL);
        tape_t t = tape_classify(&s);
        if (t != ui.state) {
            ui.state = t;
            tape_enter_state(t);
        }
        if (t == T_LOADED) {
            printf("tape: %u/%u slots verified at field %ld\n", s.dt_have, s.dt_n, (long)tape_pos(&s));
            return true;
        }
        tape_picture(ui.state != T_OFF && ui.unlock == 0);
        pad();
        tape_auto_step(&s);
        ui.idle = inc(ui.idle);
        ui.key_age = inc(ui.key_age);
        ui.hit_age = inc(ui.hit_age);
        tape_draw(&s);
        link_present(video_get_framebuffer(), 100);
    }
}
