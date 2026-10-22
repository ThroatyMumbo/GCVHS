// The BIOS screen: waits for the game tape, works the deck from the controller, reads the tape's state off STATUS.
#ifndef TAPE_UI_H
#define TAPE_UI_H

#include <stdbool.h>

// Takes the DE1 (OWN, overlay, preview window). Call again after a game returns.
void tape_ui_enter(void);

// Runs until the DE1 holds a verified image (true), or poll(), called every frame, returns true (false).
// `error` (or NULL) is shown first, as after a failed load.
bool tape_ui_run(const char *error, const char *detail, bool (*poll)(void));

// One static screen, for while the image is read over the link.
void tape_ui_show(const char *title, const char *detail);

#endif
