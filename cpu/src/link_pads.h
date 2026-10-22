// Link pad drive, shared by the link driver and link_test: 2, 4, 8 or 12 mA, fast or slow slew.
#ifndef LINK_PADS_H
#define LINK_PADS_H

#include <stdbool.h>
#include "hardware/gpio.h"

static inline void link_pads_drive(uint first, uint n, unsigned ma, bool fast) {
    enum gpio_drive_strength ds = ma >= 12 ? GPIO_DRIVE_STRENGTH_12MA : ma >= 8 ? GPIO_DRIVE_STRENGTH_8MA
                                : ma >= 4 ? GPIO_DRIVE_STRENGTH_4MA : GPIO_DRIVE_STRENGTH_2MA;
    for (uint p = first; p < first + n; p++) {
        gpio_set_drive_strength(p, ds);
        gpio_set_slew_rate(p, fast ? GPIO_SLEW_RATE_FAST : GPIO_SLEW_RATE_SLOW);
    }
}

#endif
