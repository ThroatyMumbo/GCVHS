// N64 controllers on the GCV2 ports, polled without blocking (onewire.pio from GameConsoleV2).
#ifndef PAD_H
#define PAD_H

#include <stdbool.h>
#include <stdint.h>

// N64 poll response, buttons[15:0] as sent (A is bit 15).
#define PAD_A      (1u << 15)
#define PAD_B      (1u << 14)
#define PAD_Z      (1u << 13)
#define PAD_START  (1u << 12)
#define PAD_DUP    (1u << 11)
#define PAD_DDOWN  (1u << 10)
#define PAD_DLEFT  (1u << 9)
#define PAD_DRIGHT (1u << 8)
#define PAD_L      (1u << 5)
#define PAD_R      (1u << 4)
#define PAD_CUP    (1u << 3)
#define PAD_CDOWN  (1u << 2)
#define PAD_CLEFT  (1u << 1)
#define PAD_CRIGHT (1u << 0)

#define PAD_PORTS 2

void pad_init(const unsigned *data_pins);

// Advance the polls; true when a port produced a new reading. *buttons is every connected port's
// buttons ORed, *connected a bitmask of ports that answer.
bool pad_service(uint16_t *buttons, unsigned *connected);

// Falling edges seen on each port's line during its last probe, one byte per port.
unsigned pad_edges(void);

#endif
