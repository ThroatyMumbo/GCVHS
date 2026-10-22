// N64 controllers on pio2, one SM per port, as a state machine the caller steps: no busy waits, so
// core 0 stays free for the VHS model between steps. A silent port is probed with 0x00 (identify)
// until it answers the N64 status; after that it is polled with 0x01.
#include "pad.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "onewire.pio.h"

#define POLL_US     16000               // once a frame
#define REPROBE_US  250000              // while nothing answers
#define REPLY_US    2000                // 4 bytes at 4 us per bit take ~130 us; allow for a slow pad

typedef struct {
    uint sm;
    bool waiting, connected;
    uint8_t want, got, reply[4];
    uint16_t buttons;
    uint32_t deadline, next;
    uint pin;
    uint8_t edges;
} port_t;

static PIO pio = pio2;
static uint offset;
static port_t ports[PAD_PORTS];

void pad_init(const unsigned *data_pins) {
    pio_set_gpio_base(pio, 16);         // controller pins are GP40/41, outside pio2's default window
    offset = pio_add_program(pio, &onewire_program);
    uint32_t now = time_us_32();
    for (int i = 0; i < PAD_PORTS; i++) {
        port_t *p = &ports[i];
        p->sm = pio_claim_unused_sm(pio, true);
        p->pin = data_pins[i];
        gpio_pull_up(p->pin);           // not the reset pull-down against the header's pull-up
        pio_sm_config c = onewire_program_get_default_config(offset);
        onewire_program_init(pio, p->sm, offset, data_pins[i], &c);
        p->next = now + 100000 + i * (REPROBE_US / PAD_PORTS);  // let the pads power up, then stagger
    }
}

// A reply that never finished leaves the SM waiting on the line; put it back at the program start.
static void restart(uint sm) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, sm, true);
}

// Bring-up aid: falling edges on the line over the next `us`; our request alone makes 9 (8 bits + stop).
static uint8_t count_edges(uint pin, uint32_t us) {
    uint32_t end = time_us_32() + us;
    uint8_t n = 0;
    bool last = gpio_get(pin);
    while ((int32_t)(time_us_32() - end) < 0) {
        bool v = gpio_get(pin);
        n += last && !v && n < 255;
        last = v;
    }
    return n;
}

// Step one port; true when it produced a poll reading in p->buttons.
static bool step(port_t *p, uint32_t now) {
    if (!p->waiting) {
        if ((int32_t)(now - p->next) < 0) return false;
        uint8_t cmd = p->connected ? 0x01 : 0x00;   // identify: some pads ignore reset (0xFF)
        p->want = p->connected ? 4 : 3;
        pio_sm_clear_fifos(pio, p->sm);
        pio_sm_put(pio, p->sm, (uint32_t)(p->want - 1) << 24);
        pio_sm_put(pio, p->sm, (uint32_t)cmd << 24);
        p->waiting = true;
        p->got = 0;
        p->deadline = now + REPLY_US;
        if (!p->connected) p->edges = count_edges(p->pin, 600);
        return false;
    }
    while (p->got < p->want && !pio_sm_is_rx_fifo_empty(pio, p->sm))
        p->reply[p->got++] = (uint8_t)pio_sm_get(pio, p->sm);
    if (p->got == p->want) {
        p->waiting = false;
        p->next = now + POLL_US;
        if (!p->connected) {
            p->connected = p->reply[0] == 0x05;
            p->buttons = 0;
            return false;
        }
        p->buttons = (uint16_t)(p->reply[0] << 8 | p->reply[1]);
        return true;
    }
    if ((int32_t)(now - p->deadline) >= 0) {
        restart(p->sm);
        p->waiting = p->connected = false;
        p->buttons = 0;
        p->next = now + REPROBE_US;
    }
    return false;
}

unsigned pad_edges(void) {
    unsigned e = 0;
    for (int i = 0; i < PAD_PORTS; i++)
        e |= (unsigned)ports[i].edges << (8 * i);
    return e;
}

bool pad_service(uint16_t *buttons, unsigned *connected) {
    uint32_t now = time_us_32();
    bool fresh = false;
    *buttons = 0;
    *connected = 0;
    for (int i = 0; i < PAD_PORTS; i++) {
        fresh |= step(&ports[i], now);
        *buttons |= ports[i].buttons;
        *connected |= (unsigned)ports[i].connected << i;
    }
    return fresh;
}
