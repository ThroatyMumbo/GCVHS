// GameConsoleV2's input/onewire.c (same header and onewire.pio), except that each transfer restarts its SM:
// the PIO receive loop has no timeout, so one unanswered request would otherwise wedge the bus for good.
#include "input/onewire.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "onewire.pio.h"

static PIO ow_pio = pio2;
static uint ow_sm[ONEWIRE_MAX_BUSES];
static uint ow_pin[ONEWIRE_MAX_BUSES];
static uint ow_offset = 0;
static bool ow_loaded = false;

bool onewire_init(uint8_t id, uint8_t data_pin) {
    if (id >= ONEWIRE_MAX_BUSES)
        return false;

    if (!ow_loaded) {
        pio_set_gpio_base(ow_pio, 16);  // GP40/41 are outside pio2's default window
        if (!pio_can_add_program(ow_pio, &onewire_program))
            return false;
        ow_offset = pio_add_program(ow_pio, &onewire_program);
        ow_loaded = true;
    }

    ow_sm[id] = pio_claim_unused_sm(ow_pio, true);
    ow_pin[id] = data_pin;

    pio_sm_config c = onewire_program_get_default_config(ow_offset);
    onewire_program_init(ow_pio, ow_sm[id], ow_offset, data_pin, &c);
    return true;
}

uint8_t onewire_transfer(uint8_t id, const uint8_t *request, uint8_t request_len, uint8_t *response,
                         uint8_t response_len) {
    if (id >= ONEWIRE_MAX_BUSES)
        return 0;
    uint sm = ow_sm[id];
    pio_sm_set_enabled(ow_pio, sm, false);
    pio_sm_clear_fifos(ow_pio, sm);
    pio_sm_restart(ow_pio, sm);
    pio_sm_set_consecutive_pindirs(ow_pio, sm, ow_pin[id], 1, false);   // release the line
    pio_sm_exec(ow_pio, sm, pio_encode_jmp(ow_offset));
    pio_sm_set_enabled(ow_pio, sm, true);

    pio_sm_put_blocking(ow_pio, sm, ((response_len - 1) & 0x1F) << 24);  // response bytes - 1
    for (int i = 0; i < request_len; i++)
        pio_sm_put_blocking(ow_pio, sm, (uint32_t)request[i] << 24);

    // A silent byte reads as 0 but isn't counted, so callers can tell an answer of 0 from no answer.
    uint8_t received = 0;
    for (int i = 0; i < response_len; i++) {
        absolute_time_t timeout = make_timeout_time_us(600);
        bool timed_out = false;
        while (pio_sm_is_rx_fifo_empty(ow_pio, sm) && !timed_out)
            timed_out = time_reached(timeout);
        if (timed_out) {
            response[i] = 0;
            continue;
        }
        response[i] = (uint8_t)(pio_sm_get(ow_pio, sm) & 0xFF);
        received++;
    }

    busy_wait_us(4 * (request_len + response_len) + 450);   // bus idle before the next transaction
    return received;
}
