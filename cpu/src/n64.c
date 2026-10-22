// GameConsoleV2's input/n64.c (same header), except a pad is found by identify (0x00), not reset (0xFF):
// some pads ignore reset, so they were never found once the console had booted without them. Reset follows, to recentre.
#include "input/n64.h"
#include "input/onewire.h"
#include "pico/stdlib.h"

// Status command (0x00 identify or 0xFF reset); true if an N64 controller answered (status byte 0x05).
static bool n64_status(uint8_t id, uint8_t cmd) {
    uint8_t req[1] = {cmd};
    uint8_t resp[3] = {0};
    uint8_t got = onewire_transfer(id, req, sizeof(req), resp, sizeof(resp));
    return got >= 1 && resp[0] == 0x05;
}

bool n64_controller_probe(uint8_t id) {
    if (!n64_status(id, 0x00))
        return false;
    n64_status(id, 0xFF);               // the stick's centre is wherever it rests now, not at power-up
    return true;
}

bool n64_controller_init(uint8_t id, uint8_t data_pin) {
    if (!onewire_init(id, data_pin))
        return false;
    sleep_ms(100);                      // controller power-up
    return n64_controller_probe(id);
}

bool n64_controller_read(uint8_t id, n64_controller_data_t *data) {
    if (!data)
        return false;
    uint8_t req[1] = {0x01};            // poll: buttons[15:8], buttons[7:0], stick x, stick y
    uint8_t resp[4] = {0};
    if (onewire_transfer(id, req, sizeof(req), resp, sizeof(resp)) < sizeof(resp))
        return false;                   // went silent: not a valid all-released reading
    data->buttons = ((uint16_t)resp[0] << 8) | resp[1];
    data->stick_x = (int8_t)resp[2];
    data->stick_y = (int8_t)resp[3];
    return true;
}
