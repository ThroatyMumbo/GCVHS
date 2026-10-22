# GCVHS - Prototype VHS-based Game System

This is firmware for the DE1-SoC and PGA2350 which turns them into a makeshift VHS-based game console.

Demonstration video: https://youtu.be/vcgEQVsysYM

At a high level, bytecode for the game + multiple video streams are encoded into a single composite
signal which can then be recorded to a VHS tape and played back into the DE1's composite input. The
PGA2350 runs the code decoded from the tape and composites the video stream(s) onto the window.

The composite stream also carries timing information which can be used to accurately seek forward or
backward through the tape via the IR blaster on the DE1 (or an external IR LED).

## Contents

| Path | Desc |
|---|---|
| `chipset/` | Quartus project for the DE1-SoC: the console's video/audio chipset. It digitizes composite through the ADV7180, files the interleaved streams in SDRAM, outputs the selected one as 480p VGA with the game's overlay on top, plays the in-line audio, and decodes the data tape, the tape position (LTC, VITC, position tags) and IR. `sim/` holds the Questa testbenches. |
| `cpu/` | Pico SDK firmware for the game CPU (RP2350-based): the BIOS, which drives the deck over IR, loads the game off the tape and runs it, and the link to the chipset. `roms/cinema` is a demo game that plays one clip per theater. |
| `vcr_sim/` | Pico SDK firmware for a GameConsoleV2 board that stands in for the VCR and tape: it renders the tagged multi-stream composite signal, with LTC on the audio, and emulates the deck's transport from its IR remote codes. |
| `scripts/` | Bench helpers: build and flash each board, load the FPGA, control the VCR sim, write SD tapes and master a tape onto a cassette. |
| `tools/` | Python host side: bakes the data tape, clips and SD images, protocol tables, timecode encoders, testbench stimulus, and JTAG readers for the chipset. |
| `external/GameConsoleV2` | Submodule: the game engine, cart SDK and hardware the console and VCR sim build on. |

The cinema demo needs your own videos: put 8 clips named `1` to `8` (any format ffmpeg reads) in
`videos/` before baking.

## Wiring (DE1 → PGA2350)

| CPU pin | DE1 signal | Header pin | Direction | Signal |
|---|---|---|---|---|
| GP2 | GPIO_0[0] | 1 | CPU → DE1 | DN strobe |
| GP3 | GPIO_0[1] | 2 | CPU → DE1 | DN data 0 |
| GP4 | GPIO_0[2] | 3 | CPU → DE1 | DN data 1 |
| GP5 | GPIO_0[3] | 4 | CPU → DE1 | DN data 2 |
| GP6 | GPIO_0[4] | 5 | CPU → DE1 | DN data 3 |
| GP7 | GPIO_0[5] | 6 | CPU → DE1 | DN data 4 |
| GP8 | GPIO_0[6] | 7 | CPU → DE1 | DN data 5 |
| GP9 | GPIO_0[7] | 8 | CPU → DE1 | DN data 6 |
| GP10 | GPIO_0[8] | 9 | CPU → DE1 | DN data 7 |
| GP11 | GPIO_0[9] | 10 | DE1 → CPU | UP strobe |
| GP12 | GPIO_0[10] | 13 | DE1 → CPU | UP data 0 |
| GP13 | GPIO_0[11] | 14 | DE1 → CPU | UP data 1 |
| GP14 | GPIO_0[12] | 15 | DE1 → CPU | UP data 2 |
| GP15 | GPIO_0[13] | 16 | DE1 → CPU | UP data 3 |
| GP16 | GPIO_0[14] | 17 | DE1 → CPU | RDY |
| GP17 | GPIO_0[15] | 18 | DE1 → CPU | VBLANK |
