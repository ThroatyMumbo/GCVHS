# (WIP) GCVHS

This is firmware for the DE1-SoC and PGA2350 which turns them into a makeshift VHS-based game console.

At a high level, bytecode for the game + multiple video streams are encoded into a single composite
signal which can then be recorded to a VHS tape and played back into the DE1's composite input. The
PGA2350 runs the code decoded from the tape and composites the video stream(s) onto the window.

The composite stream also carries timing information which can be used to accurately seek forward or
backward through the tape via the IR blaster on the DE1 (or an external IR LED).

## Disclaimer

This repo will be temporarily empty while I get the firmware code in a presentable state. Hopefully
done sometime this week.
