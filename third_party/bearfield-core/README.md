# Bearfield Core

Portable, hardware-independent 8x8 optical-field processing shared by
Bearamin and BearCV.

The public boundary is `include/bearfield/bearfield.h`. Sensor adapters must
provide genuine 8x8 frames at 15 Hz in canonical instrument coordinates.
`Processor::tick()` services interpolated product output between real frames;
it never creates another optical measurement or repeats an edge-triggered
gesture.

Hardware drivers, web services, audio, display wiring, and DAC code belong to
product repositories. Product repositories pin an exact Bearfield Core commit.

Run `./test.sh` for the portable host suite.
