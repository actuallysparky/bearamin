#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#ifndef BEARAMIN_ID_V0
#define BEARAMIN_ID_V0 0
#define BEARAMIN_ID_V1 2
#define BEARAMIN_ID_V2 0
#define BEARAMIN_ID_V3 1
#endif
#if BEARAMIN_ID_V0 < 0 || BEARAMIN_ID_V0 > 127 || BEARAMIN_ID_V1 < 0 || BEARAMIN_ID_V1 > 127 || BEARAMIN_ID_V2 < 0 || BEARAMIN_ID_V2 > 127 || BEARAMIN_ID_V3 < 0 || BEARAMIN_ID_V3 > 127
#error MIDI identity version components must fit seven bits
#endif
// Bearamin family MIDI Identity Reply: development ID 7D, family 2141,
// model 2 = Virtual, 3 = T-Dongle. Version 0.2.0.1. No settings writes.
typedef struct { bool pending; uint8_t first[4]; } bearamin_identity_probe;
static inline bool bearamin_identity_accept(bearamin_identity_probe *state,
    const uint8_t packet[4], void (*forward)(const uint8_t *)) {
    static const uint8_t start[4] = {4, 0xf0, 0x7e, 0x7f};
    static const uint8_t end[4] = {7, 6, 1, 0xf7};
    if (state->pending && packet[1] >= 0xf8) {
        if (forward) forward(packet);
        return false;
    }
    if (state->pending) {
        state->pending = false;
        if (!memcmp(packet, end, 4)) return true;
        if (forward) forward(state->first);
    }
    if (!memcmp(packet, start, 4)) {
        memcpy(state->first, packet, 4); state->pending = true;
    } else if (forward) forward(packet);
    return false;
}
static inline void bearamin_identity_reply(uint8_t model, void (*send)(const uint8_t *)) {
    const uint8_t packets[5][4] = {
        {4,0xf0,0x7e,0x7f}, {4,6,2,0x7d}, {4,0x41,0x42,model},
        {4,0,BEARAMIN_ID_V0,BEARAMIN_ID_V1}, {7,BEARAMIN_ID_V2,BEARAMIN_ID_V3,0xf7}
    };
    for (unsigned i=0;i<5;++i) send(packets[i]);
}
