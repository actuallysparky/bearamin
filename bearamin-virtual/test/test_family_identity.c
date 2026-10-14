#include "../firmware/BearaminVirtual2040/family_identity.h"
#include <assert.h>
static unsigned forwarded, sent;
static uint8_t output[20];
static void forward(const uint8_t *p) { (void)p; forwarded++; }
static void send(const uint8_t *p) { memcpy(output+sent*4,p,4); sent++; }
int main(void) {
    bearamin_identity_probe state={0};
    const uint8_t first[]={4,0xf0,0x7e,0x7f}, last[]={7,6,1,0xf7};
    const uint8_t note[]={9,0x90,60,100};
    assert(!bearamin_identity_accept(&state,note,forward));assert(forwarded==1);
    assert(!bearamin_identity_accept(&state,first,forward));assert(state.pending);
    assert(bearamin_identity_accept(&state,last,forward));assert(!state.pending);assert(forwarded==1);
    bearamin_identity_reply(3,send);assert(sent==5);assert(output[11]==3);assert(output[19]==0xf7);
    assert(!bearamin_identity_accept(&state,first,forward));
    assert(!bearamin_identity_accept(&state,note,forward));assert(forwarded==3);
    return 0;
}
