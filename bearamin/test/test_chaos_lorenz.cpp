#include <cassert>

#include "../firmware/common/chaos_lorenz.h"

int main() {
  bearamin_chaos::StartupUnlock unlock;
  unlock.begin(100);
  for (uint8_t i = 0; i < 4; ++i) assert(!unlock.event(200 + i * 400));
  assert(unlock.event(1900));
  assert(unlock.latched());

  bearamin_chaos::StartupUnlock forced;
  forced.begin(0);
  forced.force();
  assert(forced.latched());

  bearamin_chaos::Lorenz a;
  bearamin_chaos::Lorenz b;
  a.seed(1234); b.seed(1234);
  for (int i = 0; i < 200; ++i) {
    const auto ax = a.tick(1.0f / 30.0f, 0.0f, 0.0f);
    const auto bx = b.tick(1.0f / 30.0f, 0.0f, 0.0f);
    assert(ax.x == bx.x && ax.y == bx.y && ax.z == bx.z);
    assert(ax.x <= 4095 && ax.y <= 4095 && ax.z <= 4095);
  }
}
