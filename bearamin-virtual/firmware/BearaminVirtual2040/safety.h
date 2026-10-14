#pragma once
#include <stdint.h>

namespace bearamin_safety {
constexpr uint32_t kChunkTimeoutUs = 10000; // 256 bytes at400kHz need about5.8ms.
constexpr uint32_t kInitBudgetUs = 5000000;
constexpr uint32_t kReadBudgetUs = 100000;
constexpr uint32_t kFreshFrameUs = 250000;
constexpr uint32_t kRetryUs = 2000000;

struct SensorHealth {
  bool ready = false;
  uint64_t retryAt = 0, freshUntil = 0;
  bool retryDue(uint64_t now) const { return !ready && now >= retryAt; }
  bool stale(uint64_t now) const { return ready && now >= freshUntil; }
  void started(uint64_t now) { ready = true; frame(now); }
  void frame(uint64_t now) { freshUntil = now + kFreshFrameUs; }
  void fault(uint64_t now) { ready = false; retryAt = now + kRetryUs; }
};

// Coalesce axes, retain failed safety transitions, and use atomic USB-MIDI
// packets. "Sent" means accepted into TinyUSB's FIFO, not acknowledged by a DAW.
class MidiOutput {
 public:
  bool quiet() const { return !sentNote_&&!wantNote_&&!dirty_[0]&&!dirty_[1]&&!dirty_[2]&&!dirty_[3]; }
  void configure(uint8_t channel,const uint8_t*cc,uint8_t note) {channel_=channel;for(unsigned i=0;i<4;++i)cc_[i]=cc[i];note_=note;}
  void disconnect() { connected_ = false; }
  void queue(uint8_t status, uint8_t data1, uint8_t data2) {
    if (!connected_) return; // Never replay gestures acquired while disconnected.
    if ((status & 0xf0) == 0xb0) {
      for(unsigned i=0;i<4;++i)if(data1==cc_[i]){values_[i]=data2;dirty_[i]=true;}
    } else if (data1 == note_ && ((status & 0xf0) == 0x80 || (status & 0xf0) == 0x90)) {
      wantNote_ = (status & 0xf0) == 0x90 && data2 != 0; velocity_ = data2;
    }
  }
  void silence() {
    for (unsigned i = 0; i < 4; ++i) { values_[i] = 0; dirty_[i] = true; }
    wantNote_ = false;
  }
  template<class Write>
  void service(bool mounted, Write write) {
    if (!mounted) { connected_ = false; return; }
    if (!connected_) {
      connected_ = true; cleanup_ = 0; sentNote_ = false; velocity_ = 100;
      silence();
    }
    // Every new USB session starts with a specific note-off and Presence0.
    while (cleanup_ < 2) {
      if (!packet(write, uint8_t((cleanup_ == 0 ? 0x80 : 0xb0)|channel_),
                  cleanup_ == 0 ? note_ : cc_[3], 0)) return;
      ++cleanup_;
    }
    if (sentNote_ && !wantNote_) {
      if (!packet(write, uint8_t(0x80|channel_), note_, 0)) return;
      sentNote_ = false;
    }
    if (dirty_[3]) { // Presence ahead of ordinary axes and new notes.
      if (!packet(write, uint8_t(0xb0|channel_), cc_[3], values_[3])) return;
      dirty_[3] = false;
    }
    for (unsigned i = 0; i < 3; ++i) {
      if (!dirty_[i]) continue;
      if (!packet(write, uint8_t(0xb0|channel_), cc_[i], values_[i])) return;
      dirty_[i] = false;
    }
    if (wantNote_ && !sentNote_) {
      if (!packet(write, uint8_t(0x90|channel_), note_, velocity_)) return;
      sentNote_ = true;
    }
  }
 private:
  template<class Write>
  static bool packet(Write write, uint8_t status, uint8_t data1, uint8_t data2) {
    const uint8_t p[4] = {static_cast<uint8_t>(status >> 4), status, data1, data2};
    return write(p);
  }
  uint8_t channel_=0,cc_[4]={20,21,22,23},note_=60;
  bool connected_ = false, wantNote_ = false, sentNote_ = false;
  uint8_t cleanup_ = 0, velocity_ = 100, values_[4] = {};
  bool dirty_[4] = {};
};
}
