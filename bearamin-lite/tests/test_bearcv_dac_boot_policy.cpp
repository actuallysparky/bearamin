#include <assert.h>
#include <stdio.h>
#include "../firmware/common/bearcv_dac_boot_policy.h"
using namespace bearcv_boot;
struct Fake {
  AddressBits pair[2] = {{0, 0}, {1, 1}};
  int recoveryCalls = 0, reads = 0, writes = 0;
  int recoverFailures = 0, readFailures = 0, busyFailures = 0;
  uint8_t extraMask = 0, budget[2] = {kWriteBudget, kWriteBudget};
  bool dropAckOnce = false, rejectWrites = false, malformedScan = false;
  bool recover() { ++recoveryCalls; if (recoverFailures>0) {--recoverFailures;return false;} return true; }
  bool readPair(AddressBits *b, AddressBits *u) {
    ++reads; if (readFailures>0) {--readFailures;return false;}
    *b=pair[0]; *u=pair[1]; return true;
  }
  bool scan(uint8_t *mask) {
    *mask=(1u<<pair[0].input)|(1u<<pair[1].input)|extraMask;
    return !malformedScan;
  }
  bool waitReady(uint8_t) { if (busyFailures>0) {--busyFailures;return false;} return true; }
  bool writeAddress(bool uni, AddressBits current) {
    const uint8_t i=uni?1:0;
    assert(current.input==pair[i].input); // Includes volatile/EEPROM disagreement.
    if (!budget[i]) return false;
    --budget[i]; ++writes;
    if (rejectWrites) return false;
    pair[i]={i,i};
    if (dropAckOnce) {dropAckOnce=false;return false;}
    return true;
  }
};
int main() {
  unsigned cases=0;
  // All shared, distinct, unexpected and swapped active address combinations.
  for (uint8_t b=0;b<8;++b) for (uint8_t u=0;u<8;++u) {
    Fake f;f.pair[0]={b,b};f.pair[1]={u,u};
    auto r=ensureAddresses(f);assert(r==Result::kReady||r==Result::kRepaired);
    assert(addressMatches(f.pair[0],0)&&addressMatches(f.pair[1],1));
    const int writes=f.writes;assert(ensureAddresses(f)==Result::kReady);assert(f.writes==writes);++cases;
  }
  for (uint8_t b=0;b<8;++b) for (uint8_t u=0;u<8;++u) {
    Fake f;f.pair[0]={static_cast<uint8_t>(7-b),b};f.pair[1]={static_cast<uint8_t>(7-u),u};
    assert(ensureAddresses(f)==Result::kRepaired);++cases;
  }
  { Fake f;f.recoverFailures=2;assert(ensureAddresses(f)==Result::kReady);assert(f.recoveryCalls==3);++cases; }
  { Fake f;f.readFailures=3;assert(ensureAddresses(f)==Result::kFailed);assert(f.writes==0);++cases; }
  { Fake f;f.pair[1]={0,0};f.busyFailures=3;assert(ensureAddresses(f)==Result::kFailed);assert(f.writes==0);assert(ensureAddresses(f)==Result::kRepaired);++cases; }
  { Fake f;f.extraMask=4;assert(ensureAddresses(f)==Result::kAmbiguous);assert(f.writes==0);++cases; }
  { Fake f;f.malformedScan=true;assert(ensureAddresses(f)==Result::kFailed);assert(f.writes==0);++cases; }
  { Fake f;f.pair[1]={0,0};f.dropAckOnce=true;assert(ensureAddresses(f)==Result::kReady);assert(f.writes==1);++cases; }
  { Fake f;f.pair[1]={0,0};f.rejectWrites=true;assert(ensureAddresses(f)==Result::kFailed);assert(f.writes==3);assert(ensureAddresses(f)==Result::kFailed);assert(f.writes==3);++cases; }
  AddressBits decoded={};assert(!decodeSelectedAddress(0xff,&decoded));assert(!decodeSelectedAddress(0,&decoded));
  for (uint8_t ee=0;ee<8;++ee) for(uint8_t input=0;input<8;++input) {
    assert(decodeSelectedAddress((ee<<5)|0x10|(input<<1),&decoded));assert(decoded.eeprom==ee&&decoded.input==input);++cases;
  }
  uint8_t data[24]={};const uint16_t codes[4]={2048,2048,2048,3472};
  for(uint8_t ch=0;ch<4;++ch) for(uint8_t bank=0;bank<2;++bank) {
    const uint8_t i=ch*6+bank*3;data[i]=0x80|(ch<<4);data[i+1]=0x90|(codes[ch]>>8);data[i+2]=codes[ch]&255;
  }
  assert(safeEeprom(data,0,codes));assert(!safeEeprom(data,1,codes));
  for(uint8_t ch=0;ch<4;++ch) {
    data[ch*6+3]&=0x7f;assert(!safeEeprom(data,0,codes));data[ch*6+3]|=0x80;
    data[ch*6+4]^=0x10;assert(!safeEeprom(data,0,codes));data[ch*6+4]^=0x10;
    data[ch*6+5]^=1;assert(!safeEeprom(data,0,codes));data[ch*6+5]^=1;++cases;
  }
  assert(sweepCode(0)==0&&sweepCode(1)==2048&&sweepCode(2)==4095);
  assert(!sweepDue(2499,0)&&sweepDue(2500,0));
  assert(sweepDue(1499,UINT32_MAX-1000));
  printf("DAC policy: %u address/fault/EEPROM cases and dwell rollover PASS\n",cases);
}
