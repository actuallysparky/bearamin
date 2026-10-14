#pragma once
#include <stdint.h>
#include <stddef.h>
namespace virtual_update {
constexpr uint32_t kApp=0x4000,kStage=0x100000,kMeta=0x1fd000,kSettings=0x1fe000;
constexpr uint32_t kMax=kStage-kApp;
constexpr char kAbi[]="BEARAMIN-RP2040-MIDI-ABI1";
struct Record{uint32_t magic,abi,size,imageCrc;uint8_t sha256[32];uint32_t recordCrc;};
inline uint32_t crc(const uint8_t*p,size_t n){uint32_t c=~0u;while(n--){c^=*p++;for(unsigned i=0;i<8;++i)c=(c>>1)^((0u-(c&1))&0xedb88320u);}return ~c;}
inline bool valid(const Record&r){return r.magic==0x42565550&&r.abi==1&&r.size>=512&&r.size<=kMax&&r.recordCrc==crc(reinterpret_cast<const uint8_t*>(&r),offsetof(Record,recordCrc));}
inline bool vectors(const uint8_t*p,size_t n){if(n<512)return false;uint32_t sp,pc;__builtin_memcpy(&sp,p+256,4);__builtin_memcpy(&pc,p+260,4);return sp>=0x20000000&&sp<=0x20042000&&(sp&3)==0&&(pc&1)&&pc>=0x10000000+kApp+256&&pc<0x10000000+kApp+n;}
inline bool marker(const uint8_t*p,size_t n,const char*s,size_t len){for(size_t i=0;i+len<=n;++i)if(!__builtin_memcmp(p+i,s,len))return true;return false;}
// The loader always recopies a verified stage if the active copy is incomplete.
// No operation touches the resident loader, stage, metadata or settings sectors.
template<class Read,class Erase,class Program>
bool install(const Record&r,Read read,Erase erase,Program program,uint8_t*buffer){
 if(!valid(r))return false;
 if(crc(read(kApp),r.size)==r.imageCrc&&vectors(read(kApp),r.size))return true;
 if(crc(read(kStage),r.size)!=r.imageCrc||!vectors(read(kStage),r.size))return false;
 for(uint32_t offset=0;offset<r.size;offset+=4096){uint32_t n=r.size-offset;if(n>4096)n=4096;__builtin_memset(buffer,255,4096);__builtin_memcpy(buffer,read(kStage+offset),n);if(!erase(kApp+offset,4096)||!program(kApp+offset,buffer,4096))return false;}
 return crc(read(kApp),r.size)==r.imageCrc;
}
}
