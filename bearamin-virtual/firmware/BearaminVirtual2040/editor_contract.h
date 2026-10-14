#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <initializer_list>
namespace virtual_editor {
constexpr uint32_t kPairingMs=120000;
constexpr size_t kMaxJson=4096, kMaxWire=4690;
struct Settings {
  int channel=1,rate=15,ccX=20,ccY=21,ccZ=22,ccPresence=23;
  int curveX=0,curveY=0,curveZ=0; // linear, gentle, steep
  bool invertX=false,invertY=false,invertZ=false,presenceEnabled=true,spankEnabled=true;
  int note=60,velocity=100,near=60,far=600,hold=250,cells=4;
  int spankSpeed=250,spankDepth=150,cooldown=300,rotation=0;
  bool mirror=false,displayEnabled=true;
  int brightness=50,smoothing=40;
};
inline bool valid(const Settings&s) {
  const int cc[]={s.ccX,s.ccY,s.ccZ,s.ccPresence};
  for(int v:cc)if(v<0||v>119)return false;
  for(int i=0;i<(s.presenceEnabled?4:3);++i)for(int j=0;j<i;++j)if(cc[i]==cc[j])return false;
  return s.channel>=1&&s.channel<=16&&s.rate==15&&s.curveX>=0&&s.curveX<=2&&s.curveY>=0&&s.curveY<=2&&s.curveZ>=0&&s.curveZ<=2&&s.note>=0&&s.note<=127&&s.velocity>=1&&s.velocity<=127&&s.near>=20&&s.near<=1000&&s.far>=50&&s.far<=1500&&s.near<s.far&&s.hold>=0&&s.hold<=1000&&s.cells>=1&&s.cells<=64&&s.spankSpeed>=50&&s.spankSpeed<=2000&&s.spankDepth>=20&&s.spankDepth<=500&&s.cooldown>=50&&s.cooldown<=2000&&(s.rotation==0||s.rotation==90||s.rotation==180||s.rotation==270)&&s.brightness>=0&&s.brightness<=100&&s.smoothing>=0&&s.smoothing<=500;
}
inline uint8_t mapAxis(uint16_t norm,int curve,bool invert) {
  float v=norm>4095?1.0f:norm/4095.0f;
  if(invert)v=1-v;
  if(curve==1)v=std::sqrt(v); else if(curve==2)v=v*v;
  return static_cast<uint8_t>(std::lround(v*127));
}
inline uint8_t brightnessRaw(int percent){return static_cast<uint8_t>((percent*32+50)/100);}
inline bool pairingOpen(uint32_t now,uint32_t start){return uint32_t(now-start)<kPairingMs;}
inline uint32_t crc32(const uint8_t*p,size_t n){uint32_t crc=~0u;while(n--){crc^=*p++;for(int i=0;i<8;++i)crc=(crc>>1)^((0u-(crc&1))&0xedb88320u);}return ~crc;}
// Incremental packed SysEx decoder; real-time bytes may interleave without cancelling it.
class Decoder {
  size_t prefix_=0,size_=0; uint8_t high_=0,group_=0;bool active_=false;
  uint32_t started_=0;
 public:
  char json[kMaxJson+1]={};uint32_t rejected=0;
  void reset(){prefix_=size_=group_=0;active_=false;}
  void expire(uint32_t now){if(active_&&uint32_t(now-started_)>1000){++rejected;reset();}}
  bool feed(uint8_t v,uint32_t now){
    expire(now);
    if(v>=0xf8)return false;
    if(v==0xf0){reset();active_=true;started_=now;return false;}
    if(!active_)return false;
    if(v==0xf7){bool ok=prefix_==4&&size_>0&&group_!=1;json[size_]=0;reset();if(!ok)++rejected;return ok;}
    if(v&0x80){++rejected;reset();return false;}
    static constexpr uint8_t prefix[]={0x7d,0x42,0x41,1};
    if(prefix_<4){if(v!=prefix[prefix_++]){++rejected;reset();}return false;}
    if(group_==0){high_=v;group_=1;return false;}
    if(size_>=kMaxJson){++rejected;reset();return false;}
    json[size_++]=static_cast<char>(v|(((high_>>(group_-1))&1)<<7));
    if(++group_==8)group_=0;
    return false;
  }
};
inline size_t encode(const char*json,size_t n,uint8_t*out,size_t cap){
  if(n==0||n>kMaxJson||cap<6+n+(n+6)/7)return 0;
  size_t w=0;for(uint8_t v:{uint8_t(0xf0),uint8_t(0x7d),uint8_t(0x42),uint8_t(0x41),uint8_t(1)})out[w++]=v;
  for(size_t i=0;i<n;i+=7){size_t h=w++;out[h]=0;for(size_t j=0;j<7&&i+j<n;++j){uint8_t v=static_cast<uint8_t>(json[i+j]);out[h]|=(v>>7)<<j;out[w++]=v&127;}}
  out[w++]=0xf7;return w;
}

} // namespace virtual_editor
