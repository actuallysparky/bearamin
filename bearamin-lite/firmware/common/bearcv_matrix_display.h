#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// MatrixCore's continuous vector/tail and depth palette, adapted for Lite's
// local pixels and DAC/sensor status overlay. No CV mapping is applied here.
namespace bearcv_display {
struct Color { uint8_t red=0, green=0, blue=0; };
// Scale once before writing RGB bytes. Retain one visible level for nonzero
// low-intensity occupancy pixels; enabled output never exceeds the 32/255 cap.
inline uint8_t scaleChannel(uint8_t value, uint8_t brightness) {
  const uint8_t cap = std::min<uint8_t>(brightness, 32);
  if (!value || !cap) return 0;
  return std::max<uint8_t>(1, (uint16_t(value) * cap + 127) / 255);
}
class Renderer {
  bool havePrevious_=false;
  uint16_t previousX_=500, previousY_=500;
  static uint16_t expand(uint16_t v) {
    const int32_t x=500+(int32_t(std::min<uint16_t>(v,1000))-500)*17/10;
    return uint16_t(std::max<int32_t>(0,std::min<int32_t>(1000,x)));
  }
  static Color depthColor(uint8_t brightness,uint16_t depth,uint16_t z) {
    uint32_t near=0;
    if(depth && depth<600) {
      if(depth<=75) near=1000;
      else { const uint32_t v=(600-depth)*1000UL/525;
        near=(v*v*(3000-2*v)+500000UL)/1000000UL; }
    }
    const uint16_t delta=z>2048?z-2048:2048-z;
    const uint16_t marker=delta<123?((123-delta)*1000UL+61)/123:0;
    const uint16_t base=1000-marker;
    const uint32_t channels[3]={
      ((220*near+500)/1000*base+220*marker+500)/1000,
      ((220*near+500)/1000*base+500)/1000,
      ((220*(1000-near)+500)/1000*base+255*marker+500)/1000};
    return {uint8_t(channels[0]*brightness/220),
            uint8_t(channels[1]*brightness/220),
            uint8_t(channels[2]*brightness/220)};
  }
 public:
  Color pixels[64]={};
  void add(uint8_t x,uint8_t y,Color c) {
    Color &p=pixels[y*8+x];
    p.red=uint8_t(std::min<int>(255,int(p.red)+c.red));
    p.green=uint8_t(std::min<int>(255,int(p.green)+c.green));
    p.blue=uint8_t(std::min<int>(255,int(p.blue)+c.blue));
  }
  void render(const uint16_t *mm,uint64_t valid,bool ready,bool active,
              uint16_t xNorm,uint16_t yNorm,uint16_t zNorm,uint16_t depth,
              bool spank) {
    for(auto &p:pixels)p={};
    if(!ready){havePrevious_=false;return;}
    for(uint8_t i=0;i<64;++i)
      if((valid&(UINT64_C(1)<<i)) && mm[i]>0 && mm[i]<=500)
        pixels[i]={6,6,6};
    const uint16_t x=expand(active?(uint32_t(xNorm)*1000+2047)/4095:500);
    const uint16_t y=expand(active?(uint32_t(yNorm)*1000+2047)/4095:500);
    const float dx=havePrevious_?(int32_t(x)-previousX_)/1000.0f:0;
    const float dy=havePrevious_?(int32_t(y)-previousY_)/1000.0f:0;
    previousX_=x;previousY_=y;havePrevious_=true;
    const float speed=std::sqrt(dx*dx+dy*dy);
    const float amount=std::min(1.0f,speed/0.050f);
    const float inv=speed>0.0001f?1.0f/speed:0;
    const float vx=dx*inv,vy=dy*inv;
    const float length=0.45f+amount*4.2f,width=0.42f+amount*0.18f;
    const uint8_t head=active?220:72,tail=active?48:18;
    for(uint8_t row=0;row<8;++row)for(uint8_t col=0;col<8;++col) {
      const float rx=col-x*7/1000.0f,ry=row-y*7/1000.0f;
      const float distance=std::sqrt(rx*rx+ry*ry);
      float intensity=distance<0.75f?1.0f-distance/0.75f:0;
      if(speed>0.0001f) {
        const float behind=-(rx*vx+ry*vy),side=std::fabs(rx*-vy+ry*vx);
        if(behind>=0 && behind<=length && side<=width) {
          const float along=1-behind/length,lateral=1-side/width;
          intensity=std::max(intensity,along*along*lateral*(0.30f+0.70f*amount));
        }
      }
      if(intensity>0.06f)
        add(col,row,depthColor(uint8_t(tail+intensity*(head-tail)),active?depth:600,active?zNorm:2048));
    }
    if(!active) add(uint8_t((x*7UL+500)/1000),uint8_t((y*7UL+500)/1000),depthColor(72,600,2048));
    if(spank)for(uint8_t i=0;i<8;++i){add(i,i,{110,36,0});add(7-i,i,{110,36,0});}
  }
};
} // namespace bearcv_display
