#include "../firmware/BearaminVirtual2040/gesture_processing.h"
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
using virtual_editor::Settings;

bearfield::DepthFrame8x8 depth(unsigned mm, unsigned count=4) {
  bearfield::DepthFrame8x8 frame{};
  for (unsigned i=0;i<count;++i) {
    unsigned cell=17+i;
    frame.distanceMm[cell]=mm;
    frame.validMask|=uint64_t{1}<<cell;
  }
  return frame;
}
bearfield::ProcessedFieldState sample(const Settings& settings, unsigned mm) {
  bearfield::Processor processor(virtual_editor::feelProfile(settings));
  auto frame=virtual_editor::orient(depth(mm),settings);
  bearfield::ProcessedFieldState field{};
  for (unsigned i=0;i<30;++i) field=processor.process(frame,1000000+i*66667);
  return field;
}
int main(int argc,char** argv) {
  Settings settings;settings.smoothing=0;
  auto profile=virtual_editor::feelProfile(settings);
  assert(profile.minimumDepthMm==60 && profile.playableMaximumMm==600);
  auto middle=sample(settings,300);
  assert(middle.present);
  auto normal=virtual_editor::gesture(middle,settings);
  settings.mirror=true;
  auto mirrored=virtual_editor::gesture(sample(settings,300),settings);
  assert(std::abs(int(normal.x)+int(mirrored.x)-4095)<=1);
  settings.mirror=false;
  for (int rotation:{0,90,180,270}) {
    settings.rotation=rotation;
    auto raw=depth(300);auto oriented=virtual_editor::orient(raw,settings);
    assert(__builtin_popcountll(oriented.validMask)==4);
    if(rotation)assert(raw.validMask!=oriented.validMask);
  }
  settings.rotation=0;
  settings.near=150;assert(sample(settings,300).zNorm12>middle.zNorm12);
  settings.near=60;settings.far=900;assert(sample(settings,300).zNorm12>middle.zNorm12);
  settings.far=600;settings.cells=5;assert(!sample(settings,300).present);
  settings.cells=4;
  for(int hold:{0,500}) {
    settings.hold=hold;
    bearfield::Processor processor(virtual_editor::feelProfile(settings));
    processor.process(depth(300),1000000);
    assert(processor.process(depth(0,0),1100000).present==(hold==500));
  }
  settings.hold=250;
  for (int smoothing:{0,500}) {
    settings.smoothing=smoothing;
    bearfield::Processor processor(virtual_editor::feelProfile(settings));
    auto first=processor.process(depth(500),1000000);
    auto next=processor.process(depth(100),1066667);
    assert(next.present);
    if(smoothing==0)assert(next.zNorm12==sample(settings,100).zNorm12);
    else assert(next.zNorm12<sample(Settings{},100).zNorm12);
    (void)first;
  }
  settings.smoothing=0;
  for(int speed:{50,2000}) {
    settings.spankSpeed=speed;
    bearfield::Processor processor(virtual_editor::feelProfile(settings));
    processor.process(depth(160),1000000);
    assert(processor.process(depth(120),1100000).spankHeld==(speed==50));
  }
  settings.spankSpeed=50;
  for(int limit:{100,200}) {
    settings.spankDepth=limit;
    bearfield::Processor processor(virtual_editor::feelProfile(settings));
    processor.process(depth(240),1000000);
    assert(processor.process(depth(120),1100000).spankHeld==(limit==200));
  }
  settings.spankDepth=150;
  for(int cooldown:{50,2000}) {
    settings.cooldown=cooldown;
    bearfield::Processor processor(virtual_editor::feelProfile(settings));
    processor.process(depth(240),1000000);
    assert(processor.process(depth(120),1100000).spankHeld);
    assert(!processor.process(depth(240),1200000).spankHeld);
    assert(processor.process(depth(120),1300000).spankHeld==(cooldown==50));
  }
  bearfield::ProcessedFieldState field{};
  field.present=true;field.spankHeld=true;field.xMilli=250;field.yMilli=500;field.xNorm12=1024;field.yNorm12=2048;field.zNorm12=3072;
  for(int axis=0;axis<3;++axis)for(int curve=0;curve<3;++curve)for(bool invert:{false,true}) {
    Settings cfg;int* curves[]={&cfg.curveX,&cfg.curveY,&cfg.curveZ};
    bool* inversions[]={&cfg.invertX,&cfg.invertY,&cfg.invertZ};
    *curves[axis]=curve;*inversions[axis]=invert;
    auto gesture=virtual_editor::gesture(field,cfg);
    unsigned norms[]={field.xNorm12,field.yNorm12,field.zNorm12};
    float expected=norms[axis]/4095.f;if(invert)expected=1-expected;
    if(curve==1)expected=std::sqrt(expected);if(curve==2)expected*=expected;
    unsigned axes[]={gesture.x,gesture.y,gesture.z};
    assert(std::abs(int(axes[axis])-int(std::lround(expected*4095)))<=1);
    std::vector<std::array<unsigned,3>> events;bool held=false;
    virtual_editor::performanceMidi(field,cfg,held,[&](unsigned a,unsigned b,unsigned c){events.push_back({a,b,c});});
    assert(events[axis][2]==unsigned(std::lround(expected*127)));
  }
  for(bool presence:{false,true})for(bool spank:{false,true}) {
    Settings cfg;cfg.presenceEnabled=presence;cfg.spankEnabled=spank;
    cfg.channel=16;cfg.ccX=80;cfg.ccY=81;cfg.ccZ=82;cfg.ccPresence=83;cfg.note=64;cfg.velocity=90;
    auto gesture=virtual_editor::gesture(field,cfg);
    assert(gesture.present && gesture.presence==presence && gesture.spank==spank);
    assert(gesture.centroidX==250 && gesture.centroidY==500);
    std::vector<std::array<unsigned,3>> events;bool held=false;
    virtual_editor::performanceMidi(field,cfg,held,[&](unsigned a,unsigned b,unsigned c){events.push_back({a,b,c});});
    assert(events[0][0]==0xbf && events[0][1]==80);
    assert(events[1][1]==81 && events[2][1]==82);
    assert(events.size()==3+unsigned(presence)+unsigned(spank));
    if(presence)assert(events[3][1]==83 && events[3][2]==127);
    if(spank)assert((events.back()==std::array<unsigned,3>{0x9f,64,90}));
    field.spankHeld=false;
    virtual_editor::performanceMidi(field,cfg,held,[&](unsigned a,unsigned b,unsigned c){events.push_back({a,b,c});});
    if(spank)assert((events.back()==std::array<unsigned,3>{0x8f,64,0}));
    field.spankHeld=true;
  }
  // Emit real firmware wire frames for consumption by the actual Rack modules.
  if(argc==2) {
    std::ofstream output(argv[1]);
    int16_t mm[64];uint8_t status[64];
    for(unsigned i=0;i<64;++i){mm[i]=300;status[i]=5;}
    unsigned cases=0;
    for(bool present:{false,true})for(int rotation:{0,90,180,270})for(bool mirror:{false,true})
      for(int curve:{0,1,2})for(bool invert:{false,true})
        for(bool presence:{false,true})for(bool spank:{false,true}) {
          field.present=present;
          Settings cfg;cfg.rotation=rotation;cfg.mirror=mirror;
          cfg.near=100;cfg.far=rotation==0?300:900;
          cfg.curveX=cfg.curveY=cfg.curveZ=curve;cfg.invertX=cfg.invertY=cfg.invertZ=invert;
          cfg.presenceEnabled=presence;cfg.spankEnabled=spank;
          auto gesture=virtual_editor::gesture(field,cfg);
          assert(gesture.nearMm==cfg.near && gesture.farMm==cfg.far);
          char wire[1024];auto n=bearamin_virtual::frameLine(wire,sizeof(wire),1,++cases,1000000,mm,status,&gesture);
          assert(n>0 && n<1024);output.write(wire,n);
        }
    assert(cases==384);
  }
  puts("Gesture settings, MIDI routing, gates, high-resolution curves and 384 Rack fixtures passed");
}
