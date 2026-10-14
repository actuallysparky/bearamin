#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
namespace bearcv_chaos {
struct Axes { uint16_t x=2048,y=2048,z=2048; };
class Unlock { public: void begin(uint32_t t){start=t;} bool on(uint32_t t){if(lock||t-start>5000)return false; if(n&&t-first>2500)n=0; if(!n)first=t; if(++n==5)return lock=true; return false;} bool active()const{return lock;} private:uint32_t start=0,first=0;uint8_t n=0;bool lock=false;};
class Lorenz { public: void seed(uint32_t s){x=-8+16*((s&1023)+1)/1025.0f;y=-7+14*(((s>>10)&1023)+1)/1025.0f;z=20+12*(((s>>20)&1023)+1)/1025.0f;} Axes tick(float sec,float control,float influence){control=std::max(0.f,std::min(1.f,control)); influence=std::max(-1.f,std::min(1.f,influence)); const float dt=sec*.65f/(5+85*control)/4; for(int i=0;i<4;i++) step(dt,influence); return {norm(x/24),norm(y/32),norm((z-25)/28)};} private: static uint16_t norm(float v){v=std::max(-1.f,std::min(1.f,v));return static_cast<uint16_t>(std::lround((v+1)*2047.5f));} void step(float d,float q){auto f=[q](float a,float b,float c,float&o,float&p,float&r){o=10*(b-a);p=a*(28+1.25f*q-c)-b;r=a*b-(8.f/3)*c;};float a,b,c,e,fv,g,h,i,j,k,l,m;f(x,y,z,a,b,c);f(x+a*d/2,y+b*d/2,z+c*d/2,e,fv,g);f(x+e*d/2,y+fv*d/2,z+g*d/2,h,i,j);f(x+h*d,y+i*d,z+j*d,k,l,m);x+=d*(a+2*e+2*h+k)/6;y+=d*(b+2*fv+2*i+l)/6;z+=d*(c+2*g+2*j+m)/6;} float x=.1f,y=0,z=25;};
}
