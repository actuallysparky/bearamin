#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "update_layout.h"
#include <string.h>
namespace u=virtual_update;
alignas(4) static uint8_t sector[4096];
int main(){
 const auto read=[](uint32_t o){return reinterpret_cast<const uint8_t*>(XIP_BASE+o);};
 const auto&r=*reinterpret_cast<const u::Record*>(read(u::kMeta));
 if(u::valid(r)){
  if(!u::install(r,read,[](uint32_t o,size_t n){auto irq=save_and_disable_interrupts();flash_range_erase(o,n);restore_interrupts(irq);return true;},[](uint32_t o,const uint8_t*p,size_t n){auto irq=save_and_disable_interrupts();flash_range_program(o,p,n);restore_interrupts(irq);return true;},sector))reset_usb_boot(0,0);
 }
 const uint8_t*app=read(u::kApp);
 if(!u::vectors(app,u::kMax)||!u::marker(app,u::kMax,u::kAbi,sizeof(u::kAbi)))reset_usb_boot(0,0);
 const uint32_t*vector=reinterpret_cast<const uint32_t*>(app+256);
 // The loader does not start USB, DMA or core1. Restore the reset IRQ mask.
 save_and_disable_interrupts();
 *reinterpret_cast<volatile uint32_t*>(0xe000e180)=0xffffffff;
 *reinterpret_cast<volatile uint32_t*>(0xe000e280)=0xffffffff;
 *reinterpret_cast<volatile uint32_t*>(0xe000e010)=0;
 *reinterpret_cast<volatile uint32_t*>(0xe000ed08)=XIP_BASE+u::kApp+256;
 __asm volatile("msr msp, %0\n bx %1"::"r"(vector[0]),"r"(vector[1]):"memory");
 __builtin_unreachable();
}
