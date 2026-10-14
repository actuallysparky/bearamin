#pragma once
#include "update_layout.h"
#include "mbedtls/sha256.h"
#include "hardware/watchdog.h"
struct MidiUpdater {
 bool active=false,committed=false;uint32_t length=0,offset=0,lastAt=0,restartAt=0,recoveryAt=0;
 uint8_t hash[32];char version[24];
 bool begin(JsonObjectConst r){
  if(committed)return false;
  if(!r["size"].is<uint32_t>()||r["size"].as<uint32_t>()<512||r["size"].as<uint32_t>()>virtual_update::kMax||r["abi"].as<int>()!=1||r["address"].as<uint32_t>()!=0x10004000||strcmp(r["target"]|"","bearamin-virtual-rp2040")||strcmp(r["hardware"]|"","waveshare-rp2040-zero"))return false;
  const char*h=r["sha256"]|"",*v=r["version"]|"";if(strlen(h)!=64||strlen(v)>=sizeof(version))return false;
  unsigned a[4],b[4];int end=0;if(sscanf(v,"%u.%u.%u.%u%n",a,a+1,a+2,a+3,&end)!=4||v[end]||sscanf(BEARAMIN_EDITOR_VERSION,"%u.%u.%u.%u",b,b+1,b+2,b+3)!=4)return false;
  bool newer=false;for(unsigned i=0;i<4;++i){if(a[i]>127)return false;if(!newer){if(a[i]<b[i])return false;if(a[i]>b[i])newer=true;}}
  for(unsigned i=0;i<32;++i){unsigned byte;for(unsigned j=0;j<2;++j)if(!((h[i*2+j]>='0'&&h[i*2+j]<='9')||(h[i*2+j]>='a'&&h[i*2+j]<='f')))return false;sscanf(h+i*2,"%2x",&byte);hash[i]=byte;}
  length=r["size"];offset=0;strcpy(version,v);committed=false;restartAt=0;lastAt=time_us_64()/1000;
  // Invalidate installation intent before touching the stage. Active app is intact.
  auto irq=save_and_disable_interrupts();flash_range_erase(virtual_update::kMeta,4096);restore_interrupts(irq);
  for(uint32_t n=0;n<length;n+=4096){irq=save_and_disable_interrupts();flash_range_erase(virtual_update::kStage+n,4096);restore_interrupts(irq);tud_task();}
  active=true;return true;
 }
 bool chunk(JsonObjectConst data){
  auto bytes=data["bytes"].as<JsonArrayConst>();if(!active||committed||!data["offset"].is<uint32_t>()||!bytes.size()||bytes.size()>256)return false;uint32_t o=data["offset"];if(o%256||o>length||bytes.size()!=std::min<uint32_t>(256,length-o))return false;
  alignas(4) uint8_t page[256];memset(page,255,sizeof(page));unsigned i=0;for(auto b:bytes){if(!b.is<unsigned>()||b.as<unsigned>()>255)return false;page[i++]=b.as<unsigned>();}
  // A repeated acknowledged chunk is safe and cannot advance the offset twice.
  if(o!=offset){return o+bytes.size()==offset&&!memcmp(reinterpret_cast<const void*>(XIP_BASE+virtual_update::kStage+o),page,bytes.size());}
  auto irq=save_and_disable_interrupts();flash_range_program(virtual_update::kStage+o,page,256);restore_interrupts(irq);
  if(memcmp(reinterpret_cast<const void*>(XIP_BASE+virtual_update::kStage+o),page,256))return false;offset+=bytes.size();lastAt=time_us_64()/1000;return true;
 }
 bool finish(const char*claim){
  if(!active||committed||offset!=length)return false;char expected[65];for(unsigned i=0;i<32;++i)snprintf(expected+i*2,3,"%02x",hash[i]);if(strcmp(expected,claim))return false;
  const auto*stage=reinterpret_cast<const uint8_t*>(XIP_BASE+virtual_update::kStage);uint8_t actual[32];if(mbedtls_sha256_ret(stage,length,actual,0)||memcmp(actual,hash,32)||!virtual_update::vectors(stage,length)||!virtual_update::marker(stage,length,virtual_update::kAbi,sizeof(virtual_update::kAbi))||!virtual_update::marker(stage,length,version,strlen(version)+1))return false;
  virtual_update::Record r{};r.magic=0x42565550;r.abi=1;r.size=length;r.imageCrc=virtual_update::crc(stage,length);memcpy(r.sha256,hash,32);r.recordCrc=virtual_update::crc(reinterpret_cast<const uint8_t*>(&r),offsetof(virtual_update::Record,recordCrc));alignas(4) uint8_t page[256];memset(page,255,sizeof(page));memcpy(page,&r,sizeof(r));auto irq=save_and_disable_interrupts();flash_range_program(virtual_update::kMeta,page,sizeof(page));restore_interrupts(irq);if(memcmp(reinterpret_cast<const void*>(XIP_BASE+virtual_update::kMeta),&r,sizeof(r)))return false;
  committed=true;restartAt=uint32_t(time_us_64()/1000)+500;return true;
 }
 void poll(){uint32_t now=time_us_64()/1000;if(recoveryAt&&int32_t(now-recoveryAt)>=0)reset_usb_boot(0,0);else if(committed&&int32_t(now-restartAt)>=0)watchdog_reboot(0,0,0);else if(active&&!committed&&now-lastAt>120000)active=false;}
} midiUpdater;
