#!/usr/bin/env python3
"""Host exercise of the real bounded editor, mapping and two-slot storage."""
from pathlib import Path
import os,subprocess,tempfile
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as tmp:
 p=Path(tmp)
 for name,body in {
  'hardware/flash.h':'#pragma once\n#define FLASH_SECTOR_SIZE 4096\n#define FLASH_PAGE_SIZE 256\n#define XIP_BASE (reinterpret_cast<uintptr_t>(flashBytes))\nvoid flash_range_erase(unsigned,size_t);void flash_range_program(unsigned,const uint8_t*,size_t);\n',
  'hardware/watchdog.h':'#pragma once\ninline void watchdog_reboot(uint32_t,uint32_t,uint32_t){}\n',
  'mbedtls/sha256.h':'#pragma once\n#include <CommonCrypto/CommonDigest.h>\ninline int mbedtls_sha256_ret(const uint8_t*p,size_t n,uint8_t*out,int){CC_SHA256(p,n,out);return 0;}\n',
  'hardware/sync.h':'#pragma once\ninline uint32_t save_and_disable_interrupts(){return 0;}inline void restore_interrupts(uint32_t){}\n',
  'pico/unique_id.h':'#pragma once\ninline void pico_get_unique_board_id_string(char*p,size_t n){snprintf(p,n,"0011223344556677");}\n'
 }.items():
  q=p/name;q.parent.mkdir(parents=True,exist_ok=True);q.write_text(body)
 (p/'test.cpp').write_text(r'''
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <cassert>
#include <bearfield/bearfield.h>
#include "safety.h"
uint8_t flashBytes[2097152];bool brokenWrite=false;
void flash_range_erase(unsigned o,size_t n){memset(flashBytes+o,255,n);}
void flash_range_program(unsigned o,const uint8_t*p,size_t n){memcpy(flashBytes+o,p,brokenWrite?16:n);}
uint64_t time_us_64(){return 1000000;}
void tud_task(){}
void reset_usb_boot(uint32_t,uint32_t){}
bearamin_safety::SensorHealth health;
bearamin_safety::MidiOutput midiOutput;
bool allowTx=true;std::vector<uint8_t> tx;
bool tud_midi_packet_write(const uint8_t*p){if(!allowTx)return false;unsigned c=p[0]&15,n=c==4?3:c==5?1:c==6?2:3;tx.insert(tx.end(),p+1,p+1+n);return true;}
#define BEARAMIN_EDITOR_VERSION "0.3.0.0"
#include "editor_runtime.h"
JsonDocument command(const char*op,JsonObjectConst data){
 editorArena.used=0;JsonDocument req;req["op"]=op;req["request"]=42;req["uid"]=editorUid;req["data"]=data;serializeJson(req,editorDecoder.json,sizeof(editorDecoder.json));editorPending=true;editorHandle();
 if(editorPending){midiOutput.service(true,[](const uint8_t*){return true;});editorHandle();}
 virtual_editor::Decoder d;for(size_t i=0;i<editorLength;++i)d.feed(editorWire[i],100);JsonDocument response;assert(!deserializeJson(response,d.json));editorLength=editorOffset=0;return response;
}
int main(){
 memset(flashBytes,255,sizeof(flashBytes));editorLoad();assert(editorRevision==0&&config.channel==1&&config.rate==15&&config.smoothing==40);
 const char* inquiry="{\"op\":\"identity\",\"request\":99,\"data\":{\"protocol\":1}}";uint8_t rawWire[256];size_t wireN=virtual_editor::encode(inquiry,strlen(inquiry),rawWire,sizeof(rawWire));for(size_t i=0;i<wireN;++i){uint8_t packet[]={15,rawWire[i],0,0};editorPacket(packet);}assert(editorPending);editorHandle();assert(editorLength>0&&editorCin[15]==wireN);editorLength=editorOffset=0;
 JsonDocument data;data["protocol"]=1;auto identity=command("identity",data.as<JsonObjectConst>());assert(identity["data"]["version"]=="0.3.0.0"&&identity["data"]["capabilities"]["settings"]==true);
 auto read=command("settings.read",data.as<JsonObjectConst>());assert(read["data"]["settings"]["channel"]==1);
 JsonDocument write;write["revision"]=0;virtual_editor::Settings next;next.channel=16;next.ccX=80;virtual_editor::writeSettings(write["settings"].to<JsonObject>(),next);
 midiOutput.service(true,[](const uint8_t*){return true;});auto changed=command("settings.write",write.as<JsonObjectConst>());assert(changed["op"]=="settings.write"&&editorRevision==1&&config.channel==16&&config.ccX==80);
 auto stale=command("settings.write",write.as<JsonObjectConst>());assert(stale["op"]=="error"&&editorRevision==1);
 write["revision"]=1;write["settings"]["ccY"]=80;auto invalid=command("settings.write",write.as<JsonObjectConst>());assert(invalid["op"]=="error"&&editorRevision==1);
 write["settings"]["ccY"]=21;write["settings"]["channel"]=3;brokenWrite=true;auto interrupted=command("settings.write",write.as<JsonObjectConst>());assert(interrupted["op"]=="error");
 editorRevision=0;config={};editorLoad();assert(editorRevision==1&&config.channel==16); // prior slot survives interrupted save
 brokenWrite=false;auto saved=command("settings.write",write.as<JsonObjectConst>());assert(saved["op"]=="settings.write"&&editorRevision==2);editorRevision=0;config={};editorLoad();assert(config.channel==3&&editorRevision==2);
 JsonDocument unsupported;unsupported["hz"]=30;assert(command("field.subscribe",unsupported.as<JsonObjectConst>())["op"]=="error");
 // Live packetization obeys USB MIDI ending lengths, and honors backpressure.
 JsonDocument reply;reply["op"]="diagnostics.read";editorQueue(reply);assert(editorLength>0);size_t length=editorLength;allowTx=false;assert(editorSend()&&editorLength==length&&editorOffset==0);allowTx=true;while(editorLength)editorSend();virtual_editor::Decoder transmitted;for(auto b:tx)transmitted.feed(b,100);JsonDocument complete;assert(!deserializeJson(complete,transmitted.json)&&complete["op"]=="diagnostics.read");
 assert(virtual_editor::mapAxis(4095,0,true)==0);
 uint8_t cc[]={80,81,82,83};midiOutput.configure(2,cc,64);midiOutput.queue(0x92,64,120);std::vector<uint8_t> statuses;midiOutput.service(true,[&](const uint8_t*p){statuses.push_back(p[1]);return true;});assert(statuses.back()==0x92);midiOutput.silence();midiOutput.service(true,[&](const uint8_t*p){statuses.push_back(p[1]);return true;});assert(midiOutput.quiet());

 // Real update handler: bounds/ordering/readback/hash/ABI validation before intent.
 uint8_t image[512]={};uint32_t sp=0x20042000,pc=0x10004181;memcpy(image+256,&sp,4);memcpy(image+260,&pc,4);memcpy(image+350,virtual_update::kAbi,sizeof(virtual_update::kAbi));memcpy(image+400,"0.3.0.1",8);uint8_t hash[32];CC_SHA256(image,sizeof(image),hash);char hex[65];for(unsigned i=0;i<32;++i)snprintf(hex+i*2,3,"%02x",hash[i]);
 JsonDocument start;auto release=start["release"].to<JsonObject>();release["size"]=512;release["abi"]=1;release["address"]=0x10004000;release["target"]="bearamin-virtual-rp2040";release["hardware"]="waveshare-rp2040-zero";release["version"]="0.3.0.1";release["sha256"]=hex;
 auto begun=command("update.begin",start.as<JsonObjectConst>());assert(begun["op"]=="update.begin"&&midiUpdater.active);
 JsonDocument chunk;chunk["offset"]=256;auto list=chunk["bytes"].to<JsonArray>();for(unsigned i=0;i<256;++i)list.add(image[i]);assert(command("update.chunk",chunk.as<JsonObjectConst>())["op"]=="error");
 chunk["offset"]=0;assert(command("update.chunk",chunk.as<JsonObjectConst>())["op"]=="update.chunk");assert(command("update.chunk",chunk.as<JsonObjectConst>())["op"]=="update.chunk");assert(midiUpdater.offset==256);
 chunk["offset"]=256;list.clear();for(unsigned i=256;i<512;++i)list.add(image[i]);assert(command("update.chunk",chunk.as<JsonObjectConst>())["op"]=="update.chunk");JsonDocument finish;finish["sha256"]="bad";assert(command("update.finish",finish.as<JsonObjectConst>())["op"]=="error");assert(!virtual_update::valid(*reinterpret_cast<virtual_update::Record*>(flashBytes+virtual_update::kMeta)));
 finish["sha256"]=hex;assert(command("update.finish",finish.as<JsonObjectConst>())["op"]=="update.finish");assert(virtual_update::valid(*reinterpret_cast<virtual_update::Record*>(flashBytes+virtual_update::kMeta))&&midiUpdater.committed);

 // Simulate power loss after each erase and256-byte programming step.
 std::vector<uint8_t> replacement(20000,0x5a);memcpy(replacement.data()+256,&sp,4);pc=0x10004181;memcpy(replacement.data()+260,&pc,4);auto rec=*reinterpret_cast<virtual_update::Record*>(flashBytes+virtual_update::kMeta);rec.size=replacement.size();rec.imageCrc=virtual_update::crc(replacement.data(),replacement.size());rec.recordCrc=virtual_update::crc(reinterpret_cast<uint8_t*>(&rec),offsetof(virtual_update::Record,recordCrc));memcpy(flashBytes+virtual_update::kStage,replacement.data(),replacement.size());
 auto readFlash=[](uint32_t o){return flashBytes+o;};uint8_t sector[4096];const auto protectedHash=virtual_update::crc(flashBytes+virtual_update::kSettings,8192);
 for(int cut=0;cut<85;++cut){memset(flashBytes+virtual_update::kApp,0,replacement.size()+4096);int step=0;
  virtual_update::install(rec,readFlash,[&](uint32_t o,size_t n){assert(o>=virtual_update::kApp&&o+n<=virtual_update::kStage);memset(flashBytes+o,255,n);return step++!=cut;},[&](uint32_t o,const uint8_t*p,size_t n){for(size_t i=0;i<n;i+=256){memcpy(flashBytes+o+i,p+i,256);if(step++==cut)return false;}return true;},sector);
  assert(virtual_update::install(rec,readFlash,[](uint32_t o,size_t n){memset(flashBytes+o,255,n);return true;},[](uint32_t o,const uint8_t*p,size_t n){memcpy(flashBytes+o,p,n);return true;},sector));assert(!memcmp(flashBytes+virtual_update::kApp,replacement.data(),replacement.size()));assert(virtual_update::crc(flashBytes+virtual_update::kSettings,8192)==protectedHash);
 }
 puts("Virtual editor/update validation and85 interruption recoveries passed");
}
''')
 subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-I',str(p),'-I',str(root/'firmware/BearaminVirtual2040'),'-I',str(root/'third_party/ArduinoJson/src'),'-I',str(root/'third_party/bearfield-core/include'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
