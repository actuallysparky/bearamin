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
  'pico/unique_id.h':'#pragma once\ninline void pico_get_unique_board_id_string(char*p,size_t n){snprintf(p,n,"1852513411091616");}\n'
 }.items():
  q=p/name;q.parent.mkdir(parents=True,exist_ok=True);q.write_text(body)
 (p/'test.cpp').write_text(r'''
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <array>
#include <cassert>
#include <iostream>
#include <bearfield/bearfield.h>
#include "safety.h"
uint8_t flashBytes[2097152];bool brokenWrite=false;
void flash_range_erase(unsigned o,size_t n){memset(flashBytes+o,255,n);}
void flash_range_program(unsigned o,const uint8_t*p,size_t n){memcpy(flashBytes+o,p,brokenWrite?16:n);}
uint64_t fakeUs=1000000;
uint64_t time_us_64(){return fakeUs;}
void tud_task(){}
void reset_usb_boot(uint32_t,uint32_t){}
bearamin_safety::SensorHealth health;
bearamin_safety::MidiOutput midiOutput;
bool allowTx=true;int txBudget=100000;std::vector<uint8_t> tx;std::vector<std::array<uint8_t,4>> packets;
bool tud_midi_packet_write(const uint8_t*p){if(!allowTx||txBudget<=0)return false;--txBudget;packets.push_back({p[0],p[1],p[2],p[3]});unsigned c=p[0]&15,n=c==4?3:c==5?1:c==6?2:3;tx.insert(tx.end(),p+1,p+1+n);return true;}
#define BEARAMIN_EDITOR_VERSION "0.3.0.0"
#include "editor_runtime.h"
#include "gesture_processing.h"
JsonDocument command(const char*op,JsonObjectConst data){
 editorArena.used=0;JsonDocument req;req["op"]=op;req["request"]=42;req["uid"]=editorUid;req["data"]=data;serializeJson(req,editorDecoder.json,sizeof(editorDecoder.json));editorPending=true;editorHandle();
 if(editorPending){midiOutput.service(true,[](const uint8_t*){return true;});editorHandle();}
 virtual_editor::Decoder d;for(size_t i=0;i<editorLength;++i)d.feed(editorWire[i],100);JsonDocument response;assert(!deserializeJson(response,d.json));editorLength=editorOffset=0;return response;
}
int main(){
 memset(flashBytes,255,sizeof(flashBytes));editorLoad();assert(editorRevision==0&&config.channel==1&&config.rate==15&&config.smoothing==40);
 const char* inquiry="{\"op\":\"identity\",\"request\":99,\"data\":{\"protocol\":1}}";uint8_t rawWire[256];size_t wireN=virtual_editor::encode(inquiry,strlen(inquiry),rawWire,sizeof(rawWire));for(size_t i=0;i<wireN;++i){uint8_t packet[]={15,rawWire[i],0,0};editorPacket(packet);}assert(editorPending);editorHandle();assert(editorLength>0&&editorCin[15]==wireN);editorLength=editorOffset=0;
 JsonDocument data;data["protocol"]=1;auto identity=command("identity",data.as<JsonObjectConst>());assert(identity["data"]["version"]=="0.3.0.0"&&identity["data"]["capabilities"]["settings"]==true);assert(identity["data"]["capabilities"]["fieldRates"].size()==1&&identity["data"]["capabilities"]["fieldRates"][0]==15);
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


 // The advertised rates follow real sensor frames, including a slightly fast
 // sensor clock. Renewal preserves phase; expiry and uint32 wrap fail closed.
 virtual_editor::PreviewCadence cadence;
 for(double hz:{5.,15.}){cadence.subscribe(hz,1000);unsigned count=0;for(unsigned i=0;i<120;++i)count+=cadence.frame(1000+i*66);assert(count==120&&cadence.hz()==15);}
 cadence.subscribe(15,1000);assert(cadence.frame(1000));cadence.subscribe(5,1001);assert(cadence.frame(1066));assert(cadence.frame(1132));assert(!cadence.active(301001));assert(!cadence.supported(7.5));
 cadence.subscribe(15,UINT32_MAX-100);assert(cadence.active(100));assert(cadence.frame(100));cadence.subscribe(0,100);assert(!cadence.frame(101));
 for(double hz:{0.,5.,15.}){JsonDocument sub;sub["hz"]=hz;assert(command("field.subscribe",sub.as<JsonObjectConst>())["op"]=="field.subscribe");}
 JsonDocument legacy;legacy["hz"]=5;assert(command("field.subscribe",legacy.as<JsonObjectConst>())["data"]["hz"]==15);legacy["hz"]=7.5;assert(command("field.subscribe",legacy.as<JsonObjectConst>())["op"]=="error");JsonDocument bad;bad["hz"]="15";assert(command("field.subscribe",bad.as<JsonObjectConst>())["op"]=="error");
 JsonDocument sub;sub["hz"]=15;command("field.subscribe",sub.as<JsonObjectConst>());
 bearfield::DepthFrame8x8 depth{};depth.validMask=1;depth.distanceMm[0]=300;depth.sequence=100;depth.capturedUs=fakeUs;
 bearfield::ProcessedFieldState field{};field.present=true;field.zNorm12=2000;
 editorPreview(depth,field);depth.sequence=101;editorPreview(depth,field);assert(previewPending&&previewDropped>0);editorFlushPreview();
 virtual_editor::Decoder snap;for(size_t i=0;i<editorLength;++i)snap.feed(editorWire[i],100);JsonDocument snapshot;assert(!deserializeJson(snapshot,snap.json)&&snapshot["data"]["seq"]==101);
 assert(snapshot["data"]["near"]==config.near && snapshot["data"]["far"]==config.far);
 // At a message boundary performance MIDI precedes preview, even under a
 // one-packet FIFO budget. A started SysEx is never interleaved/truncated.
 tx.clear();packets.clear();midiOutput.queue(0xb2,82,70);txBudget=1;editorService(true);assert(packets.size()==1&&packets[0][1]==0xb2&&packets[0][2]==82&&packets[0][3]==70);assert(editorOffset==0);
 txBudget=16;editorService(true);assert(editorOffset>0);size_t sysexStart=packets.size();midiOutput.queue(0x92,64,100);
 while(editorLength){txBudget=16;editorService(true);}
 for(size_t i=sysexStart;i<packets.size();++i)assert((packets[i][0]&15)>=4&&(packets[i][0]&15)<=7);
 assert(midiOutput.pending());txBudget=100000;editorService(true);assert(packets.back()[1]==0x92);assert(!midiOutput.pending()); // held notes do not block previews
 depth.sequence=102;editorPreview(depth,field);fakeUs+=300001;editorFlushPreview();assert(!editorLength&&!previewPending);fakeUs-=300001;
 depth.capturedUs=fakeUs;editorPreview(depth,field);editorFlushPreview();assert(editorIsPreview&&editorOffset==0);JsonDocument empty;command("diagnostics.read",empty.as<JsonObjectConst>());assert(!editorIsPreview); // reply displaces an unstarted preview
 midiOutput.silence();midiOutput.service(true,[](const uint8_t*){return true;});editorStopPreview();
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

 // Round-trip every Field Guide preference through the real settings.write,
 // readback and flash-slot reload. Rate is a fixed 15 Hz Virtual capability;
 // display fields are retained for schema compatibility, hidden on Virtual.
 midiUpdater={};editorSaving=false;editorStopPreview(); // application restart after update tests
 JsonDocument alternatives;
 auto values=alternatives.to<JsonObject>();
 values["channel"]=2;values["rate"]=15;values["ccX"]=80;values["ccY"]=81;
 values["ccZ"]=82;values["ccPresence"]=83;values["note"]=64;values["velocity"]=80;
 values["near"]=100;values["far"]=900;values["hold"]=500;values["cells"]=8;
 values["spankSpeed"]=400;values["spankDepth"]=200;values["cooldown"]=600;
 values["brightness"]=25;values["smoothing"]=80;values["rotation"]="90";
 for(const char* key:{"curveX","curveY","curveZ"})values[key]="gentle";
 for(const char* key:{"invertX","invertY","invertZ","mirror"})values[key]=true;
 for(const char* key:{"presenceEnabled","spankEnabled","displayEnabled"})values[key]=false;
 assert(values.size()==28);
 for(JsonPair preference:values){
  JsonDocument input;input["revision"]=editorRevision;
  virtual_editor::Settings defaults;virtual_editor::writeSettings(input["settings"].to<JsonObject>(),defaults);
  input["settings"][preference.key()]=preference.value();
  auto response=command("settings.write",input.as<JsonObjectConst>());
  if(response["op"]!="settings.write"){fprintf(stderr,"Preference failed: %s\n",preference.key().c_str());serializeJson(response,std::cerr);}
  assert(response["op"]=="settings.write");
  auto back=command("settings.read",JsonObjectConst());
  std::string wanted,actual;serializeJson(input["settings"],wanted);serializeJson(back["data"]["settings"],actual);assert(wanted==actual);
  editorRevision=0;config={};editorLoad();
  auto rebooted=command("settings.read",JsonObjectConst());actual="";serializeJson(rebooted["data"]["settings"],actual);assert(wanted==actual);
 }

 // Exercise each numeric boundary and the neighboring invalid values through
 // the real handler. Coupled Near/Far limits use a valid enclosing interval.
 struct Limit {const char* key;int low,high;};
 for(auto limit:std::initializer_list<Limit>{{"channel",1,16},{"rate",15,15},{"ccX",0,119},{"ccY",0,119},{"ccZ",0,119},{"ccPresence",0,119},{"note",0,127},{"velocity",1,127},{"near",20,1000},{"far",50,1500},{"hold",0,1000},{"cells",1,64},{"spankSpeed",50,2000},{"spankDepth",20,500},{"cooldown",50,2000},{"brightness",0,100},{"smoothing",0,500}}) {
  for(int value:{limit.low-1,limit.low,limit.high,limit.high+1}) {
   JsonDocument input;input["revision"]=editorRevision;
   virtual_editor::Settings defaults;defaults.near=20;defaults.far=1500;
   virtual_editor::writeSettings(input["settings"].to<JsonObject>(),defaults);
   input["settings"][limit.key]=value;auto revision=editorRevision;
   auto response=command("settings.write",input.as<JsonObjectConst>());
   bool valid=value>=limit.low&&value<=limit.high;
   assert((response["op"]=="settings.write")==valid);
   assert(editorRevision==revision+unsigned(valid));
  }
 }
 for(const char* key:{"curveX","curveY","curveZ","rotation"}) {
  std::vector<const char*> options=strcmp(key,"rotation")?std::vector<const char*>{"linear","gentle","steep","invalid"}:std::vector<const char*>{"0","90","180","270","45"};
  for(unsigned i=0;i<options.size();++i){
   JsonDocument input;input["revision"]=editorRevision;virtual_editor::Settings defaults;
   virtual_editor::writeSettings(input["settings"].to<JsonObject>(),defaults);input["settings"][key]=options[i];
   assert((command("settings.write",input.as<JsonObjectConst>())["op"]=="settings.write")==bool(i+1<options.size()));
  }
 }
 // Invalid types/values must never advance the flash revision.
 for(JsonPair preference:values){
  JsonDocument input;input["revision"]=editorRevision;
  virtual_editor::Settings defaults;virtual_editor::writeSettings(input["settings"].to<JsonObject>(),defaults);
  input["settings"][preference.key()]=nullptr;
  auto revision=editorRevision;assert(command("settings.write",input.as<JsonObjectConst>())["op"]=="error");assert(editorRevision==revision);
 }
 puts("All 28 settings: save, readback, reboot and invalid-type rejection passed");
 puts("Virtual editor/update validation and85 interruption recoveries passed");
}
''')
 subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-I',str(p),'-I',str(root/'firmware/BearaminVirtual2040'),'-I',str(root/'third_party/ArduinoJson/src'),'-I',str(root/'third_party/bearfield-core/include'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
