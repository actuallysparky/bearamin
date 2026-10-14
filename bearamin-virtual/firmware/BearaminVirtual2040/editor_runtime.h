#pragma once
#include "editor_settings_json.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/unique_id.h"
#include <cstdlib>
// One bounded JSON arena, reused only after documents leave scope. No heap growth.
struct EditorArena : ArduinoJson::Allocator {
  alignas(max_align_t) uint8_t bytes[24576]; size_t used=0;
  void* allocate(size_t n) override {size_t total=(n+sizeof(size_t)+15)&~size_t(15);if(total>sizeof(bytes)-used)return nullptr;auto p=bytes+used;used+=total;*reinterpret_cast<size_t*>(p)=n;return p+sizeof(size_t);}
  void deallocate(void*) override {}
  void* reallocate(void*p,size_t n) override {if(!p)return allocate(n);void*q=allocate(n);if(q)memcpy(q,p,std::min(n,*reinterpret_cast<size_t*>(static_cast<uint8_t*>(p)-sizeof(size_t))));return q;}
} editorArena;
virtual_editor::Settings config;
virtual_editor::Decoder editorDecoder;
uint8_t editorWire[virtual_editor::kMaxWire];size_t editorLength=0,editorOffset=0;
char editorJson[virtual_editor::kMaxJson+1],editorUid[17];
bool editorPending=false,editorApply=false,editorSaving=false;
uint32_t editorParseErrors=0,editorCin[16]={};
uint32_t editorRevision=0,previewUntil=0,previewAt=0,editorFrames=0;
constexpr uint32_t kSettingsOffset=2*1024*1024-2*FLASH_SECTOR_SIZE;
struct EditorRecord {uint32_t magic,schema,revision;virtual_editor::Settings settings;uint32_t crc;};
bool recordValid(const EditorRecord&r){return r.magic==0x42565254&&r.schema==1&&r.revision&&r.crc==virtual_editor::crc32(reinterpret_cast<const uint8_t*>(&r),offsetof(EditorRecord,crc))&&virtual_editor::valid(r.settings);}
void editorLoad(){
 pico_get_unique_board_id_string(editorUid,sizeof(editorUid));
 for(unsigned i=0;i<2;++i){const auto&r=*reinterpret_cast<const EditorRecord*>(XIP_BASE+kSettingsOffset+i*FLASH_SECTOR_SIZE);if(recordValid(r)&&r.revision>editorRevision){config=r.settings;editorRevision=r.revision;}}
}
bool editorPersist(const virtual_editor::Settings&s){
 if(editorRevision==UINT32_MAX)return false;
 alignas(4) static uint8_t page[FLASH_PAGE_SIZE];memset(page,0xff,sizeof(page));EditorRecord r{};r.magic=0x42565254;r.schema=1;r.revision=editorRevision+1;r.settings=s;r.crc=virtual_editor::crc32(reinterpret_cast<const uint8_t*>(&r),offsetof(EditorRecord,crc));static_assert(sizeof(r)<=FLASH_PAGE_SIZE);memcpy(page,&r,sizeof(r));
 const uint32_t offset=kSettingsOffset+(r.revision%2)*FLASH_SECTOR_SIZE;
 const uint32_t irq=save_and_disable_interrupts();flash_range_erase(offset,FLASH_SECTOR_SIZE);flash_range_program(offset,page,sizeof(page));restore_interrupts(irq);
 return !memcmp(reinterpret_cast<const void*>(XIP_BASE+offset),&r,sizeof(r));
}
#include "midi_update.h"
void editorQueue(JsonDocument&doc){if(doc.overflowed())return;size_t n=measureJson(doc);if(n>virtual_editor::kMaxJson)return;serializeJson(doc,editorJson,sizeof(editorJson));editorLength=virtual_editor::encode(editorJson,n,editorWire,sizeof(editorWire));editorOffset=0;}
void editorError(JsonDocument&d,const char*s){d["op"]="error";d["error"]=s;d.remove("data");}
void editorHandle(){
 if(!editorPending||editorLength)return;editorPending=false;editorArena.used=0;
 JsonDocument req(&editorArena),reply(&editorArena);
 if(deserializeJson(req,editorDecoder.json,DeserializationOption::NestingLimit(6))||!req["request"].is<uint32_t>()||!req["op"].is<const char*>()){++editorParseErrors;return;}
 const char*op=req["op"];if(strlen(op)>40)return;
 if(strcmp(op,"identity")&&(!req["uid"].is<const char*>()||strcmp(req["uid"],editorUid)))return;
 reply["op"]=op;reply["request"]=req["request"];reply["uid"]=editorUid;auto data=reply["data"].to<JsonObject>();
 if(!strcmp(op,"identity")){
  if(!req["data"]["protocol"].is<int>()||req["data"]["protocol"].as<int>()!=1)editorError(reply,"Unsupported protocol.");
  else {data["protocol"]=1;data["uid"]=editorUid;data["model"]="virtual";data["hardware"]="waveshare-rp2040-zero";data["version"]=BEARAMIN_EDITOR_VERSION;data["build"]=0;data["transport"]="USB MIDI";auto c=data["capabilities"].to<JsonObject>();c["settings"]=true;c["field"]=true;c["diagnostics"]=true;c["display"]=false;c["ble"]=false;c["update"]="sysex";c["bootSel"]=true;data["updateAbi"]=virtual_update::kAbi;}
 }else if(!strncmp(op,"update.",7)){
  if(!strcmp(op,"update.bootsel")){midiUpdater.recoveryAt=uint32_t(time_us_64()/1000)+500;data["ok"]=true;}
  else if(!midiUpdater.active&&strcmp(op,"update.begin"))editorError(reply,"Start the update again.");
  else if(!strcmp(op,"update.begin")){
   if(!editorSaving){editorSaving=true;midiOutput.silence();previewUntil=0;}
   if(!midiOutput.quiet()){editorPending=true;return;}
   if(!midiUpdater.begin(req["data"]["release"].as<JsonObjectConst>()))editorError(reply,"This firmware is not compatible with your Bearamin.");
   else {health.fault(time_us_64());data["chunkSize"]=256;data["offset"]=0;}
   editorSaving=false;
  }else if(!strcmp(op,"update.chunk")){if(!midiUpdater.chunk(req["data"].as<JsonObjectConst>()))editorError(reply,"Update chunk could not be verified. Start again.");else data["offset"]=midiUpdater.offset;}
  else if(!strcmp(op,"update.finish")){if(!midiUpdater.finish(req["data"]["sha256"]|""))editorError(reply,"Firmware verification failed. Start the update again.");else data["ok"]=true;}
  else if(!strcmp(op,"update.abort")){if(!midiUpdater.committed)midiUpdater.active=false;data["ok"]=true;}
  else editorError(reply,"This update operation is unavailable.");
 }else if(midiUpdater.active&&strcmp(op,"diagnostics.read"))editorError(reply,"Finish the update before changing settings.");
 else if(!strcmp(op,"settings.read")){virtual_editor::writeSettings(data["settings"].to<JsonObject>(),config);data["revision"]=editorRevision;}
 else if(!strcmp(op,"settings.write")){
  virtual_editor::Settings next;
  if(!req["data"]["revision"].is<uint32_t>()||req["data"]["revision"].as<uint32_t>()!=editorRevision)editorError(reply,"Settings changed. Reconnect and try again.");
  else if(!virtual_editor::readSettings(req["data"]["settings"].as<JsonObjectConst>(),next))editorError(reply,"Check your settings and try again.");
  else {
   if(!editorSaving){editorSaving=true;midiOutput.silence();}
   if(!midiOutput.quiet()){editorPending=true;return;}
   if(!editorPersist(next))editorError(reply,"Settings could not be saved.");
   else {config=next;++editorRevision;editorApply=true;data["revision"]=editorRevision;}
   editorSaving=false;
  }
 }else if(!strcmp(op,"diagnostics.read")){data["sensor"]=health.ready?"Ready":"Not receiving";data["sensorHz"]=health.ready?15:0;data["sourceFrames"]=editorFrames;data["uptimeMs"]=uint32_t(time_us_64()/1000);data["rejectedMessages"]=editorDecoder.rejected;data["parseErrors"]=editorParseErrors;data["updateOffset"]=midiUpdater.offset;auto counters=data["inputPackets"].to<JsonArray>();for(auto n:editorCin)counters.add(n);}
 else if(!strcmp(op,"field.subscribe")){if(!req["data"]["hz"].is<int>()||(req["data"]["hz"].as<int>()!=0&&req["data"]["hz"].as<int>()!=5))editorError(reply,"Choose a supported preview rate.");else {previewUntil=req["data"]["hz"].as<int>()?uint32_t(time_us_64()/1000)+300000:0;data["hz"]=req["data"]["hz"];}}
 else editorError(reply,"This operation is unavailable on Virtual.");
 editorQueue(reply);
}
void editorPacket(const uint8_t*p){unsigned cin=p[0]&15;++editorCin[cin];unsigned n=cin==15?1:cin==4?3:cin==5?1:cin==6?2:cin==7?3:0;if(!editorPending)for(unsigned i=0;i<n;++i)if(editorDecoder.feed(p[i+1],uint32_t(time_us_64()/1000)))editorPending=true;}
void editorPreview(const bearfield::DepthFrame8x8&depth,const bearfield::ProcessedFieldState&field){
 ++editorFrames;uint32_t now=time_us_64()/1000;if(!previewUntil||int32_t(previewUntil-now)<=0||now-previewAt<200||editorLength||editorPending)return;previewAt=now;editorArena.used=0;JsonDocument d(&editorArena);d["op"]="field";d["uid"]=editorUid;auto a=d["data"].to<JsonObject>();auto mm=a["mm"].to<JsonArray>();for(unsigned i=0;i<64;++i){if((depth.validMask>>i)&1)mm.add(depth.distanceMm[i]);else mm.add(nullptr);}a["x"]=virtual_editor::mapAxis(field.xNorm12,config.curveX,config.invertX);a["y"]=virtual_editor::mapAxis(field.yNorm12,config.curveY,config.invertY);a["z"]=virtual_editor::mapAxis(field.zNorm12,config.curveZ,config.invertZ);a["presence"]=field.present;a["spank"]=field.spankHeld;editorQueue(d);
}
bool editorSend(){
 for(unsigned i=0;i<16&&editorLength;++i){size_t n=std::min<size_t>(3,editorLength-editorOffset);uint8_t p[4]={uint8_t(editorOffset+n==editorLength?4+n:4),0,0,0};memcpy(p+1,editorWire+editorOffset,n);if(!tud_midi_packet_write(p))return true;editorOffset+=n;if(editorOffset==editorLength)editorLength=editorOffset=0;}return editorLength!=0;
}
