#pragma once
namespace lite_runtime {
using lite::Settings;
USBMIDI midi("Bearamin Lite");
Settings config;
Preferences preferences;
uint32_t revision=0;
char uid[32]={},digest[65]={};
bearfield::DepthFrame8x8 lastCanonical={};
uint32_t lastMidiMs=0,frameCount=0,resetAfterMs=0,rateEpoch=0,rateFrames=0;
float measuredHz=0;
std::atomic<uint32_t> pairStart{0},bleEpoch{1};
std::atomic<uint16_t> bleHandle{BLE_HS_CONN_HANDLE_NONE},bleMtu{23};
std::atomic<bool> bleSecure{false},bleSubscribed{false},admittedBond{false},advDirty{false},rxOverflow{false};
NimBLEServer* server=nullptr;
NimBLECharacteristic* characteristic=nullptr;
QueueHandle_t bleRx=nullptr;
struct RxChunk{uint32_t epoch;uint16_t n;uint8_t bytes[244];};
struct Endpoint:lite::PerformanceState {
  lite::Decoder decoder;
  uint8_t reply[lite::kMaxWire]={};size_t length=0,offset=0;
  uint32_t subscribeUntil=0,lastField=0,epoch=0;
  void clear(){decoder.reset();length=offset=0;events.clear();noteHeld=presence=panic=false;panicMask=0;subscribeUntil=0;for(auto&v:sent)v=255;}
};
Endpoint usb,ble;
using lite::writeSettings;using lite::readSettings;
struct Record{uint32_t magic,schema,generation;Settings settings;uint32_t crc;};
bool readRecord(const char*key,Record&r){return preferences.getBytesLength(key)==sizeof(r)&&preferences.getBytes(key,&r,sizeof(r))==sizeof(r)&&r.magic==0x42414c54&&r.schema==1&&r.generation>0&&r.crc==lite::crc32(reinterpret_cast<const uint8_t*>(&r),offsetof(Record,crc))&&lite::valid(r.settings);}
bool persist(const Settings&s){
  if(revision==UINT32_MAX)return false;
  Record r={};r.magic=0x42414c54;r.schema=1;r.generation=revision+1;r.settings=s;r.crc=lite::crc32(reinterpret_cast<const uint8_t*>(&r),offsetof(Record,crc));
  const char*key=r.generation%2?"slot1":"slot0";
  if(preferences.putBytes(key,&r,sizeof(r))!=sizeof(r))return false;
  Record check={};return readRecord(key,check)&&memcmp(&check,&r,sizeof(r))==0;
}
void applyConfig(){
  auto profile=bearfield::BearCvV1();profile.minimumDepthMm=config.near;profile.playableMaximumMm=config.far;profile.fadeMaximumMm=config.far+150;profile.maximumSourceDepthMm=config.far+150;profile.presenceHoldMs=config.hold;profile.minimumBlobCells=config.cells;profile.spankMaximumDepthMm=config.spankDepth;profile.spankMinimumApproachMmPerSecond=config.spankSpeed;profile.spankRefractoryMs=config.cooldown;
  fieldProcessor=bearfield::Processor(profile);fieldState=fieldProcessor.target();spankGesture.reset();trajectoryAuthority=bearamin_trajectory::Authority();lastSensorFrameMs=0;
  setDisplayBrightness(config.displayEnabled?lite::brightnessRaw(config.brightness):0);
  forceSafeOutputs();
}
void error(JsonDocument&doc,const char*text){doc["op"]="error";doc["error"]=text;doc.remove("data");}
void queueReply(Endpoint&e,JsonDocument&doc){
  static char json[lite::kMaxJson+1];size_t n=measureJson(doc);
  if(n>lite::kMaxJson){++e.txFailures;return;}
  serializeJson(doc,json,sizeof(json));e.length=lite::encode(json,n,e.reply,sizeof(e.reply));e.offset=0;
}
void releasePerformance(Endpoint&e){e.release();}
void identify(JsonObject data,bool isBle){
  data["protocol"]=1;data["model"]="lite";data["hardware"]="bearcv-dual-mcp4728-v1";data["uid"]=uid;data["version"]=LITE_VERSION;data["build"]=LITE_BUILD_ID;data["digest"]=digest;data["transport"]=isBle?"Bluetooth MIDI":"USB MIDI";
  auto caps=data["capabilities"].to<JsonObject>();caps["settings"]=true;caps["field"]=true;caps["diagnostics"]=true;caps["display"]=true;caps["ble"]=true;caps["update"]=false;
}
void diagnostics(JsonObject data){
  data["sensor"]=sensorReady&&lastSensorFrameMs&&millis()-lastSensorFrameMs<=kSensorStaleMs?"Ready":"Not receiving";data["sensorHz"]=measuredHz;data["sourceFrames"]=frameCount;data["dacReady"]=dacReady;data["outputFault"]=outputFault;data["uptimeMs"]=millis();data["freeHeap"]=ESP.getFreeHeap();data["minimumHeap"]=ESP.getMinFreeHeap();data["usbQueueOverflows"]=usb.events.overflow;data["bleQueueOverflows"]=ble.events.overflow;data["usbTxRetries"]=usb.txFailures;data["bleTxRetries"]=ble.txFailures;data["resetReason"]=int(esp_reset_reason());data["pairingOpen"]=lite::pairingOpen(millis(),pairStart.load());
}
void handle(Endpoint&e,bool isBle){
  JsonDocument request;auto parse=deserializeJson(request,e.decoder.json,DeserializationOption::NestingLimit(8));
  if(parse||!request["request"].is<uint32_t>()||!request["op"].is<const char*>())return;
  const char*op=request["op"];if(strlen(op)>40)return;
  if(strcmp(op,"identity")&&(!request["uid"].is<const char*>()||strcmp(request["uid"],uid)))return;
  JsonDocument reply;reply["op"]=op;reply["request"]=request["request"].as<uint32_t>();reply["uid"]=uid;auto data=reply["data"].to<JsonObject>();
  if(!strcmp(op,"identity")){if(!request["data"]["protocol"].is<int>()||request["data"]["protocol"].as<int>()!=1)error(reply,"Unsupported protocol.");else identify(data,isBle);}
  else if(!strcmp(op,"settings.read")){writeSettings(data["settings"].to<JsonObject>(),config);data["revision"]=revision;}
  else if(!strcmp(op,"settings.write")){
    Settings next;
    if(!request["data"]["revision"].is<uint32_t>()||request["data"]["revision"].as<uint32_t>()!=revision)error(reply,"Settings changed. Reconnect and try again.");
    else if(!readSettings(request["data"]["settings"].as<JsonObjectConst>(),next))error(reply,"Check your settings and try again.");
    else if(!persist(next))error(reply,"Settings could not be saved.");
    else {releasePerformance(usb);releasePerformance(ble);config=next;++revision;applyConfig();data["revision"]=revision;}
  }else if(!strcmp(op,"diagnostics.read"))diagnostics(data);
  else if(!strcmp(op,"field.subscribe")){
    if(!request["data"]["hz"].is<int>()||(request["data"]["hz"].as<int>()!=0&&request["data"]["hz"].as<int>()!=5))error(reply,"Choose a supported preview rate.");
    else {e.subscribeUntil=request["data"]["hz"].as<int>()?millis()+300000:0;data["hz"]=request["data"]["hz"];}
  }else if(!strcmp(op,"bluetooth.reset")){data["ok"]=true;resetAfterMs=millis()+300;}
  else error(reply,"This prototype does not support that operation.");
  queueReply(e,reply);
}
class ServerCallbacks:public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*s,NimBLEConnInfo&c)override{
    if(bleHandle.load()!=BLE_HS_CONN_HANDLE_NONE||(!lite::pairingOpen(millis(),pairStart.load())&&!NimBLEDevice::isBonded(c.getIdAddress()))){s->disconnect(c);return;}
    admittedBond=NimBLEDevice::isBonded(c.getIdAddress());bleHandle=c.getConnHandle();bleMtu=23;bleSecure=false;bleSubscribed=false;++bleEpoch;advDirty=true;
    NimBLEDevice::startSecurity(c.getConnHandle());
  }
  void onMTUChange(uint16_t mtu,NimBLEConnInfo&c)override{if(c.getConnHandle()==bleHandle.load())bleMtu=mtu;}
  void onDisconnect(NimBLEServer*,NimBLEConnInfo&c,int)override{if(bleHandle.load()==c.getConnHandle()){bleHandle=BLE_HS_CONN_HANDLE_NONE;bleSecure=false;bleSubscribed=false;++bleEpoch;advDirty=true;}}
  void onAuthenticationComplete(NimBLEConnInfo&c)override{
    if(!admittedBond.load()&&!lite::pairingOpen(millis(),pairStart.load())){NimBLEDevice::deleteBond(c.getIdAddress());server->disconnect(c);return;}
    if(!c.isEncrypted()||!c.isBonded()){server->disconnect(c);return;}
    if(bleHandle.load()==c.getConnHandle()){bleSecure=true;advDirty=true;}
  }
} serverCallbacks;
class MidiCallbacks:public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic*c,NimBLEConnInfo&info)override{
    if(!info.isEncrypted()||!info.isBonded()||info.getConnHandle()!=bleHandle.load())return;
    auto v=c->getValue();if(v.size()<3||v.size()>244||!(v[0]&0x80)||!(v[1]&0x80))return;
    RxChunk chunk={};chunk.epoch=bleEpoch.load();chunk.n=v.size();memcpy(chunk.bytes,v.data(),v.size());if(xQueueSend(bleRx,&chunk,0)!=pdTRUE)rxOverflow=true;
  }
  void onSubscribe(NimBLECharacteristic*,NimBLEConnInfo&info,uint16_t sub)override{if(info.getConnHandle()==bleHandle.load())bleSubscribed=(sub&1)!=0;}
} midiCallbacks;
void advertise(){
  auto adv=NimBLEDevice::getAdvertising();adv->stop();
  if(bleHandle.load()!=BLE_HS_CONN_HANDLE_NONE)return;
  bool open=lite::pairingOpen(millis(),pairStart.load());int bonds=NimBLEDevice::getNumBonds();
  if(!open&&!bonds)return;
  for(int i=0;i<bonds;++i)NimBLEDevice::whiteListAdd(NimBLEDevice::getBondedAddress(i));
  adv->setScanFilter(!open,!open);adv->setDiscoverableMode(open?BLE_GAP_DISC_MODE_GEN:BLE_GAP_DISC_MODE_NON);
  if(!adv->start())Serial.println("BLE advertising could not start");
}
void beginBle(){
  bleRx=xQueueCreate(16,sizeof(RxChunk));if(!bleRx){Serial.println("BLE RX queue unavailable");return;}
  NimBLEDevice::init("Bearamin Lite");NimBLEDevice::setSecurityAuth(true,false,true);NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC|BLE_SM_PAIR_KEY_DIST_ID);NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC|BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setMTU(247);server=NimBLEDevice::createServer();server->setCallbacks(&serverCallbacks);server->advertiseOnDisconnect(false);
  auto service=server->createService("03b80e5a-ede8-4b33-a751-6ce34ec4c700");
  characteristic=service->createCharacteristic("7772e5db-3868-4112-a1a9-f2669d106bf3",NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::WRITE_NR|NIMBLE_PROPERTY::WRITE|NIMBLE_PROPERTY::NOTIFY|NIMBLE_PROPERTY::READ_ENC|NIMBLE_PROPERTY::WRITE_ENC,244);
  characteristic->setCallbacks(&midiCallbacks);service->start();auto adv=NimBLEDevice::getAdvertising();adv->addServiceUUID(service->getUUID());adv->enableScanResponse(true);adv->setName("Bearamin Lite");advertise();
}
void readSensor(){
  if(!sensorReady||!matrixSensor.isDataReady())return;
  if(!matrixSensor.getRangingData(&matrixResults)){sensorReady=false;spankGesture.reset();fieldState.spankHeld=false;sensorState=InitializerState::kError;nonDacOutputFault=outputFault=true;forceSafeOutputs();return;}
  bearfield::DepthFrame8x8 raw={};raw.sequence=++sourceSequence;raw.capturedUs=esp_timer_get_time();
  for(int i=0;i<64;++i){int16_t d=matrixResults.distance_mm[i];raw.distanceMm[i]=d>0?d:0;uint8_t status=matrixResults.target_status[i];if((status==5||status==9)&&d>0&&d<=2000)raw.validMask|=UINT64_C(1)<<i;}
  bearfield::SensorMountConfig mount=kSensorMount;mount.rawToCanonical.rotation=static_cast<bearfield::QuarterTurns>(config.rotation/90);mount.rawToCanonical.mirrorX=config.mirror;
  lastCanonical=bearfield::orientFrame(raw,mount);matrixDisplayFrame=lastCanonical;fieldProcessor.process(lastCanonical,raw.capturedUs);fieldState=fieldProcessor.target();updateSpankGesture(lastCanonical,millis(),config.spankDepth,config.spankSpeed,config.cooldown,config.cells);++frameCount;
  chaosInfluence=(float(fieldState.xNorm12)-2048)/4095+(float(fieldState.yNorm12)-2048)/8190;
  uint16_t target[]={fieldState.xNorm12,fieldState.yNorm12,fieldState.zNorm12,fieldState.pitchNorm12,fieldState.rollNorm12};
  uint32_t now=millis();uint16_t tau=bearcv::trajectoryTauMs(bearcv::normalizedKnob(analogRead(kSlewAdcPin)));
  if(!trajectoryAuthority.publishTarget(target,now,tau)){nonDacOutputFault=outputFault=true;forceSafeOutputs();return;}
  lastSensorFrameMs=now;
}
void samplePerformance(Endpoint&e){
  bool healthy=!outputFault&&sensorReady&&lastSensorFrameMs&&millis()-lastSensorFrameMs<=kSensorStaleMs;
  // Slew follows the physical knob; Range attenuates CV only. Curves/invert are MIDI-only.
  e.sample(config,healthy,fieldState.present,fieldState.spankHeld,trajectoryAxes.norm);
}
bool sendEvent(Endpoint&,bool isBle,lite::Event v){
  if(isBle){uint16_t ts=millis()&8191;uint8_t packet[]={uint8_t(0x80|((ts>>7)&63)),uint8_t(0x80|(ts&127)),v.status,v.a,v.b};return characteristic->notify(packet,sizeof(packet),bleHandle.load());}
  midiEventPacket_t packet={uint8_t(v.status>>4),v.status,v.a,v.b};return midi.writePacket(&packet);
}
bool sendReplyPart(Endpoint&e,bool isBle){
  size_t n=std::min<size_t>(3,e.length-e.offset);bool ok;
  if(isBle){uint8_t packet[244];auto slice=lite::bleSlice(e.reply+e.offset,e.length-e.offset,bleMtu.load(),millis()&8191,packet,sizeof(packet));n=slice.consumed;ok=n&&characteristic->notify(packet,slice.length,bleHandle.load());}
  else {bool final=e.offset+n==e.length;midiEventPacket_t packet={uint8_t(final?4+n:4),e.reply[e.offset],uint8_t(n>1?e.reply[e.offset+1]:0),uint8_t(n>2?e.reply[e.offset+2]:0)};ok=midi.writePacket(&packet);}
  if(ok){e.offset+=n;if(e.offset==e.length)e.length=e.offset=0;}else ++e.txFailures;
  return ok;
}
void transmit(Endpoint&e,bool isBle,bool axesDue){
  if(!e.ready)return;
  // Complete SysEx before channel messages; never splice a channel event into its payload.
  for(int i=0;i<(isBle?4:16)&&e.length;++i)if(!sendReplyPart(e,isBle))return;
  if(e.length)return;
  if(e.panic){
    if(!e.panicMask){e.panicMask=1u<<(e.channel-1);e.panicCc[e.channel-1]=e.presenceCc;}
    for(int ch=0;ch<16;++ch)if(e.panicMask&(1u<<ch)){
      if(!sendEvent(e,isBle,{uint8_t(0xb0|ch),123,0}))return;
      if(!sendEvent(e,isBle,{uint8_t(0xb0|ch),e.panicCc[ch],0}))return;
      e.panicMask&=~(1u<<ch);
    }e.panic=false;
  }
  for(int i=0;i<8&&e.events.front();++i){if(!sendEvent(e,isBle,*e.events.front())){++e.txFailures;return;}e.events.pop();}
  if(axesDue){const int cc[]={config.ccX,config.ccY,config.ccZ};for(int i=0;i<3;++i)if(e.axes[i]!=e.sent[i]){if(sendEvent(e,isBle,{uint8_t(0xb0|(config.channel-1)),uint8_t(cc[i]),e.axes[i]}))e.sent[i]=e.axes[i];else ++e.txFailures;}}
}
void preview(Endpoint&e){
  uint32_t now=millis();if(e.length||!e.ready||!e.subscribeUntil||int32_t(e.subscribeUntil-now)<=0||now-e.lastField<200)return;
  // Performance edges take priority over optional telemetry.
  if(e.events.size())return;
  e.lastField=now;JsonDocument doc;doc["op"]="field";doc["uid"]=uid;auto data=doc["data"].to<JsonObject>();auto cells=data["mm"].to<JsonArray>();bool fresh=sensorReady&&lastSensorFrameMs&&now-lastSensorFrameMs<=kSensorStaleMs;
  for(int i=0;i<64;++i){if(fresh&&(lastCanonical.validMask&(UINT64_C(1)<<i)))cells.add(lastCanonical.distanceMm[i]);else cells.add(nullptr);}
  data["x"]=e.axes[0];data["y"]=e.axes[1];data["z"]=e.axes[2];data["presence"]=e.presence;data["spank"]=e.noteHeld;queueReply(e,doc);
}
void receiveUsb(){
  if(usb.length)return;midiEventPacket_t packet;
  for(int i=0;i<48&&midi.readPacket(&packet);++i){uint8_t cin=packet.header&15,n=cin==4?3:cin>=5&&cin<=7?cin-4:0;if((packet.header>>4)!=0)continue;const uint8_t bytes[]={packet.byte1,packet.byte2,packet.byte3};for(int j=0;j<n;++j)if(usb.decoder.feed(bytes[j],millis())){handle(usb,false);return;}}
}
void receiveBle(){
  if(!bleRx||ble.length)return;if(rxOverflow.exchange(false)){ble.decoder.reset();xQueueReset(bleRx);}
  RxChunk chunk;
  for(int i=0;i<4&&xQueueReceive(bleRx,&chunk,0)==pdTRUE;++i){if(chunk.epoch!=bleEpoch.load())continue;for(int j=2;j<chunk.n;++j){uint8_t v=chunk.bytes[j];if((v&0x80)&&v!=0xf0&&v!=0xf7)continue;if(ble.decoder.feed(v,millis())){handle(ble,true);return;}}}
}
void resetBonds(){
  releasePerformance(usb);releasePerformance(ble);
  for(int i=0;i<NimBLEDevice::getNumBonds();++i)NimBLEDevice::whiteListRemove(NimBLEDevice::getBondedAddress(i));
  if(bleHandle.load()!=BLE_HS_CONN_HANDLE_NONE)server->disconnect(bleHandle.load());
  NimBLEDevice::deleteAllBonds();pairStart=millis();usb.subscribeUntil=ble.subscribeUntil=0;usb.decoder.reset();ble.decoder.reset();if(bleRx)xQueueReset(bleRx);advDirty=true;resetAfterMs=0;
}
void begin(){
  pairStart=millis();
  uint8_t mac[6];esp_read_mac(mac,ESP_MAC_WIFI_STA);snprintf(uid,sizeof(uid),"lite-%02x%02x%02x%02x%02x%02x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  uint8_t hash[32]={};const esp_partition_t*app=esp_ota_get_running_partition();if(app&&esp_partition_get_sha256(app,hash)==ESP_OK)for(int i=0;i<32;++i)snprintf(digest+2*i,3,"%02x",hash[i]);
  USB.productName("Bearamin Lite");USB.manufacturerName("Hivemind Synthesis");USB.serialNumber(uid);midi.begin();USB.begin();
  bearcvSetup();Wire.setTimeOut(20);
  preferences.begin("bearamin-lite",false);Record a={},b={};bool av=readRecord("slot0",a),bv=readRecord("slot1",b);if(av||bv){const Record&r=av&&(!bv||a.generation>b.generation)?a:b;config=r.settings;revision=r.generation;}
  applyConfig();beginBle();Serial.printf("Bearamin Lite %s build %lu; USB MIDI + bonded BLE MIDI; settings revision %lu\n",LITE_VERSION,(unsigned long)LITE_BUILD_ID,(unsigned long)revision);
}
void loop(){
  uint32_t now=millis();if(!dacReady&&now-lastDacRecoveryMs>=bearcv_boot::kRetryMs){lastDacRecoveryMs=now;DacProvisionStatus status=DacProvisionStatus::kAddressVerifyFailed;dacReady=beginDacs(&status);if(dacReady)outputFault=nonDacOutputFault||!sensorReady;Serial.printf("DAC recovery: %s\n",dacProvisionStatusText(status));}
  pollChaosImu(now);Wire.setTimeOut(20);readSensor();serviceOutputs(esp_timer_get_time());
  if(now-rateEpoch>=1000){measuredHz=float(frameCount-rateFrames)*1000/uint32_t(now-rateEpoch);rateFrames=frameCount;rateEpoch=now;}
  bool usbReady=tud_mounted();if(usbReady!=usb.ready){usb.clear();usb.ready=usbReady;usb.panic=usbReady;}
  uint32_t epoch=bleEpoch.load();bool bleReady=bleSecure.load()&&bleSubscribed.load();if(ble.epoch!=epoch||bleReady!=ble.ready){ble.clear();ble.ready=bleReady;ble.epoch=epoch;ble.panic=bleReady;}
  samplePerformance(usb);samplePerformance(ble);
  bool due=now-lastMidiMs>=1000u/config.rate;if(due)lastMidiMs=now;
  transmit(usb,false,due);transmit(ble,true,due);
  receiveUsb();receiveBle();usb.decoder.expire(now);ble.decoder.expire(now);
  preview(usb);preview(ble);
  static bool wasOpen=true;bool open=lite::pairingOpen(now,pairStart.load());if(open!=wasOpen){wasOpen=open;advDirty=true;}if(server&&advDirty.exchange(false))advertise();
  if(resetAfterMs&&int32_t(now-resetAfterMs)>=0&&!usb.length&&!ble.length)resetBonds();
  delay(1);
}
#undef LITE_INT_FIELDS
#undef LITE_BOOL_FIELDS
} // namespace lite_runtime
