#pragma once
#include <ArduinoJson.h>
#include "editor_contract.h"
namespace virtual_editor {
#define LITE_INT_FIELDS(X) X(channel) X(rate) X(ccX) X(ccY) X(ccZ) X(ccPresence) X(note) X(velocity) X(near) X(far) X(hold) X(cells) X(spankSpeed) X(spankDepth) X(cooldown) X(brightness) X(smoothing)
#define LITE_BOOL_FIELDS(X) X(invertX) X(invertY) X(invertZ) X(presenceEnabled) X(spankEnabled) X(mirror) X(displayEnabled)
const char*curveName(int v){return v==1?"gentle":v==2?"steep":"linear";}
void writeSettings(JsonObject obj,const Settings&s){
#define WRITE_INT(k) obj[#k]=s.k;
  LITE_INT_FIELDS(WRITE_INT)
#undef WRITE_INT
#define WRITE_BOOL(k) obj[#k]=s.k;
  LITE_BOOL_FIELDS(WRITE_BOOL)
#undef WRITE_BOOL
  obj["curveX"]=curveName(s.curveX);obj["curveY"]=curveName(s.curveY);obj["curveZ"]=curveName(s.curveZ);
  obj["rotation"]=s.rotation==90?"90":s.rotation==180?"180":s.rotation==270?"270":"0";
}
bool readSettings(JsonObjectConst obj,Settings&s){
  if(obj.isNull()||obj.size()!=28)return false;
#define READ_INT(k) if(!obj[#k].is<int>())return false; s.k=obj[#k].as<int>();
  LITE_INT_FIELDS(READ_INT)
#undef READ_INT
#define READ_BOOL(k) if(!obj[#k].is<bool>())return false; s.k=obj[#k].as<bool>();
  LITE_BOOL_FIELDS(READ_BOOL)
#undef READ_BOOL
  const char*names[]={"curveX","curveY","curveZ"};int*curves[]={&s.curveX,&s.curveY,&s.curveZ};
  for(int i=0;i<3;++i){if(!obj[names[i]].is<const char*>())return false;const char*v=obj[names[i]];if(!strcmp(v,"linear"))*curves[i]=0;else if(!strcmp(v,"gentle"))*curves[i]=1;else if(!strcmp(v,"steep"))*curves[i]=2;else return false;}
  if(obj["rotation"].is<int>())s.rotation=obj["rotation"].as<int>();
  else if(obj["rotation"].is<const char*>()){const char*v=obj["rotation"];if(!strcmp(v,"0"))s.rotation=0;else if(!strcmp(v,"90"))s.rotation=90;else if(!strcmp(v,"180"))s.rotation=180;else if(!strcmp(v,"270"))s.rotation=270;else return false;}else return false;
  return virtual_editor::valid(s);
}

#undef LITE_INT_FIELDS
#undef LITE_BOOL_FIELDS
} // namespace lite
