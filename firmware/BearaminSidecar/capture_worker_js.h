#pragma once

#include <Arduino.h>

const char CAPTURE_WORKER_JS[] PROGMEM = R"JS(
'use strict';

const DB_NAME='bearamin-captures';
const DB_VERSION=1;
const RAW_CHUNK_BYTES=64*1024;
const ARTIFACT_CHUNK_BYTES=64*1024;
const FETCH_LIMIT=8192;
const INITIAL_SEQUENCE=0xffffffff;
const RECORD_NAMES=['unknown','session_start','reserved_2','matrix','imu','snapshot_v1','control','diagnostic','gap','session_end','metadata','snapshot_v2'];
const ARTIFACT_FILES=['manifest.json','playback_frames.jsonl','matrix.jsonl','imu.jsonl','control.jsonl','diagnostics.jsonl','gaps.jsonl'];
const encoder=new TextEncoder();
let dbPromise;
let current=null;
let pollRunning=false;

function request(r){return new Promise((resolve,reject)=>{r.onsuccess=()=>resolve(r.result);r.onerror=()=>reject(r.error)})}
function transactionDone(tx){return new Promise((resolve,reject)=>{tx.oncomplete=resolve;tx.onerror=()=>reject(tx.error);tx.onabort=()=>reject(tx.error||new Error('transaction aborted'))})}
function sleep(ms){return new Promise(resolve=>setTimeout(resolve,ms))}
function openDb(){
  if(dbPromise)return dbPromise;
  dbPromise=new Promise((resolve,reject)=>{
    const r=indexedDB.open(DB_NAME,DB_VERSION);
    r.onupgradeneeded=()=>{
      const db=r.result;
      if(!db.objectStoreNames.contains('captures'))db.createObjectStore('captures',{keyPath:'id'});
      if(!db.objectStoreNames.contains('rawChunks')){
        const s=db.createObjectStore('rawChunks',{keyPath:['captureId','index']});
        s.createIndex('captureId','captureId');
      }
      if(!db.objectStoreNames.contains('artifactChunks')){
        const s=db.createObjectStore('artifactChunks',{keyPath:['captureId','filename','index']});
        s.createIndex('captureId','captureId');
      }
    };
    r.onsuccess=()=>resolve(r.result);
    r.onerror=()=>reject(r.error);
  });
  return dbPromise;
}
async function putCapture(meta){
  const db=await openDb(),tx=db.transaction('captures','readwrite');
  tx.objectStore('captures').put(meta);
  await transactionDone(tx);
}
async function getCapture(id){
  const db=await openDb(),tx=db.transaction('captures','readonly');
  return request(tx.objectStore('captures').get(id));
}
async function allCaptures(){
  const db=await openDb(),tx=db.transaction('captures','readonly');
  const values=await request(tx.objectStore('captures').getAll());
  return values.sort((a,b)=>b.createdAt-a.createdAt);
}
async function rowsFor(storeName,id){
  const db=await openDb(),tx=db.transaction(storeName,'readonly');
  const rows=await request(tx.objectStore(storeName).index('captureId').getAll(id));
  return rows.sort((a,b)=>a.index-b.index);
}
async function deleteRowsFor(storeName,id){
  const db=await openDb(),tx=db.transaction(storeName,'readwrite'),index=tx.objectStore(storeName).index('captureId');
  await new Promise((resolve,reject)=>{
    const cursor=index.openKeyCursor(IDBKeyRange.only(id));
    cursor.onerror=()=>reject(cursor.error);
    cursor.onsuccess=()=>{
      const c=cursor.result;
      if(!c){resolve();return}
      tx.objectStore(storeName).delete(c.primaryKey);
      c.continue();
    };
  });
  await transactionDone(tx);
}
async function deleteCapture(id){
  if(current&&current.id===id)throw new Error('active capture cannot be deleted');
  await deleteRowsFor('rawChunks',id);
  await deleteRowsFor('artifactChunks',id);
  const db=await openDb(),tx=db.transaction('captures','readwrite');
  tx.objectStore('captures').delete(id);
  await transactionDone(tx);
  await publishLibrary();
}
function u16(v,o){return v.getUint16(o,true)}
function i16(v,o){return v.getInt16(o,true)}
function u32(v,o){return v.getUint32(o,true)}
function u64(v,o){return u32(v,o)+u32(v,o+4)*4294967296}
function crc32(bytes){
  let crc=0xffffffff;
  for(const byte of bytes){
    crc^=byte;
    for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);
  }
  return (crc^0xffffffff)>>>0;
}
function parseRecords(bytes,strict=true){
  const records=[];
  let offset=0;
  while(offset<bytes.length){
    if(offset+26>bytes.length)throw new Error('partial capture record');
    if(bytes[offset]!==66||bytes[offset+1]!==67||bytes[offset+2]!==65||bytes[offset+3]!==80){
      if(strict)throw new Error('capture framing error');
      offset++;continue;
    }
    const view=new DataView(bytes.buffer,bytes.byteOffset+offset);
    const version=view.getUint8(4),type=view.getUint8(5),flags=u16(view,6),payloadSize=u16(view,8);
    const size=26+payloadSize;
    if(version!==1||payloadSize>224||offset+size>bytes.length)throw new Error('invalid capture record length');
    const expected=u32(view,22+payloadSize);
    const body=bytes.subarray(offset,offset+22+payloadSize);
    if(crc32(body)!==expected)throw new Error('capture CRC mismatch');
    records.push({type,flags,sequence:u32(view,10),coreUs:u64(view,14),payload:bytes.subarray(offset+22,offset+22+payloadSize),bytes:bytes.subarray(offset,offset+size)});
    offset+=size;
  }
  return records;
}
function sequenceAfter(a,b){return ((a-b)|0)>0}
function status(extra={}){
  if(!current)return;
  const explicit=(current.matrixDrops||0)+Math.max(current.sidecarGapDrops||0,current.deviceStatus?.drops||0);
  postMessage({type:'status',id:current.id,createdAt:current.createdAt,elapsedMs:Date.now()-current.createdAt,bytes:current.rawBytes||0,records:current.recordCount||0,counts:current.counts||{},drops:explicit+(current.clientGaps||[]).reduce((n,g)=>n+g.count,0),crcErrors:current.crcErrors||0,sequenceErrors:current.sequenceErrors||0,stage:current.stage||current.status,device:current.deviceStatus||{},...extra});
}
async function updateCurrent(){
  current.updatedAt=Date.now();
  await putCapture({...current,pendingSegments:undefined,deviceStatus:undefined});
  status();
}
function addFetched(bytes){
  const records=parseRecords(bytes);
  for(const record of records){
    if(current.haveSequence&&record.sequence!==current.expectedSequence){
      if(record.type!==8){
        current.sequenceErrors++;
        current.clientGaps.push({stage:'browser_sequence',first_seq:current.expectedSequence,last_seq:(record.sequence-1)>>>0,count:(record.sequence-current.expectedSequence)>>>0});
      }
    }
    current.haveSequence=true;
    current.expectedSequence=(record.sequence+1)>>>0;
    current.lastParsedSeq=record.sequence;
    current.recordCount++;
    const name=RECORD_NAMES[record.type]||'unknown';
    current.counts[name]=(current.counts[name]||0)+1;
    if(record.type===8&&record.payload.length===16){
      const gap=new DataView(record.payload.buffer,record.payload.byteOffset,record.payload.byteLength),count=u32(gap,12);
      if(gap.getUint8(1)===1)current.matrixDrops=(current.matrixDrops||0)+count;
      if(gap.getUint8(1)===2)current.sidecarGapDrops=(current.sidecarGapDrops||0)+count;
    }
    current.pendingSegments.push({bytes:new Uint8Array(record.bytes),endSeq:record.sequence});
    current.pendingBytes+=record.bytes.length;
    current.rawBytes+=record.bytes.length;
  }
}
async function commitChunk(segments){
  const size=segments.reduce((n,s)=>n+s.bytes.length,0),joined=new Uint8Array(size);
  let cursor=0;
  for(const segment of segments){joined.set(segment.bytes,cursor);cursor+=segment.bytes.length}
  const startSeq=segments[0].endSeq,endSeq=segments[segments.length-1].endSeq;
  const db=await openDb(),tx=db.transaction(['rawChunks','captures'],'readwrite');
  tx.objectStore('rawChunks').put({captureId:current.id,index:current.nextChunkIndex++,startSeq,endSeq,bytes:joined.buffer});
  current.committedSeq=endSeq;
  current.lastCommitAt=Date.now();
  tx.objectStore('captures').put({...current,pendingSegments:undefined,deviceStatus:undefined});
  await transactionDone(tx);
}
async function flushPending(force=false){
  while(current.pendingSegments.length&&(force||current.pendingBytes>=RAW_CHUNK_BYTES)){
    const selected=[];
    let size=0;
    while(current.pendingSegments.length){
      const next=current.pendingSegments[0];
      if(selected.length&&size+next.bytes.length>RAW_CHUNK_BYTES)break;
      selected.push(current.pendingSegments.shift());
      current.pendingBytes-=next.bytes.length;
      size+=next.bytes.length;
      if(size>=RAW_CHUNK_BYTES)break;
    }
    await commitChunk(selected);
    if(!force&&current.pendingBytes<RAW_CHUNK_BYTES)break;
  }
}
async function fetchDeviceStatus(){
  const r=await fetch('/capture/status',{cache:'no-store'});
  if(!r.ok)throw new Error('capture status unavailable');
  current.deviceStatus=await r.json();
  if(current.deviceStatus.final_seq!=null)current.finalSeq=current.deviceStatus.final_seq>>>0;
}
async function fetchData(){
  const after=current.lastParsedSeq==null?INITIAL_SEQUENCE:current.lastParsedSeq;
  const ack=current.committedSeq==null?INITIAL_SEQUENCE:current.committedSeq;
  const url=`/capture/data?id=${encodeURIComponent(current.id)}&after=${after>>>0}&ack=${ack>>>0}&limit=${FETCH_LIMIT}`;
  const r=await fetch(url,{cache:'no-store'});
  if(!r.ok)throw new Error(`capture data ${r.status}`);
  const bytes=new Uint8Array(await r.arrayBuffer());
  if(bytes.length)addFetched(bytes);
  if(current.pendingBytes>=RAW_CHUNK_BYTES||Date.now()-current.lastCommitAt>=1000)await flushPending(true);
  status();
  return bytes.length;
}
async function requestStop(){
  current.stage='stopping';
  status();
  const r=await fetch(`/capture/stop?id=${encodeURIComponent(current.id)}`,{method:'POST',cache:'no-store'});
  if(r.status!==200&&r.status!==202)throw new Error(`capture stop ${r.status}`);
  const result=await r.json();
  if(result.final_seq!=null)current.finalSeq=result.final_seq>>>0;
}
async function pollCapture(){
  if(pollRunning)return;
  pollRunning=true;
  try{
    while(current&&(current.status==='active'||current.status==='stopping')){
      if(current.stopRequested&&current.status==='active'){
        current.status='stopping';
        await requestStop();
      }
      const bytes=await fetchData();
      if(Date.now()-(current.lastStatusAt||0)>400){
        await fetchDeviceStatus();
        current.lastStatusAt=Date.now();
      }
      if(current.status==='stopping'&&current.finalSeq!=null&&current.lastParsedSeq===current.finalSeq){
        await flushPending(true);
        if(current.committedSeq===current.finalSeq)break;
      }
      if(!bytes)await sleep(25);
    }
    if(current&&current.status==='stopping')await finalizeCapture();
  }catch(error){
    await failCurrent(error);
  }finally{pollRunning=false}
}
async function failCurrent(error){
  if(!current)return;
  try{await fetch(`/capture/stop?id=${encodeURIComponent(current.id)}`,{method:'POST',cache:'no-store'})}catch(_){}
  try{await flushPending(true)}catch(_){}
  current.status='incomplete';
  current.stage='incomplete';
  current.error=String(error&&error.message||error);
  await updateCurrent().catch(()=>{});
  postMessage({type:'error',id:current.id,error:current.error});
  current=null;
  await publishLibrary();
}
async function rawBlob(id){
  const rows=await rowsFor('rawChunks',id);
  return new Blob(rows.map(row=>row.bytes),{type:'application/octet-stream'});
}
async function verifyRaw(id){
  const rows=await rowsFor('rawChunks',id);
  let count=0,last=null,bytes=0,crc=0xffffffff;
  for(const row of rows){
    const chunk=new Uint8Array(row.bytes);
    for(const byte of chunk){crc^=byte;for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0)}
    const records=parseRecords(chunk);
    if(!records.length)throw new Error('empty raw chunk');
    count+=records.length;
    bytes+=chunk.length;
    last=records[records.length-1].sequence;
  }
  return {count,last,bytes,chunks:rows.length,crc32:(crc^0xffffffff)>>>0};
}
class ArtifactWriter{
  constructor(captureId,filename){this.captureId=captureId;this.filename=filename;this.text='';this.index=0;this.rows=0;this.bytes=0;this.crc=0xffffffff}
  async line(value){this.text+=JSON.stringify(value)+'\n';this.rows++;if(this.text.length>=ARTIFACT_CHUNK_BYTES)await this.flush()}
  async flush(){
    if(!this.text)return;
    const bytes=encoder.encode(this.text);
    for(const byte of bytes){this.crc^=byte;for(let bit=0;bit<8;bit++)this.crc=(this.crc>>>1)^((this.crc&1)?0xedb88320:0)}
    const db=await openDb(),tx=db.transaction('artifactChunks','readwrite');
    tx.objectStore('artifactChunks').put({captureId:this.captureId,filename:this.filename,index:this.index++,bytes:bytes.buffer});
    await transactionDone(tx);
    this.bytes+=bytes.length;
    this.text='';
  }
  async finish(){await this.flush();return{filename:this.filename,rows:this.rows,bytes:this.bytes,crc:(this.crc^0xffffffff)>>>0,chunks:this.index}}
}
function sensorFlags(flags){return{matrix_ready:!!(flags&4),matrix_present:!!(flags&8),imu_ready:!!(flags&16),impact_gate:!!(flags&32),spank_gate:!!(flags&64)}}
function decodeMatrix(record){
  const v=new DataView(record.payload.buffer,record.payload.byteOffset,record.payload.byteLength),cells=[],statuses=[];
  for(let i=0;i<64;i++){cells.push(u16(v,16+i*2));statuses.push(v.getUint8(144+i))}
  return{core_us:record.coreUs,record_seq:record.sequence,ready:!!(v.getUint8(2)&1),presence:!!(v.getUint8(2)&2),valid_count:v.getUint8(0),blob_count:v.getUint8(1),depth_mm:u16(v,4),x_milli:u16(v,6),y_milli:u16(v,8),z_norm:u16(v,10),x_norm:u16(v,12),y_norm:u16(v,14),cells_mm:cells,statuses};
}
function decodeSnapshot(record,matrix){
  const v=new DataView(record.payload.buffer,record.payload.byteOffset,record.payload.byteLength);
  const schema=u16(v,0),v1=record.payload.length===34&&schema===1,v2=record.payload.length===40&&schema===2;
  if(!v1&&!v2)throw new Error('unsupported processed snapshot');
  const flags=sensorFlags(u16(v,2));
  return{core_us:record.coreUs,core_ms:Math.floor(record.coreUs/1000),source_sequences:{snapshot:record.sequence,matrix:matrix?matrix.record_seq:null},...flags,matrix8x8_nearest_mm:u16(v,8),matrix8x8_x_milli:u16(v,10),matrix8x8_y_milli:u16(v,12),matrix8x8_z_norm:u16(v,14),matrix8x8_x_norm:u16(v,16),matrix8x8_y_norm:u16(v,18),matrix8x8_valid_count:v.getUint8(20),matrix8x8_blob_count:v.getUint8(21),impact_velocity:u16(v,22),impact_age_ms:u16(v,24),accel_mg:[i16(v,26),i16(v,28),i16(v,30)],sensor_age_ms:u16(v,32),pitch_norm:v2?u16(v,34):2048,roll_norm:v2?u16(v,36):2048,orientation_confidence:v2?v.getUint8(38):0,cells_mm:matrix?matrix.cells_mm:null,cell_statuses:matrix?matrix.statuses:null};
}
function hex(bytes){return Array.from(bytes,b=>b.toString(16).padStart(2,'0')).join('')}
async function storeManifest(id,manifest){
  const bytes=encoder.encode(JSON.stringify(manifest,null,2)+'\n'),db=await openDb(),tx=db.transaction('artifactChunks','readwrite');
  tx.objectStore('artifactChunks').put({captureId:id,filename:'manifest.json',index:0,bytes:bytes.buffer});
  await transactionDone(tx);
  return{filename:'manifest.json',rows:1,bytes:bytes.length,crc:crc32(bytes),chunks:1};
}
async function cook(id,verification){
  await deleteRowsFor('artifactChunks',id);
  const writers={playback:new ArtifactWriter(id,'playback_frames.jsonl'),matrix:new ArtifactWriter(id,'matrix.jsonl'),imu:new ArtifactWriter(id,'imu.jsonl'),control:new ArtifactWriter(id,'control.jsonl'),diagnostics:new ArtifactWriter(id,'diagnostics.jsonl'),gaps:new ArtifactWriter(id,'gaps.jsonl')};
  let latestMatrix=null,sessionEnd=null,cooked=0;
  const rows=await rowsFor('rawChunks',id);
  for(const row of rows){
    for(const record of parseRecords(new Uint8Array(row.bytes))){
      cooked++;
      const v=new DataView(record.payload.buffer,record.payload.byteOffset,record.payload.byteLength);
      if(record.type===3){
        latestMatrix=decodeMatrix(record);await writers.matrix.line(latestMatrix);
      }else if(record.type===4){
        const count=v.getUint8(0);
        for(let i=0;i<count;i++){const o=8+i*14;await writers.imu.line({core_us:record.coreUs+u16(v,o),batch_seq:record.sequence,sample_index:i,raw_accel:[i16(v,o+2),i16(v,o+4),i16(v,o+6)],raw_gyro:[i16(v,o+8),i16(v,o+10),i16(v,o+12)],impact_velocity:u16(v,2),impact_age_ms:u16(v,4),impact_gate:!!v.getUint8(6)})}
      }else if(record.type===5||record.type===11){
        await writers.playback.line(decodeSnapshot(record,latestMatrix));
      }else if(record.type===6){
        const length=v.getUint8(1);await writers.control.line({core_us:record.coreUs,record_seq:record.sequence,kind:v.getUint8(0),length,frame_hex:hex(record.payload.subarray(2,2+length)),replayable_sensor_control:false});
      }else if(record.type===7){
        await writers.diagnostics.line({core_us:record.coreUs,record_seq:record.sequence,recovery_failures:u16(v,4),i2c_recoveries:u16(v,6),matrix_age_ms:u16(v,10),recovery_reason:v.getUint8(12),bus_flags:v.getUint8(15),imu_read_failures:u32(v,16),imu_recoveries:u32(v,20)});
      }else if(record.type===8){
        await writers.gaps.line({core_us:record.coreUs,record_seq:record.sequence,dropped_type:v.getUint8(0),stage:v.getUint8(1),first_seq:u32(v,4),last_seq:u32(v,8),count:u32(v,12)});
      }else if(record.type===9){
        sessionEnd={final_seq:u32(v,4),record_count:u32(v,8),drop_total:u32(v,52),drop_by_type:Array.from({length:10},(_,i)=>u32(v,12+i*4))};
        await writers.diagnostics.line({core_us:record.coreUs,record_seq:record.sequence,session_end:sessionEnd});
      }else{
        await writers.diagnostics.line({core_us:record.coreUs,record_seq:record.sequence,type:RECORD_NAMES[record.type]||record.type,payload_hex:hex(record.payload)});
      }
      if(cooked%256===0)status({cooking:{records:cooked,total:verification.count}});
    }
  }
  for(const gap of current?.clientGaps||[])await writers.gaps.line(gap);
  const artifacts=[];
  for(const writer of Object.values(writers))artifacts.push(await writer.finish());
  const meta=await getCapture(id);
  const manifest={format:'bearamin-capture',capture_schema:2,id,created_at:new Date(meta.createdAt).toISOString(),completed_at:new Date().toISOString(),raw:verification,counts:meta.counts||{},client_sequence_errors:meta.sequenceErrors||0,client_gaps:meta.clientGaps||[],authoritative_end:sessionEnd,playback:{processed_snapshot:'ProcessedSensorSnapshotV2',legacy_processed_snapshot:'ProcessedSensorSnapshotV1',live_controls_excluded:['knobs','mappings','dpt_mode','dpt_status','dpt_control_flags'],accepted_future_inputs:['legacy CSV','.bcap','playback_frames.jsonl']},files:Object.fromEntries(artifacts.map(a=>[a.filename,{rows:a.rows,bytes:a.bytes,crc32:a.crc}]))};
  artifacts.unshift(await storeManifest(id,manifest));
  return{manifest,artifacts};
}
function dosDateTime(date=new Date()){
  const time=(date.getHours()<<11)|(date.getMinutes()<<5)|(date.getSeconds()>>1);
  const year=Math.max(1980,date.getFullYear());
  return{time,date:((year-1980)<<9)|((date.getMonth()+1)<<5)|date.getDate()};
}
function write16(v,o,n){v.setUint16(o,n,true)}function write32(v,o,n){v.setUint32(o,n>>>0,true)}
async function zipFileRows(id,filename){
  const rows=(await rowsFor('artifactChunks',id)).filter(row=>row.filename===filename);
  return rows.map(row=>new Uint8Array(row.bytes));
}
async function prepareZipEntry(id,filename){
  const chunks=await zipFileRows(id,filename),plain=new Blob(chunks),plainSize=chunks.reduce((n,b)=>n+b.length,0);
  let crc=0xffffffff;
  for(const chunk of chunks)for(const byte of chunk){crc^=byte;for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0)}
  crc=(crc^0xffffffff)>>>0;
  let method=0,dataParts=chunks,compressedSize=plainSize;
  if(plainSize<=8*1024*1024&&typeof CompressionStream!=='undefined'){
    try{
      const compressed=new Uint8Array(await new Response(plain.stream().pipeThrough(new CompressionStream('deflate-raw'))).arrayBuffer());
      if(compressed.length<plainSize){method=8;dataParts=[compressed];compressedSize=compressed.length}
    }catch(_){}
  }
  return{filename,name:encoder.encode(filename),method,crc,plainSize,compressedSize,dataParts};
}
async function buildZip(id){
  const entries=[];
  for(const filename of ARTIFACT_FILES)entries.push(await prepareZipEntry(id,filename));
  const parts=[],central=[];
  let offset=0;
  const dt=dosDateTime();
  for(const entry of entries){
    const header=new Uint8Array(30+entry.name.length),v=new DataView(header.buffer);
    write32(v,0,0x04034b50);write16(v,4,20);write16(v,6,0);write16(v,8,entry.method);write16(v,10,dt.time);write16(v,12,dt.date);write32(v,14,entry.crc);write32(v,18,entry.compressedSize);write32(v,22,entry.plainSize);write16(v,26,entry.name.length);write16(v,28,0);header.set(entry.name,30);
    parts.push(header,...entry.dataParts);
    const c=new Uint8Array(46+entry.name.length),cv=new DataView(c.buffer);
    write32(cv,0,0x02014b50);write16(cv,4,20);write16(cv,6,20);write16(cv,8,0);write16(cv,10,entry.method);write16(cv,12,dt.time);write16(cv,14,dt.date);write32(cv,16,entry.crc);write32(cv,20,entry.compressedSize);write32(cv,24,entry.plainSize);write16(cv,28,entry.name.length);write16(cv,30,0);write16(cv,32,0);write16(cv,34,0);write16(cv,36,0);write32(cv,38,0);write32(cv,42,offset);c.set(entry.name,46);central.push(c);
    offset+=header.length+entry.compressedSize;
  }
  const centralSize=central.reduce((n,b)=>n+b.length,0),end=new Uint8Array(22),ev=new DataView(end.buffer);
  write32(ev,0,0x06054b50);write16(ev,4,0);write16(ev,6,0);write16(ev,8,entries.length);write16(ev,10,entries.length);write32(ev,12,centralSize);write32(ev,16,offset);write16(ev,20,0);
  return new Blob([...parts,...central,end],{type:'application/zip'});
}
function requestDownload(kind,filename,blob){postMessage({type:'download',kind,filename,blob})}
async function finalizeCapture(){
  current.stage='verifying';
  await flushPending(true);
  const verification=await verifyRaw(current.id);
  if(current.finalSeq!=null&&verification.last!==current.finalSeq)throw new Error('raw final sequence mismatch');
  current.verification=verification;
  current.status='stored';
  await updateCurrent();
  const raw=await rawBlob(current.id);
  requestDownload('raw',`${current.basename}.bcap`,raw);
  current.rawDownloadRequested=true;
  current.stage='cooking';
  await updateCurrent();
  const cooked=await cook(current.id,verification);
  current.artifacts=cooked.artifacts;
  current.stage='zipping';
  await updateCurrent();
  const zip=await buildZip(current.id);
  requestDownload('json',`${current.basename}-json.zip`,zip);
  current.jsonDownloadRequested=true;
  current.exported=true;
  current.drops=(current.matrixDrops||0)+Math.max(current.sidecarGapDrops||0,current.deviceStatus?.drops||0)+(current.clientGaps||[]).reduce((n,g)=>n+g.count,0);
  current.status='complete';
  current.stage='complete';
  current.completedAt=Date.now();
  await updateCurrent();
  await cleanupExported();
  const completed={...current};
  current=null;
  postMessage({type:'complete',capture:completed});
  await publishLibrary();
}
async function cleanupExported(){
  let captures=await allCaptures();
  const cutoff=Date.now()-7*24*60*60*1000;
  for(const meta of captures)if(meta.exported&&meta.completedAt<cutoff)await deleteCapture(meta.id);
  captures=(await allCaptures()).filter(meta=>meta.exported).sort((a,b)=>a.completedAt-b.completedAt);
  while(captures.length>5){const oldest=captures.shift();await deleteCapture(oldest.id)}
}
async function startCapture(message){
  if(current)throw new Error('capture already active');
  const now=Date.now(),id=message.id,basename=message.basename;
  current={id,basename,createdAt:now,updatedAt:now,status:'active',stage:'starting',rawBytes:0,recordCount:0,counts:{},clientGaps:[],matrixDrops:0,sidecarGapDrops:0,crcErrors:0,sequenceErrors:0,haveSequence:false,expectedSequence:0,lastParsedSeq:null,committedSeq:null,nextChunkIndex:0,pendingSegments:[],pendingBytes:0,lastCommitAt:now,stopRequested:false,rawDownloadRequested:false,jsonDownloadRequested:false,exported:false};
  await putCapture({...current,pendingSegments:undefined});
  const r=await fetch(`/capture/start?id=${encodeURIComponent(id)}`,{method:'POST',cache:'no-store'});
  if(!r.ok)throw new Error(`capture start ${r.status}`);
  current.stage='capturing';
  await updateCurrent();
  pollCapture();
}
async function retryDownload(id,kind){
  const meta=await getCapture(id);
  if(!meta)throw new Error('capture not found');
  if(kind==='raw')requestDownload('raw',`${meta.basename}.bcap`,await rawBlob(id));
  else requestDownload('json',`${meta.basename}-json.zip`,await buildZip(id));
}
async function publishLibrary(){postMessage({type:'library',captures:await allCaptures()})}
async function recover(){
  const captures=await allCaptures(),active=captures.find(meta=>meta.status==='active'||meta.status==='stopping');
  if(active){
    try{
      const r=await fetch('/capture/status',{cache:'no-store'}),device=await r.json();
      if(device.session_id===active.id&&(device.active||device.stopping)){
        current={...active,pendingSegments:[],pendingBytes:0,deviceStatus:device,lastCommitAt:Date.now(),stopRequested:active.status==='stopping'};
        current.lastParsedSeq=active.lastParsedSeq??null;current.committedSeq=active.committedSeq??null;
        pollCapture();
      }else{
        active.status='interrupted';active.stage='interrupted';await putCapture(active);
      }
    }catch(_){active.status='interrupted';active.stage='interrupted';await putCapture(active)}
  }
  await publishLibrary();
}
self.onmessage=async event=>{
  const m=event.data||{};
  try{
    if(m.type==='start')await startCapture(m);
    else if(m.type==='stop'&&current){current.stopRequested=true;status()}
    else if(m.type==='retry')await retryDownload(m.id,m.kind);
    else if(m.type==='delete')await deleteCapture(m.id);
    else if(m.type==='library')await publishLibrary();
  }catch(error){
    if(current&&(m.type==='start'||m.type==='stop'))await failCurrent(error);
    else postMessage({type:'error',id:m.id,error:String(error&&error.message||error)});
  }
};
recover();
)JS";
