'use strict';
// Shared draft v1 contract. 0x7d is the development SysEx ID, not a commercial assignment.
(function (root) {
  const PREFIX = [0xf0, 0x7d, 0x42, 0x41, 1], MAX_BYTES = 4096;
  function encode(value) {
    const bytes = new TextEncoder().encode(JSON.stringify(value));
    if (bytes.length > MAX_BYTES) throw Error('Message is too large.');
    const out = [...PREFIX];
    for (let i=0;i<bytes.length;i+=7) {
      let high=0;const low=[];
      for(let j=0;j<7&&i+j<bytes.length;j++){high|=((bytes[i+j]>>7)&1)<<j;low.push(bytes[i+j]&127);}
      out.push(high,...low);
    }
    return new Uint8Array([...out,0xf7]);
  }
  function decode(data) {
    if(data.length<7||data.length>MAX_BYTES*8/7+7||!PREFIX.every((v,i)=>data[i]===v)||data[data.length-1]!==0xf7)return null;
    const bytes=[];
    for(let i=5;i<data.length-1;){const high=data[i++];if(high>127)return null;for(let j=0;j<7&&i<data.length-1;j++){if(data[i]>127)return null;bytes.push(data[i++]|(((high>>j)&1)<<7));}}
    if(bytes.length>MAX_BYTES)return null;
    try {const v=JSON.parse(new TextDecoder('utf-8',{fatal:true}).decode(new Uint8Array(bytes)));return v&&typeof v==='object'&&!Array.isArray(v)?v:null;} catch{return null;}
  }
  function updateState(device, release) {
    if(!device)return {label:'Update',enabled:false,message:'Connect to check your firmware.'};
    if(!release)return {label:'Update',enabled:false,message:'No firmware release is available yet.'};
    if(!Number.isSafeInteger(device.build)||!Number.isSafeInteger(release.build))return {label:'Update',enabled:false,message:'Firmware version could not be verified.'};
    if(device.build>release.build)return {label:'Update',enabled:false,message:'Your device has newer firmware than the website.'};
    if(device.build===release.build){if(device.digest!==release.imageSha256)return {label:'Update',enabled:false,message:'This version does not match the published release.'};return {label:'Re-install',enabled:true,message:'Your Bearamin is up to date.'};}
    return {label:'Update',enabled:true,message:'New firmware is ready for your Bearamin.'};
  }
  function validIdentity(v) {
    return v&&v.protocol===1&&['lite','virtual'].includes(v.model)&&typeof v.uid==='string'&&v.uid.length>0&&v.uid.length<=96&&typeof v.hardware==='string'&&typeof v.version==='string'&&Number.isSafeInteger(v.build)&&v.build>=0&&v.capabilities&&typeof v.capabilities==='object';
  }
  root.BearaminProtocol={encode,decode,updateState,validIdentity};
  if(typeof module!=='undefined')module.exports=root.BearaminProtocol;
})(typeof window!=='undefined'?window:globalThis);
