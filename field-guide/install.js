 'use strict';
const installElement=id=>document.getElementById(id), I=BearaminInstall, P=BearaminProtocol;
let firmwareBlobUrl,manifestBlobUrl,selection=0,midiAccess,currentSelection;
const descriptions={
 lite:{name:'Bearamin Lite',file:'Bearamin-Lite-MIDI.bin',note:'For the ESP32-S3 Matrix with two MCP4728 DACs. Installation replaces stored settings and Bluetooth pairings.',steps:['Connect Bearamin Lite with a USB data cable. Use desktop Chrome or Edge.','Close any app using its USB connection. If needed, hold BOOT, press and release RESET, then release BOOT.','Choose Install, select your Bearamin’s USB port, and follow the prompts. Keep it powered and connected.','Restart your Bearamin, then open Field Guide and choose Connect.'],finish:'Bluetooth is available after installation.'},
 virtual:{name:'Bearamin Virtual',file:'Bearamin-Virtual.uf2',note:'For the Waveshare RP2040-Zero carrier. VCV Rack and USB MIDI remain available together.',steps:['Close VCV Rack and any app using your Bearamin.','Unplug USB. Hold BOOT while reconnecting USB, then release BOOT.','Download firmware below. Copy it onto the RPI-RP2 drive that appears on your computer.','Wait for the drive to disappear and your Bearamin to restart. Reconnect it in VCV Rack or your DAW.'],finish:'Virtual connects by USB. This build supports device discovery; open Field Guide to tune your MIDI and gestures.'},
 bridge:{name:'T-Dongle Wireless MIDI',file:'T-Dongle-MIDI.bin',note:'For the LilyGO T-Dongle S3. Installation replaces internal pairing and staging data; configuration on the SD card is retained.',steps:['Use desktop Chrome or Edge. Disconnect the bridge from your DAW and wireless MIDI peer.','Enter USB download mode using the T-Dongle’s BOOT button while connecting USB.','Choose Install, select the T-Dongle USB port, and keep it connected until installation finishes.','Restart the bridge, then reconnect your wireless MIDI peer and DAW.'],finish:'Your little forest accomplice is ready for USB and wireless MIDI.'}
};
function clean(){if(firmwareBlobUrl)URL.revokeObjectURL(firmwareBlobUrl);if(manifestBlobUrl)URL.revokeObjectURL(manifestBlobUrl);firmwareBlobUrl=manifestBlobUrl=null;installElement('usb-install').hidden=true;installElement('firmware-download').hidden=true;installElement('firmware-download').textContent='Download firmware ↓';installElement('install-confirm').checked=false;installElement('install-confirm').disabled=true;installElement('install-button').disabled=true;}
async function prepareInstall(model,device,reinstall=false){
 currentSelection={model,device};installElement('play-guide').textContent='Let’s play ↗';installElement("ready").hidden=true;
 const token=++selection;clean();const d=descriptions[model];if(!d)return;
 installElement('install-illustration').hidden=model!=='lite';installElement('open-guide').hidden=false;
 installElement('installation').hidden=false;installElement('device-label').textContent=d.name.toUpperCase();installElement('install-version').textContent='Checking firmware…';installElement('install-status').textContent='Preparing your installation.';
 installElement('connected-version').textContent=device?'Connected version: '+device.version:'First installation · '+d.name;
 installElement('install-steps').replaceChildren(...d.steps.map(text=>{const li=document.createElement('li');li.textContent=text;return li;}));
 installElement('hardware-note').textContent=(device?.bootstrap?'':'')+d.note;installElement('finish-note').textContent=d.finish;installElement('confirm-text').textContent='I’m installing this on '+d.name+' and understand the installation steps above.';
 try{
  const response=await fetch(model==='lite'?'installer/release.json':'installer/family.json',{cache:'no-store'});if(!response.ok)throw Error('No firmware build is available.');
  const data=await response.json(),release=model==='lite'?I.validate(data):I.validateFamily(data.products?.[model],model);
  if(device&&!device.bootstrap&&device.version===release.version&&(model!=='lite'||P.updateState(device,release).label==='Re-install'&&P.updateState(device,release).enabled)&&!reinstall){
    if(token!==selection)return;
    installElement('installation').hidden=true;installElement('ready').hidden=false;installElement('ready-name').textContent=d.name+' is ready.';installElement('ready-status').textContent=model==='bridge'?'Your MIDI bridge is up to date. Let’s get playing.':'You’re up to date. Shape your sound in Field Guide.';installElement('play-guide').href='index.html?connect=1&model='+model;return;
  }
  if(model==='virtual'&&device?.capabilities?.update==='sysex'&&!reinstall&&I.versionState(device.version,release.version).enabled){
    installElement('installation').hidden=true;installElement('ready').hidden=false;installElement('ready-name').textContent='An update is ready.';installElement('ready-status').textContent='Bring your Bearamin up to date, then get back to playing.';installElement('play-guide').textContent='Update ↗';installElement('play-guide').href='index.html?connect=1&model=virtual&update=1';return;
  }
  const binary=await fetch(release.url,{cache:'no-store'});if(!binary.ok)throw Error('Could not download firmware. Reload and try again.');
  const bytes=await binary.arrayBuffer();await (model==='lite'?I.verify(bytes,release):I.verifyFamily(bytes,release,model));if(token!==selection)return;
  if(device&&!device.bootstrap){const newer=model==='lite'?P.updateState(device,release).enabled===false:device.version.split('.').map(Number).some((n,i,a)=>a.slice(0,i).every((x,j)=>x===Number(release.version.split('.')[j]))&&n>Number(release.version.split('.')[i]));if(newer){installElement('install-status').textContent='Your device has newer or different firmware than this release.';return;}}
  firmwareBlobUrl=URL.createObjectURL(new Blob([bytes],{type:'application/octet-stream'}));
  installElement('install-version').textContent=release.version;installElement('install-status').textContent='Firmware verified. Follow the steps below.';
  installElement('firmware-download').href=firmwareBlobUrl;installElement('firmware-download').download=d.file;
  if(model==='virtual'){installElement('firmware-download').hidden=false;installElement('firmware-download').textContent=device?(device.version===release.version?'Re-install ↓':'Update ↓'):'Download firmware ↓';installElement('install-confirm').disabled=false;return;}
  const manifest=model==='lite'?I.manifest(release,firmwareBlobUrl):{name:d.name,version:release.version,new_install_improv_wait_time:0,new_install_prompt_erase:false,builds:[{chipFamily:'ESP32-S3',parts:[{path:firmwareBlobUrl,offset:0}]}]};
  manifestBlobUrl=URL.createObjectURL(new Blob([JSON.stringify(manifest)],{type:'application/json'}));installElement('usb-install').setAttribute('manifest',manifestBlobUrl);installElement('usb-install').hidden=false;installElement('firmware-download').hidden=false;
  let loaderTimer;try{await Promise.race([customElements.whenDefined('esp-web-install-button'),new Promise((_,reject)=>{loaderTimer=setTimeout(()=>reject(Error('The USB installer could not load. Reload and try again.')),15000);})]);}finally{clearTimeout(loaderTimer);}if(token!==selection)return;
  installElement('install-button').textContent=(device?(device.bootstrap?'Install ':device.version===release.version?'Re-install ':'Update '):'Install ')+d.name;installElement('install-confirm').disabled=false;
  if(!window.isSecureContext||!('serial' in navigator))installElement('install-status').textContent='Use desktop Chrome or Edge on HTTPS to install by USB.';
 }catch(e){if(token===selection){installElement('install-status').textContent=e.message;installElement('install-version').textContent='Installation unavailable';}}
}
async function identify(){
 const status=installElement('discovery-status');installElement('identify').disabled=true;status.textContent='Looking for your Bearamin…';installElement('device-choices').replaceChildren();
 try{
  if(!navigator.requestMIDIAccess||!window.isSecureContext)throw Error('Use desktop Chrome or Edge on HTTPS to connect.');
  midiAccess=await navigator.requestMIDIAccess({sysex:true});const inputs=[...midiAccess.inputs.values()],outputs=[...midiAccess.outputs.values()];const found=[];
  for(const input of inputs)await input.open();
  for(const output of outputs){await output.open();const request=Math.floor(Math.random()*1000000);const device=await new Promise(resolve=>{
   let timer,standardCandidate;const finish=v=>{clearTimeout(timer);for(const input of inputs)input.onmidimessage=null;resolve(v);};
   for(const input of inputs)input.onmidimessage=e=>{const standard=I.familyIdentity(e.data),message=P.decode(e.data);if(standard&&(!/t-dongle/i.test(output.name||'')||standard.model==='bridge')){standardCandidate={...standard,uid:'port:'+output.id};if(standard.model==='bridge')finish(standardCandidate);}else if(message?.request===request&&message.op==='identity'&&P.validIdentity(message.data)&&((message.data.model==='lite'&&message.data.hardware==='bearcv-dual-mcp4728-v1')||(message.data.model==='virtual'&&message.data.hardware==='waveshare-rp2040-zero')))finish(message.data);};
   timer=setTimeout(()=>finish(standardCandidate||null),/bearamin virtual|bearamin midi/i.test(output.name||'')?8000:1300);output.send(new Uint8Array([0xf0,0x7e,0x7f,6,1,0xf7]));if(/bearamin lite|bearamin virtual|bearamin midi|ESP32S3_DEV/i.test(output.name||''))output.send(P.encode({op:'identity',request,data:{protocol:1}}));
  });if(device)found.push(device);}
  const devices=found.filter((d,i,a)=>a.findIndex(x=>d.uid?x.uid===d.uid:false)===i);
  if(!devices.length)throw Error('No compatible device answered. Check your USB or Bluetooth MIDI connection, or use First time here for an initial installation.');
  status.textContent=devices.length===1?'Found '+descriptions[devices[0].model].name+'.':'Choose the device you want to install.';
  if(devices.length===1)await prepareInstall(devices[0].model,devices[0]);else for(const d of devices){const b=document.createElement('button');b.className='button subtle';b.textContent=descriptions[d.model].name+' · '+d.version;b.onclick=()=>prepareInstall(d.model,d);installElement('device-choices').append(b);}
 }catch(e){status.textContent=e.name==='NotAllowedError'?'Allow MIDI access to find your Bearamin.':e.message;}finally{if(midiAccess){for(const p of midiAccess.inputs.values()){p.onmidimessage=null;await p.close();}for(const p of midiAccess.outputs.values())await p.close();}installElement('identify').disabled=false;}
}
installElement('identify').addEventListener('click',identify);
installElement('choose-lite').addEventListener('click',()=>prepareInstall('lite'));
installElement('choose-virtual').addEventListener('click',()=>prepareInstall('virtual'));
installElement('install-confirm').addEventListener('change',e=>{installElement('install-button').disabled=!e.target.checked;});
window.addEventListener('pagehide',clean);

installElement("show-reinstall").addEventListener("click",()=>{if(currentSelection?.device?.capabilities?.update==='sysex'){location.href='index.html?connect=1&model=virtual&update=1&reinstall=1';}else if(currentSelection)prepareInstall(currentSelection.model,currentSelection.device,true);});

if(typeof location!=="undefined"&&new URLSearchParams(location.search).get("connect")==="1")identify();
