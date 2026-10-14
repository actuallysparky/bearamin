#!/usr/bin/env python3
"""Verify a completed build and package public release assets."""
import argparse, hashlib, json, re, shutil, struct, time
from pathlib import Path

def prepare(product,build,output,root,companions=None,entry=None,version=None,number=None):
 output.mkdir(parents=True,exist_ok=True);files={}
 def add(src,name):
  if src.is_symlink():raise ValueError('Symlink build artifact')
  data=src.read_bytes()
  if b'/Users/' in data or b'AGENTS.md' in data:raise ValueError('Private machine data in firmware; rebuild with normalized paths')
  (output/name).write_bytes(data);files[name]={'size':len(data),'sha256':hashlib.sha256(data).hexdigest()};return data
 m={'schema':1,'product':product,'files':files,'build':number or int(time.time()),'version':version}
 if product=='lite':
  source=json.loads((build/'manifest.json').read_text());m['version']=source['version'];m['build']=source['build']
  for n,c in source['files'].items():
   data=(build/n).read_bytes()
   if hashlib.sha256(data).hexdigest()!=c['sha256'] or len(data)!=c['size']:raise ValueError('Build manifest mismatch')
  full=add(build/'Bearamin-Lite-MIDI--full.bin','Bearamin-Lite-MIDI.bin');app=add(build/'Bearamin-Lite-MIDI--app.bin','Bearamin-Lite-MIDI-app.bin');add(build/'Bearamin-Lite-MIDI--partitions.bin','Bearamin-Lite-MIDI-partitions.bin')
  if len(full)!=4194304 or full[0]!=0xe9 or struct.unpack_from('<H',full,12)[0]!=9 or full[0x10000:0x10000+len(app)]!=app or hashlib.sha256(app[:-32]).digest()!=app[-32:]:raise ValueError('Invalid Lite build')
  m['installer']={'hardware':['bearcv-dual-mcp4728-v1'],'imageSha256':app[-32:].hex(),'schema':1,'target':'bearamin-lite-midi-prototype','version':m['version'],'build':m['build'],'chip':'ESP32-S3','offset':0,'size':len(full),'sha256':hashlib.sha256(full).hexdigest(),'url':'installer/Bearamin-Lite-MIDI.bin'}
  catalog=json.loads((companions/'family.json').read_text());m['companions']={'bridge':catalog['products']['bridge']}
  for name in ['T-Dongle-MIDI.bin','TDongleBridge.bin','partition-table.bin']:add(companions/name,name)
 elif product=='virtual':
  if entry:
   v=json.loads(entry.read_text());m['installer']=v;m['version']=v['version'];m['build']=v['build']
   add(build/'bearamin_virtual_2040.uf2','Bearamin-Virtual.uf2');add(build/'Bearamin-Virtual-MIDI.bin','Bearamin-Virtual-MIDI.bin')
  else:
   s=(root/'bearamin-virtual/firmware/BearaminVirtual2040/CMakeLists.txt').read_text();m['version']=re.search(r'set\(BEARAMIN_VERSION "([^"]+)"',s)[1]
   uf2=add(build/'bearamin_virtual_2040.uf2','Bearamin-Virtual.uf2');app=add(build/'Bearamin-Virtual-MIDI.bin','Bearamin-Virtual-MIDI.bin')
   base={'schema':1,'target':'bearamin-virtual-rp2040','hardware':'waveshare-rp2040-zero','version':m['version'],'build':m['build']}
   m['installer']=dict(base,method='uf2',url='installer/Bearamin-Virtual.uf2',size=len(uf2),sha256=hashlib.sha256(uf2).hexdigest(),midi=dict(base,abi=1,address=0x10004000,method='sysex',url='installer/Bearamin-Virtual-MIDI.bin',size=len(app),sha256=hashlib.sha256(app).hexdigest()))
  if not re.fullmatch(r'\d{1,3}(?:\.\d{1,3}){3}',m['version']) or any(int(x)>127 for x in m['version'].split('.')):raise ValueError('Invalid four-part MIDI version')
 elif product=='full':
  if not version:raise ValueError('Full release requires a version')
  for name in ['bearamin-sidecar-s3.bin','bearamin-sidecar-s3--app.bin','bearamin-matrix-vl53l7cx-s3.bin','bearamin-matrix-vl53l7cx-s3--app.bin','bearamin-dpt-multimode.bin']:add(build/name,name)
  for stem,size in [('bearamin-sidecar-s3',8388608),('bearamin-matrix-vl53l7cx-s3',4194304)]:
   full=(output/(stem+'.bin')).read_bytes();app=(output/(stem+'--app.bin')).read_bytes()
   if len(full)!=size or full[0]!=0xe9 or struct.unpack_from('<H',full,12)[0]!=9 or full[0x10000:0x10000+len(app)]!=app or hashlib.sha256(app[:-32]).digest()!=app[-32:]:raise ValueError('Invalid Full Bearamin image')
 else:raise ValueError('Unknown product')
 (output/'metadata.json').write_text(json.dumps(m,indent=2)+'\n')
 (output/'SHA256SUMS').write_text(''.join(f'{v["sha256"]}  {k}\n' for k,v in files.items()))
 print('Verified '+product+' '+m['version']);return m

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('product',choices=['lite','virtual','full']);p.add_argument('--build',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--companions',type=Path);p.add_argument('--entry',type=Path);p.add_argument('--version');p.add_argument('--build-id',type=int);a=p.parse_args();prepare(a.product,a.build,a.output,Path(__file__).resolve().parents[2],a.companions,a.entry,a.version,a.build_id)
