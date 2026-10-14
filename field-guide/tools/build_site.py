#!/usr/bin/env python3
"""Assemble the shared public installer from verified current release assets."""
import argparse, hashlib, json, re, shutil, subprocess, tempfile, time
from pathlib import Path
PRODUCTS={'lite':'bearamin-lite-','virtual':'bearamin-virtual-','full':'bearamin-beta.'}
STATIC=('style.css','app.js','protocol.js','install-protocol.js','install.css','panel.png')
def read_release(directory, product):
 m=json.loads((directory/'metadata.json').read_text())
 if m.get('schema')!=1 or m.get('product')!=product or type(m.get('build')) is not int or m['build']<0:raise ValueError('Invalid release identity')
 if not isinstance(m.get('files'),dict) or not m['files']:raise ValueError('Missing release payloads')
 for name,claim in m['files'].items():
  if not re.fullmatch(r'[A-Za-z0-9_.-]+',name):raise ValueError('Unsafe release filename')
  p=directory/name
  if not p.is_file() or p.is_symlink() or not 0<p.stat().st_size<=32*1024*1024:raise ValueError('Invalid release payload')
  b=p.read_bytes()
  if len(b)!=claim['size'] or hashlib.sha256(b).hexdigest()!=claim['sha256']:raise ValueError('Release size/hash mismatch')
 return m

def assemble(source, releases, destination):
 lite=read_release(releases/'lite','lite');virtual=read_release(releases/'virtual','virtual')
 # Validate the declared hardware and image layout, independently of catalog text.
 lr=lite['installer'];vr=virtual['installer'];bridge=lite['companions']['bridge']
 if lr['target']!='bearamin-lite-midi-prototype' or lr['hardware']!=['bearcv-dual-mcp4728-v1'] or lr['url']!='installer/Bearamin-Lite-MIDI.bin':raise ValueError('Wrong Lite target')
 if (vr['target'],vr['hardware'],vr['url'],vr['method'])!=('bearamin-virtual-rp2040','waveshare-rp2040-zero','installer/Bearamin-Virtual.uf2','uf2'):raise ValueError('Wrong Virtual target')
 if (bridge['target'],bridge['hardware'],bridge['url'])!=('tdongle-wireless-midi','lilygo-tdongle-s3','installer/T-Dongle-MIDI.bin'):raise ValueError('Wrong companion target')
 import struct
 full=(releases/'lite/Bearamin-Lite-MIDI.bin').read_bytes();app=(releases/'lite/Bearamin-Lite-MIDI-app.bin').read_bytes()
 if len(full)!=4194304 or full[0]!=0xe9 or struct.unpack_from('<H',full,12)[0]!=9 or full[0x10000:0x10000+len(app)]!=app or hashlib.sha256(app[:-32]).digest()!=app[-32:] or lr['imageSha256']!=app[-32:].hex():raise ValueError('Invalid Lite image')
 # Reuse the firmware-specific UF2 / MIDI ABI / bridge partition validation.
 from family_artifacts import validate
 with tempfile.TemporaryDirectory() as tmp:
  p=Path(tmp)
  for r,d in [(vr,releases/'virtual'),(bridge,releases/'lite')]:
   name=Path(r['url']).name;shutil.copyfile(d/name,p/name)
  for name,d in [('Bearamin-Virtual-MIDI.bin',releases/'virtual'),('TDongleBridge.bin',releases/'lite'),('partition-table.bin',releases/'lite')]:shutil.copyfile(d/name,p/name)
  (p/'family.json').write_text(json.dumps({'schema':1,'products':{'virtual':vr,'bridge':bridge}}));validate(p)
 destination.mkdir(parents=True,exist_ok=True)
 for name in STATIC:shutil.copyfile(source/name,destination/name)
 # Root is the friendly installer; guide.html retains the musical settings page.
 (destination/'guide.html').write_text((source/'index.html').read_text())
 for name in ('index.html','install.html'):(destination/name).write_text((source/'install.html').read_text().replace('href="index.html','href="guide.html'))
 (destination/'install.js').write_text((source/'install.js').read_text().replace('index.html?','guide.html?'))
 (destination/'installer').mkdir(exist_ok=True);(destination/'firmware').mkdir(exist_ok=True)
 for r,d in [(lr,releases/'lite'),(vr,releases/'virtual'),(vr['midi'],releases/'virtual'),(bridge,releases/'lite')]:
  name=Path(r['url']).name;data=(d/name).read_bytes()
  if len(data)!=r['size'] or hashlib.sha256(data).hexdigest()!=r['sha256']:raise ValueError('Catalog does not match payload')
  (destination/r['url']).write_bytes(data)
 (destination/'installer/release.json').write_text(json.dumps(lr,indent=2)+'\n')
 (destination/'installer/family.json').write_text(json.dumps({'schema':1,'products':{'virtual':vr,'bridge':bridge}},indent=2)+'\n')
 (destination/'firmware/latest.json').write_text('{"schema":1,"products":{}}\n')
 for name in ('index.html','install.html','guide.html'):
  p=destination/name;s=p.read_text()
  for asset in (*STATIC,'install.js'):
   if (destination/asset).is_file():s=s.replace('"'+asset+'"','"'+asset+'?v='+hashlib.sha256((destination/asset).read_bytes()).hexdigest()[:16]+'"')
  p.write_text(s)
 (destination/'.nojekyll').touch()
 print('Verified latest installer: Lite '+lr['version']+', Virtual '+vr['version'])

def download_latest(repo, destination):
 all_releases=json.loads(subprocess.check_output(['gh','api','--paginate','--slurp',f'repos/{repo}/releases'],text=True))
 items=[r for page in all_releases for r in page if not r['draft']]
 for product,prefix in PRODUCTS.items():
  if product=='full':continue # Full keeps its own sidecar UI.
  candidates=[]
  for r in items:
   if not r['tag_name'].startswith(prefix):continue
   with tempfile.TemporaryDirectory() as tmp:
    subprocess.run(['gh','release','download',r['tag_name'],'--repo',repo,'--pattern','metadata.json','--dir',tmp],check=True,stdout=subprocess.DEVNULL)
    m=json.loads((Path(tmp)/'metadata.json').read_text());candidates.append((m['build'],m['version'],r,m))
  if not candidates:raise ValueError('No published '+product+' release')
  # Build ID selection prevents an older version released later becoming current.
  candidates.sort(key=lambda x:(tuple(map(int,re.findall(r"\d+",x[1]))),x[0]));best=candidates[-1]
  for c in candidates:
   if c[0]==best[0] and c[3]['files']!=best[3]['files']:raise ValueError('Conflicting firmware under one build ID')
   if c[1]==best[1] and c[3]['files']!=best[3]['files']:raise ValueError('Conflicting firmware under one version')
  d=destination/product;d.mkdir(parents=True,exist_ok=True)
  subprocess.run(['gh','release','download',best[2]['tag_name'],'--repo',repo,'--dir',str(d)],check=True,stdout=subprocess.DEVNULL)
  read_release(d,product)

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--releases',type=Path);p.add_argument('--github');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 source=Path(__file__).resolve().parents[1]
 if a.github:
  with tempfile.TemporaryDirectory() as tmp:
   d=Path(tmp);download_latest(a.github,d);assemble(source,d,a.output)
 else:assemble(source,a.releases,a.output)
