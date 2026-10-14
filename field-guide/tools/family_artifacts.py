"""Validate public Virtual/bridge installer artifacts before publication."""
import hashlib
import json
import struct
import re
from pathlib import Path
SPECS = {
    'virtual': ('bearamin-virtual-rp2040', 'waveshare-rp2040-zero', 'installer/Bearamin-Virtual.uf2', 'uf2'),
    'bridge': ('tdongle-wireless-midi', 'lilygo-tdongle-s3', 'installer/T-Dongle-MIDI.bin', 'esp'),
}

def payload_from_uf2(data):
    return b"".join(data[i+32:i+288] for i in range(0,len(data),512))

def validate(directory):
    catalog = json.loads((directory / 'family.json').read_text())
    if catalog.get('schema') != 1 or set(catalog.get('products', {})) != set(SPECS):
        raise ValueError('Incomplete family installer catalog')
    artifacts = []
    for model, (target, hardware, url, method) in SPECS.items():
        r = catalog['products'][model]
        if (r.get('target'), r.get('hardware'), r.get('url'), r.get('method')) != (target, hardware, url, method) or r.get('schema') != 1 or not isinstance(r.get('version'),str) or not re.fullmatch(r'\d{1,3}(?:\.\d{1,3}){3}',r['version']) or any(int(x)>127 for x in r['version'].split('.')) or type(r.get('build')) is not int or r['build'] < 0:
            raise ValueError('Wrong family artifact identity')
        path = directory / Path(url).name
        if path.is_symlink():
            raise ValueError('Symlink firmware rejected')
        data = path.read_bytes()
        if len(data) != r['size'] or hashlib.sha256(data).hexdigest() != r['sha256']:
            raise ValueError('Family artifact hash/size mismatch')
        if model == 'virtual':
            if not data or len(data) % 512 or len(data) > 4*1024*1024:
                raise ValueError('Invalid UF2 size')
            blocks = len(data)//512
            for i in range(blocks):
                block = data[i*512:(i+1)*512]
                header = struct.unpack_from('<8I', block)
                address=0x101fd000+(i-(blocks-16))*256 if r.get("midi") and i>=blocks-16 else 0x10000000+i*256
                if r.get("midi") and i>=blocks-16 and block[32:288]!=b"\xff"*256:raise ValueError("UF2 must clear stale update intent")
                if header != (0x0A324655, 0x9E5D5157, 0x2000, address, 256, i, blocks, 0xE48BFF56) or struct.unpack_from('<I',block,508)[0] != 0x0AB16F30:
                    raise ValueError('UF2 target/layout mismatch')
            if r.get('midi'):
                m=r['midi']
                if (m.get('target'),m.get('hardware'),m.get('abi'),m.get('address'),m.get('method'),m.get('url'),m.get('version'))!=('bearamin-virtual-rp2040','waveshare-rp2040-zero',1,0x10004000,'sysex','installer/Bearamin-Virtual-MIDI.bin',r['version']):raise ValueError('Wrong MIDI update target/layout')
                raw=(directory/'Bearamin-Virtual-MIDI.bin').read_bytes()
                if len(raw)!=m['size'] or not 512<=len(raw)<=0x100000-0x4000 or hashlib.sha256(raw).hexdigest()!=m['sha256'] or b'BEARAMIN-RP2040-MIDI-ABI1\0' not in raw or (m['version']+'\0').encode() not in raw:raise ValueError('MIDI update payload verification failed')
                if payload_from_uf2(data)[0x4000:0x4000+len(raw)]!=raw:raise ValueError('MIDI payload differs from UF2 application')
                sp,pc=struct.unpack_from('<II',raw,256)
                if not 0x20000000<=sp<=0x20042000 or sp%4 or not pc&1 or not 0x10004100<=pc<0x10004000+len(raw):raise ValueError('Invalid application vectors')
                artifacts.append((m,raw))
        else:
            if len(data) != 16*1024*1024 or data[0] != 0xe9 or struct.unpack_from('<H',data,12)[0] != 9:
                raise ValueError('Invalid T-Dongle ESP32-S3 image')
            # Publication inputs include matching application/partition hashes.
            for name, offset in [('TDongleBridge.bin',0x10000),('partition-table.bin',0x8000)]:
                payload=(directory/name).read_bytes()
                if not payload or data[offset:offset+len(payload)] != payload:
                    raise ValueError('Bridge image layout mismatch')
            app=(directory/'TDongleBridge.bin').read_bytes()
            if hashlib.sha256(app[:-32]).digest()!=app[-32:]:
                raise ValueError('Bridge application digest mismatch')
        artifacts.append((r, data))
    return catalog, artifacts
