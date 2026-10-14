import hashlib
import importlib.util
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
TOOLS=Path(__file__).resolve().parents[1]/'tools'
sys.path.insert(0,str(TOOLS))
import publish_lite

class PublisherTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(dir='/private/tmp');self.addCleanup(self.temp.cleanup)
        self.build=Path(self.temp.name)/'build';self.build.mkdir();self.site=Path(self.temp.name)/'site'
        full=bytearray(4194304);full[0]=0xe9;struct.pack_into('<H',full,12,9);app=b'A'*64;app+=hashlib.sha256(app).digest();parts=b'P'*32;full[0x10000:0x10000+len(app)]=app;full[0x8000:0x8000+len(parts)]=parts
        self.meta=dict(target='bearamin-lite-midi-prototype',hardware='bearcv-dual-mcp4728-v1',version='0.2.0-prototype.123',build=123,manual_full_flash_offset='0x0',app_offset='0x10000',files={})
        for name,data in [('Bearamin-Lite-MIDI--full.bin',full),('Bearamin-Lite-MIDI--app.bin',app),('Bearamin-Lite-MIDI--partitions.bin',parts)]:
            (self.build/name).write_bytes(data);self.meta['files'][name]=dict(size=len(data),sha256=hashlib.sha256(data).hexdigest())
        self.write()
    def write(self):(self.build/'manifest.json').write_text(json.dumps(self.meta))
    def test_valid_current_and_portable_paths(self):
        release=publish_lite.stage(self.build,self.site)
        self.assertEqual(release['url'],'installer/Bearamin-Lite-MIDI.bin');self.assertEqual((self.site/release['url']).stat().st_size,4194304)
        self.assertNotIn('commit',json.loads((self.site/'installer/release.json').read_text()));self.assertFalse((self.site/'AGENTS.md').exists())
        self.assertIn('install.js?v=',(self.site/'install.html').read_text());self.assertEqual(json.loads((self.site/'firmware/latest.json').read_text())['products'],{})
    def test_retains_family_catalog_on_lite_only_publication(self):
        publish_lite.stage(self.build,self.site)
        data=b'firmware';path=self.site/'installer/Bearamin-Virtual.uf2';path.write_bytes(data)
        catalog={'schema':1,'products':{'virtual':{'url':'installer/Bearamin-Virtual.uf2','size':len(data),'sha256':hashlib.sha256(data).hexdigest()}}}
        (self.site/'installer/family.json').write_text(json.dumps(catalog))
        publish_lite.stage(self.build,self.site)
        self.assertEqual(json.loads((self.site/'installer/family.json').read_text()),catalog)
    def test_bad_hash_and_wrong_target_do_not_replace_website(self):
        publish_lite.stage(self.build,self.site);prior=(self.site/'installer/release.json').read_bytes()
        (self.build/'Bearamin-Lite-MIDI--app.bin').write_bytes(b'bad')
        with self.assertRaises(ValueError):publish_lite.stage(self.build,self.site)
        self.assertEqual((self.site/'installer/release.json').read_bytes(),prior)
        self.meta['target']='virtual';self.write()
        with self.assertRaises(ValueError):publish_lite.stage(self.build,self.site)
    def test_no_downgrade_or_conflicting_build(self):
        publish_lite.stage(self.build,self.site);self.meta['build']=122;self.write()
        with self.assertRaises(ValueError):publish_lite.stage(self.build,self.site)
        self.meta['build']=123;self.write();old=json.loads((self.site/'installer/release.json').read_text());old['sha256']='0'*64;(self.site/'installer/release.json').write_text(json.dumps(old))
        with self.assertRaises(ValueError):publish_lite.stage(self.build,self.site)

if __name__=='__main__':unittest.main()
