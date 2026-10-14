import hashlib
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from family_artifacts import SPECS, validate
class FamilyArtifacts(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir='/private/tmp');self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        block=bytearray(512);struct.pack_into('<8I',block,0,0x0A324655,0x9E5D5157,0x2000,0x10000000,256,0,1,0xE48BFF56);struct.pack_into('<I',block,508,0x0AB16F30)
        app=b'A'*64;app+=hashlib.sha256(app).digest();parts=b'P'*32;full=bytearray(16*1024*1024);full[0]=0xe9;struct.pack_into('<H',full,12,9);full[0x10000:0x10000+len(app)]=app;full[0x8000:0x8000+len(parts)]=parts
        (self.root/'TDongleBridge.bin').write_bytes(app);(self.root/'partition-table.bin').write_bytes(parts)
        products={}
        for model,data in [('virtual',block),('bridge',full)]:
            target,hw,url,method=SPECS[model];(self.root/Path(url).name).write_bytes(data)
            products[model]=dict(schema=1,target=target,hardware=hw,url=url,method=method,version='0.2.0.1',build=1,size=len(data),sha256=hashlib.sha256(data).hexdigest())
        self.catalog=dict(schema=1,products=products);self.write()
    def write(self):(self.root/'family.json').write_text(json.dumps(self.catalog))
    def test_valid_and_wrong_model(self):
        self.assertEqual(len(validate(self.root)[1]),2)
        self.catalog['products']['virtual']['hardware']='lilygo-tdongle-s3';self.write()
        with self.assertRaises(ValueError):validate(self.root)
    def test_uf2_wrong_chip_even_with_matching_hash(self):
        path=self.root/'Bearamin-Virtual.uf2';data=bytearray(path.read_bytes());struct.pack_into('<I',data,28,0xdeadbeef);path.write_bytes(data)
        self.catalog['products']['virtual']['sha256']=hashlib.sha256(data).hexdigest();self.write()
        with self.assertRaises(ValueError):validate(self.root)
    def test_bridge_partition_mismatch(self):
        (self.root/'partition-table.bin').write_bytes(b'wrong')
        with self.assertRaises(ValueError):validate(self.root)
    def test_midi_payload_matches_uf2_and_rejects_stale_install_intent(self):
        raw=bytearray(512);struct.pack_into('<II',raw,256,0x20042000,0x10004101);raw[300:300+len(b'BEARAMIN-RP2040-MIDI-ABI1\0')]=b'BEARAMIN-RP2040-MIDI-ABI1\0';raw[400:408]=b'0.3.0.2\0'
        payload=bytes(0x4000)+raw;count=len(payload)//256+16;uf2=bytearray()
        for i in range(count):
            block=bytearray(512);address=0x10000000+i*256 if i<count-16 else 0x101fd000+(i-(count-16))*256
            struct.pack_into('<8I',block,0,0x0A324655,0x9E5D5157,0x2000,address,256,i,count,0xE48BFF56);block[32:288]=payload[i*256:(i+1)*256] if i<count-16 else b'\xff'*256;struct.pack_into('<I',block,508,0x0AB16F30);uf2+=block
        r=self.catalog['products']['virtual'];r.update(version='0.3.0.2',size=len(uf2),sha256=hashlib.sha256(uf2).hexdigest());r['midi']=dict(schema=1,target=r['target'],hardware=r['hardware'],abi=1,address=0x10004000,method='sysex',url='installer/Bearamin-Virtual-MIDI.bin',version=r['version'],build=2,size=len(raw),sha256=hashlib.sha256(raw).hexdigest())
        (self.root/'Bearamin-Virtual.uf2').write_bytes(uf2);(self.root/'Bearamin-Virtual-MIDI.bin').write_bytes(raw);self.write();self.assertEqual(len(validate(self.root)[1]),3)
        uf2[-512+32]=0;(self.root/'Bearamin-Virtual.uf2').write_bytes(uf2);r['sha256']=hashlib.sha256(uf2).hexdigest();self.write()
        with self.assertRaisesRegex(ValueError,'stale update intent'):validate(self.root)
if __name__=='__main__':unittest.main()
