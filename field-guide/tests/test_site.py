import hashlib,json,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from build_site import read_release
class ReleaseSafety(unittest.TestCase):
 def fixture(self,p):
  data=b'known firmware bytes';(p/'app.bin').write_bytes(data)
  m={'schema':1,'product':'virtual','build':10,'version':'0.3.0.2','files':{'app.bin':{'size':len(data),'sha256':hashlib.sha256(data).hexdigest()}}};(p/'metadata.json').write_text(json.dumps(m));return m
 def test_tampered_image_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);self.fixture(p);(p/'app.bin').write_bytes(b'changed firmware bytes')
   with self.assertRaises(ValueError):read_release(p,'virtual')
 def test_path_escape_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);m=self.fixture(p);m['files']['../app.bin']=m['files'].pop('app.bin');(p/'metadata.json').write_text(json.dumps(m))
   with self.assertRaises(ValueError):read_release(p,'virtual')
 def test_wrong_product_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);self.fixture(p)
   with self.assertRaises(ValueError):read_release(p,'lite')
 def test_symlink_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);self.fixture(p);(p/'real.bin').write_bytes((p/'app.bin').read_bytes());(p/'app.bin').unlink();(p/'app.bin').symlink_to(p/'real.bin')
   with self.assertRaises(ValueError):read_release(p,'virtual')
if __name__=='__main__':unittest.main()
