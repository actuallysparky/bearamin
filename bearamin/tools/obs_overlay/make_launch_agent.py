#!/usr/bin/env python3
"""Render a portable OBS LaunchAgent template; does not install or start it."""
import argparse,plistlib,sys
from pathlib import Path
root=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,default=root/'_local/com.bearamin.obs-overlay.plist');p.add_argument('--python',default=sys.executable);a=p.parse_args()
j=plistlib.loads((root/'com.bearamin.obs-overlay.plist').read_bytes());logs=a.output.resolve().parent/'logs';logs.mkdir(parents=True,exist_ok=True)
j['ProgramArguments']=[a.python,str(root/'bearamin_obs_bridge.py'),'--mapping-value-mode','modulation'];j['StandardOutPath']=str(logs/'obs-overlay.log');j['StandardErrorPath']=str(logs/'obs-overlay-error.log');a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_bytes(plistlib.dumps(j));print(a.output)
