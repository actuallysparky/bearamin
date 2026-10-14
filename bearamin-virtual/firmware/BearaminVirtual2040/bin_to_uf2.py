#!/usr/bin/env python3
"""Wrap an RP2040 XIP binary in UF2 blocks for USB BOOTSEL loading."""
from pathlib import Path
import argparse
import struct

parser = argparse.ArgumentParser()
parser.add_argument('binary', type=Path)
parser.add_argument('uf2', type=Path)
parser.add_argument('--clear-update-intent',action='store_true')
args = parser.parse_args()
data = args.binary.read_bytes()
application_blocks = (len(data) + 255) // 256
blocks = application_blocks + (16 if args.clear_update_intent else 0)
with args.uf2.open('wb') as output:
    for index in range(blocks):
        payload = data[index * 256:(index + 1) * 256].ljust(256, b'\0') if index<application_blocks else b'\xff'*256
        address=0x10000000+index*256 if index<application_blocks else 0x101fd000+(index-application_blocks)*256
        header = struct.pack('<8I', 0x0A324655, 0x9E5D5157, 0x2000,
                             address, 256, index, blocks,
                             0xE48BFF56)
        output.write(header + payload + bytes(476 - 256) + struct.pack('<I', 0x0AB16F30))
