#!/usr/bin/env python3
"""Pin and MIDI contracts for the RP2040-Zero board."""
from pathlib import Path
import os
import re
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
HARDWARE = ROOT / 'hw/bearamin-virtual'
fork = (HARDWARE / 'Bearamin-Virtual_2040.kicad_pcb').read_text()
assert '(footprint "Bearamin_Virtual:RP2040-Zero_1x23"' in fork
module = fork.split('(footprint "Bearamin_Virtual:RP2040-Zero_1x23"', 1)[1]
pads = []
for match in re.finditer(r'\(pad "(\d+)"', module):
    pad = module[match.start():module.index('\n\t\t)', match.start()) + 4]
    position = re.search(r'\(at ([\d.]+) (-?[\d.]+)\)', pad)
    net = re.search(r'\(net "(\w+)"\)', pad)
    pads.append((match.group(1), *position.groups(), net.group(1) if net else ''))
    if len(pads) == 23:
        break
assert len(pads) == 23, pads
assert {int(pin) for pin, *_ in pads} == set(range(1, 24))
assert dict((pin, net) for pin, _, _, net in pads if net) == {
    '13': 'SDA', '10': 'SCL', '15': 'LPR', '14': 'INT',
    '21': '3V3', '22': 'GND'}
assert pads[0][1:3] == ('16.62', '-1.59')
assert pads[-1][1:3] == ('1.38', '-1.59')

# Confirmed direct supply (SUPPLY_LINKS.md): U2.21 feeds the sensor rail,
# R1/R2 are omitted, and GP2 is unused. Logical-pad checks are not copper proof.
def block(source, start):
    depth = 0
    quoted = escaped = False
    for i in range(start, len(source)):
        c = source[i]
        if quoted:
            if escaped:
                escaped = False
            elif c == "\\":
                escaped = True
            elif c == '"':
                quoted = False
        elif c == '"':
            quoted = True
        elif c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return source[start:i + 1]
    raise AssertionError('unterminated KiCad expression')


def footprint(reference):
    at = fork.index(f'(property "Reference" "{reference}"')
    return block(fork, fork.rindex('\n\t(footprint ', 0, at) + 2)


def pad_net(part, number):
    start = part.index(f'(pad "{number}" ')
    match = re.search(r'\(net "([^\"]+)"\)', block(part, start))
    return match.group(1) if match else ''


references = set(re.findall(r'\(property "Reference" "([^"]+)"', fork))
assert not references.intersection({'R1', 'R2'}), 'R1/R2 must remain omitted'
assert pad_net(footprint('U2'), '3') == '', 'GP2 must remain unconnected'
assert pad_net(footprint('U2'), '23') == '', 'VSYS must remain unconnected'
assert pad_net(footprint('J1'), '1') == '3V3'
assert pad_net(footprint('J1'), '2') == 'GND'
for ref in ('C1', 'C2', 'C3'):
    assert pad_net(footprint(ref), '1') == '3V3', ref
    assert pad_net(footprint(ref), '2') == 'GND', ref
assert 'constexpr uint kSda = 12, kScl = 9, kLpn = 14, kInt = 13;' in (
    ROOT / 'firmware/BearaminVirtual2040/main.cpp').read_text()

source = r'''
#include "midi_map.h"
#include <cassert>
#include <vector>
#include <array>
int main() {
  assert(bearamin_midi::sevenBit(0) == 0);
  assert(bearamin_midi::sevenBit(4095) == 127);
  std::vector<std::array<int,3>> messages;
  auto send = [&](uint8_t a, uint8_t b, uint8_t c) { messages.push_back({a,b,c}); };
  bool held = false;
  bearfield::ProcessedFieldState f{};
  f.present = true; f.xNorm12 = 0; f.yNorm12 = 2048; f.zNorm12 = 4095;
  f.spankStarted = true; f.spankHeld = true;
  bearamin_midi::publish(f, held, send);
  assert(messages.size() == 5 && messages[0] == (std::array<int,3>{0xB0,20,0}));
  assert(messages[1][1] == 21 && messages[1][2] == 64);
  assert(messages[2] == (std::array<int,3>{0xB0,22,127}));
  assert(messages[3] == (std::array<int,3>{0xB0,23,127}));
  assert(messages[4] == (std::array<int,3>{0x90,60,100}) && held);
  messages.clear(); f.spankStarted = false; f.spankHeld = false;
  bearamin_midi::publish(f, held, send);
  assert(messages.back() == (std::array<int,3>{0x80,60,0}) && !held);
  messages.clear(); bearamin_midi::silence(held, send);
  assert(messages.size() == 1 && messages[0] == (std::array<int,3>{0xB0,23,0}));
}
'''
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory)
    (path / 'test.cpp').write_text(source)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-I', str(ROOT / 'firmware/BearaminVirtual2040'),
                    '-I', str(ROOT / 'third_party/bearfield-core/include'),
                    str(path / 'test.cpp'), '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)
    (path / 'payload.bin').write_bytes(bytes(range(255)) * 2)
    subprocess.run(['python3', str(ROOT / 'firmware/BearaminVirtual2040/bin_to_uf2.py'),
                    str(path / 'payload.bin'), str(path / 'image.uf2')], check=True)
    image = (path / 'image.uf2').read_bytes()
    assert len(image) == 1024
    assert struct.unpack('<8I', image[:32]) == (
        0x0A324655, 0x9E5D5157, 0x2000, 0x10000000, 256, 0, 2, 0xE48BFF56)
    assert struct.unpack('<I', image[508:512])[0] == 0x0AB16F30
print('RP2040 footprint and MIDI mapping passed')
