#!/usr/bin/env python3
"""Validate and assemble Field Guide's public files. Dry-run unless --write."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[1]
FILES = ('index.html', 'style.css', 'app.js', 'protocol.js', 'panel.png', 'firmware/latest.json', 'install.html', 'install.css', 'install.js', 'install-protocol.js', 'installer/family.json')


def package(destination, write=False):
    catalog = json.loads((SOURCE / 'firmware/latest.json').read_text())
    if catalog.get('schema') != 1 or not isinstance(catalog.get('products'), dict):
        raise ValueError('Invalid release catalog')
    files = list(FILES)
    for release in catalog['products'].values():
        path = SOURCE / release['url']
        if not path.resolve().is_relative_to((SOURCE / 'firmware').resolve()) or path.is_symlink():
            raise ValueError('Release payload must be a regular file beneath firmware/')
        data = path.read_bytes()
        if len(data) != release['size'] or hashlib.sha256(data).hexdigest() != release['sha256']:
            raise ValueError('Release payload hash or size mismatch')
        files.append(str(path.relative_to(SOURCE)))
    for relative in files:
        src = SOURCE / relative
        if not src.is_file() or src.is_symlink():
            raise ValueError('Missing or non-ordinary public file: ' + relative)
        if write:
            target = destination / relative
            if target.is_symlink() or not target.resolve().is_relative_to(destination.resolve()):
                raise ValueError('Output path escapes destination')
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src, target)
        print(('WRITE ' if write else 'PLAN  ') + relative)
    if write:
        for entry in ('index.html', 'install.html'):
            index = destination / entry
            text = index.read_text()
            for name in ('app.js', 'protocol.js', 'style.css', 'install.js', 'install-protocol.js', 'install.css'):
                digest = hashlib.sha256((destination / name).read_bytes()).hexdigest()[:16]
                text = text.replace('"' + name + '"', '"' + name + '?v=' + digest + '"')
            index.write_text(text)
    # No removal: superseded public firmware retirement is a separate qualified step.


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    package(args.output.resolve(), args.write)
