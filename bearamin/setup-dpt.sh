#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
OUT="${DPT_PLATFORM_ROOT:-$ROOT/_local/dpt-platform}"
if [[ -e "$OUT" ]]; then echo "DPT checkout already exists: $OUT; preserve it and verify its revision."; exit 0; fi
mkdir -p "$(dirname "$OUT")"
git clone https://github.com/joemisra/dpt.git "$OUT"
git -C "$OUT" checkout ade2b14b0e4a0d274d75c4a24df95b1ba579747d
git -C "$OUT" submodule update --init --recursive
git -C "$OUT/libDaisy" fetch origin cc146d5065dd8286078a662e2830bf820c37a612
git -C "$OUT/libDaisy" checkout cc146d5065dd8286078a662e2830bf820c37a612
git -C "$OUT/libDaisy" submodule update --init --recursive
git -C "$OUT" apply "$ROOT/firmware/patches/dpt-platform.patch"
git -C "$OUT/libDaisy" apply "$ROOT/firmware/patches/libdaisy-bounded-sd.patch"
