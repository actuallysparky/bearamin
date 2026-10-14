#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_ROOT="${BUILD_ROOT:-$PROJECT_ROOT/_local/test}"
mkdir -p "$BUILD_ROOT"
c++ -std=c++17 -Wall -Wextra -Werror \
  "$PROJECT_ROOT/test/test_bearfield.cpp" \
  -o "$BUILD_ROOT/test_bearfield"
"$BUILD_ROOT/test_bearfield" "$PROJECT_ROOT/fixtures/bearamin-v1-migration-smoke.csv"
echo "bearfield-core tests passed"
