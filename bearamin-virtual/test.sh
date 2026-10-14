#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$ROOT/_local/tests"
python3 "$ROOT/tests/test_bearamin_virtual_2040.py"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror "$ROOT/test/test_bearamin_virtual_wire.cpp" -o "$ROOT/_local/tests/wire"
"$ROOT/_local/tests/wire"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -I"$ROOT/third_party/bearfield-core/include" "$ROOT/test/test_bearamin_virtual_protocol.cpp" $(pkg-config --cflags --libs jansson) -o "$ROOT/_local/tests/protocol"
"$ROOT/_local/tests/protocol"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -I"$ROOT/third_party/bearfield-core/include" "$ROOT/test/test_bearamin_virtual_output.cpp" -o "$ROOT/_local/tests/output"
"$ROOT/_local/tests/output"
echo 'Bearamin Virtual source tests passed'

"${CC:-cc}" -std=c11 -Wall -Wextra -Werror "$ROOT/test/test_family_identity.c" -o "$ROOT/_local/tests/family-identity"
"$ROOT/_local/tests/family-identity"

python3 "$ROOT/tests/test_editor.py"
