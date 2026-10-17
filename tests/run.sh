#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
test_binary=$(mktemp /tmp/drv8833-test.XXXXXX)
trap 'rm -f "$test_binary"' EXIT
${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs -Iinclude -Isrc \
    src/esp_drv8833.c tests/test_control.c -lm -o "$test_binary"
"$test_binary"
