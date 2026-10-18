#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
if [ "$(uname -s)" = Darwin ] && [ -z "${SDKROOT:-}" ]; then
    SDKROOT=$(xcrun --sdk macosx --show-sdk-path)
    export SDKROOT
fi
test_dir=${TEST_BUILD_DIR:-${TMPDIR:-/tmp}}
mkdir -p "$test_dir"
test_binary=$(mktemp "$test_dir/drv8833-test.XXXXXX")
trap 'rm -f "$test_binary"' EXIT
${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs -Iinclude -Isrc \
    src/esp_drv8833.c tests/test_control.c -lm -o "$test_binary"
"$test_binary"
