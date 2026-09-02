#!/usr/bin/env bash
#
# Produces kratt-diagnostic.log with everything needed to debug a failed build
# or a failed test, in one command. Never stops on the first error: the point is
# to collect the whole picture, not to bail out early.
#
#   ./tools/diag.sh              full diagnostic
#   ./tools/diag.sh geofence     same, but only the geofence tests
#
# Then paste the contents of kratt-diagnostic.log.

FILTER="${1:-}"
# Google Test filters are case-sensitive, but nobody types "Geofence" with a
# capital G on the command line. Capitalise the first letter so the obvious
# invocation works.
if [ -n "$FILTER" ]; then
    FILTER="$(echo "${FILTER:0:1}" | tr '[:lower:]' '[:upper:]')${FILTER:1}"
fi
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG="$ROOT/kratt-diagnostic.log"
cd "$ROOT" || exit 1

: > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }
run() { echo "\$ $*" >> "$LOG"; "$@" >> "$LOG" 2>&1; echo "  -> exit $?" >> "$LOG"; echo >> "$LOG"; }

say "=================================================================="
say " KRATT DIAGNOSTIC  -  $(date)"
say "=================================================================="

say ""
say "--- STEP 1/4 : toolchain -----------------------------------------"
run uname -a
run bash -c 'g++ --version | head -1'
run bash -c 'cmake --version | head -1'
run bash -c 'git --version'

say ""
say "--- STEP 2/4 : dependencies --------------------------------------"
for dep in mavlink/common/mavlink.h googletest/CMakeLists.txt; do
    if [ -e "libs/$dep" ]; then
        say "  OK      libs/$dep"
    else
        say "  MISSING libs/$dep   <-- run: git submodule update --init --recursive"
    fi
done
echo "--- libs/ contents ---" >> "$LOG"
ls -la libs >> "$LOG" 2>&1

say ""
say "--- STEP 3/4 : configure + build ---------------------------------"
run rm -rf build
run cmake -S . -B build -DKRATT_BUILD_GUI=OFF -DCMAKE_BUILD_TYPE=Debug
run cmake --build build -j2

if [ ! -x build/bin/DomainTests ]; then
    say "  BUILD FAILED - no test binary produced. See the log for the compiler errors."
    say ""
    say "Log written to: $LOG"
    exit 1
fi
say "  build OK"

say ""
say "--- STEP 4/4 : tests ---------------------------------------------"
if [ -n "$FILTER" ]; then
    say "  filter: *${FILTER}*"
    run ./build/bin/DomainTests --gtest_filter="*${FILTER}*" --gtest_color=no
    run ./build/bin/IntegrationTests --gtest_filter="*${FILTER}*" --gtest_color=no
    # A filter that matches nothing looks like a pass; say so explicitly.
    MATCHED=$(grep -c "^\[ RUN" "$LOG")
    if [ "$MATCHED" = "0" ]; then
        say "  WARNING: the filter matched 0 tests. Available suites:"
        ./build/bin/DomainTests --gtest_list_tests 2>/dev/null | grep -v "^ " | tee -a "$LOG"
        ./build/bin/IntegrationTests --gtest_list_tests 2>/dev/null | grep -v "^ " | tee -a "$LOG"
    else
        say "  ${MATCHED} test(s) ran"
    fi
else
    run ./build/bin/DomainTests --gtest_color=no
    run ./build/bin/IntegrationTests --gtest_color=no
    run bash -c 'cd build && ctest --output-on-failure'
fi

say ""
say "=================================================================="
say " Log written to: $LOG"
say " Paste that file back in the conversation."
say "=================================================================="
