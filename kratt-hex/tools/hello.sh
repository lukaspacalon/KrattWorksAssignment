#!/usr/bin/env bash
#
# Hello world: builds, then starts the GCS and the Drone and lets them talk to
# each other over real UDP on localhost for a few seconds.
#
#   ./tools/hello.sh
#
# Success looks like: "GCS listening", "drone listening", then "drone connected".

set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT" || exit 1

echo "--- building (GUI off) ---"
cmake -S . -B build -DKRATT_BUILD_GUI=OFF > /dev/null 2>&1
if ! cmake --build build -j2 > /tmp/kratt-build.log 2>&1; then
    echo "BUILD FAILED. Last 30 lines:"
    tail -30 /tmp/kratt-build.log
    exit 1
fi
echo "build OK"
echo

echo "--- starting GCS on port 14550 ---"
./build/bin/GCS --bind-port 14550 > /tmp/kratt-gcs.log 2>&1 &
GCS_PID=$!
sleep 1

echo "--- starting Drone on port 14551, streaming to 127.0.0.1:14550 ---"
./build/bin/Drone --bind-port 14551 --gcs 127.0.0.1:14550 > /tmp/kratt-drone.log 2>&1 &
DRONE_PID=$!

echo "--- letting them talk for 4 seconds ---"
sleep 4

kill "$DRONE_PID" "$GCS_PID" 2>/dev/null
wait "$DRONE_PID" "$GCS_PID" 2>/dev/null

echo
echo "========== DRONE OUTPUT =========="
cat /tmp/kratt-drone.log
echo "========== GCS OUTPUT ============"
cat /tmp/kratt-gcs.log
echo "=================================="
echo
if grep -q "drone connected" /tmp/kratt-gcs.log; then
    echo "SUCCESS: the GCS saw the drone's heartbeat."
else
    echo "PROBLEM: no 'drone connected' line. Copy everything above."
fi
