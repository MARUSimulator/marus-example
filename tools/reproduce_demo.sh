#!/usr/bin/env bash
# Reproduce the OceanScale ROV Sim demo end-to-end from this repository.
#
#   Requirements: Unity 6000.0.42f1 (with the HDRP + org.mujoco packages this project pins),
#   ffmpeg, and an X display for the recording step (the player captures its own framebuffer).
#
#   Usage:   UNITY=/path/to/Editor/Unity  DISPLAY_ID=:1  tools/reproduce_demo.sh
#   Outputs (all under the repo, git-ignored):
#     Builds/rov_sim_v1.x86_64            standalone Linux player
#     Builds/OceanScale_ROV_Sim_demo.mp4  the ~90 s four-task pitch/teaching video
#     Logs/{build,oracle,player}.log      run logs   ·   Logs/rov_session.csv  telemetry
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
UNITY="${UNITY:-/mnt/storage/marus-rov-sim/unity-6000.0.42f1/Editor/Unity}"
DISPLAY_ID="${DISPLAY_ID:-:1}"
SECS="${ROVSIM_RECORD_SECONDS:-90}"
BUILD="$REPO/Builds/rov_sim_v1.x86_64"
FRAMES="$REPO/Builds/recframes"
mkdir -p "$REPO/Logs" "$REPO/Builds"

echo "[1/4] Build standalone player (Unity)..."
"$UNITY" -batchmode -nographics -quit -projectPath "$REPO" \
  -executeMethod RovSimBuild.BuildStandalone -logFile "$REPO/Logs/build.log" < /dev/null
grep -q "result=Succeeded" "$REPO/Logs/build.log" || { echo "BUILD FAILED"; tail -25 "$REPO/Logs/build.log"; exit 1; }
echo "    OK: $BUILD"

echo "[2/4] Headless physics oracles (RovSimTest)..."
"$UNITY" -batchmode -nographics -quit -projectPath "$REPO" \
  -executeMethod RovSimTest.RunHeadless -logFile "$REPO/Logs/oracle.log" < /dev/null
grep -E "ROVTEST_RESULT" "$REPO/Logs/oracle.log" || { echo "ORACLE LOG MISSING"; exit 1; }

echo "[3/4] Record the ${SECS}s demo (DISPLAY=$DISPLAY_ID)..."
rm -rf "$FRAMES"; mkdir -p "$FRAMES"
for p in $(pgrep -f "Builds/rov_sim_v1.x86_64" 2>/dev/null || true); do kill "$p" 2>/dev/null || true; done
sleep 1
DISPLAY="$DISPLAY_ID" ROVSIM_RECORD="$FRAMES" ROVSIM_RECORD_SECONDS="$SECS" ROVSIM_CSV="$REPO/Logs/rov_session.csv" \
  "$BUILD" -screen-fullscreen 1 -screen-width 1920 -screen-height 1080 -logFile "$REPO/Logs/player.log" &
PID=$!
for i in $(seq 1 150); do kill -0 "$PID" 2>/dev/null || break; sleep 3; done
kill "$PID" 2>/dev/null || true
echo "    frames: $(ls "$FRAMES"/frame_*.png 2>/dev/null | wc -l)"

echo "[4/4] Encode video..."
ffmpeg -v error -y -framerate 30 -i "$FRAMES/frame_%05d.png" -c:v libx264 -preset veryfast \
  -pix_fmt yuv420p -crf 20 -movflags +faststart "$REPO/Builds/OceanScale_ROV_Sim_demo.mp4"
echo "DONE -> $REPO/Builds/OceanScale_ROV_Sim_demo.mp4"
grep -E "MISSION step" "$REPO/Logs/player.log" 2>/dev/null || true
