#!/bin/bash
# Bikin file .tar.gz untuk diupload ke Web Simulator, lalu salin ke <repo>/07_Submission
#   bash 04_Solusi/bikin_submission.sh [nama]      (dari folder repo)
set -e
NAME="${1:-tim_$(date +%m%d_%H%M)}"
USERS="$HOME/krpc/krpc7th-simulator/users"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$REPO/07_Submission"
cd "$USERS"
# buang cache Python / file sampingan supaya submission bersih
find catkin_ws/src -name "__pycache__" -type d -prune -exec rm -rf {} +
find catkin_ws/src -name "*.pyc" -delete
bash ./packing.sh "$NAME"
mkdir -p "$OUT"
cp "$NAME.tar.gz" "$OUT/"
echo ">> $USERS/$NAME.tar.gz"
echo ">> disalin ke $OUT/$NAME.tar.gz"
echo "Upload: https://jaxa.krpc.jp/user-auth -> SIMULATION -> pilih file -> START SIMULATION"
