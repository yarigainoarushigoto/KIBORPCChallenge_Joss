#!/bin/bash
# =====================================================================
#  Kibo-RPC 7 - jalankan simulasi berulang secara otomatis.
#  Tiap putaran:
#    1. memasang program TERBARU dari <repo>/04_Solusi (jadi Claude bisa
#       memperbaiki program di antara run tanpa kamu perlu mengetik apa-apa)
#    2. menjalankan simulator (watchdog 30 menit, cocok untuk laptop lambat)
#    3. menyalin log ke <repo>/08_Logs/<waktu> (diabaikan git) + menambah ringkasan
#       ke <repo>/08_Logs/RINGKASAN.txt dan result.json ke 08_Logs/runs/ (masuk git)
#
#  Pakai (di Ubuntu):
#     bash 05_Setup/auto_run.sh            # 3 run, veryhard
#     bash 05_Setup/auto_run.sh 5 normal   # 5 run, normal
#  Berhenti: buat file <repo>/08_Logs/STOP (atau tutup jendela / wsl --shutdown)
# =====================================================================
N="${1:-3}"
LEVEL="${2:-veryhard}"
KIBO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="$HOME/krpc/krpc7th-simulator"
LOGS="$KIBO/08_Logs"
mkdir -p "$LOGS"
rm -f "$LOGS/STOP"

for i in $(seq 1 "$N"); do
  if [ -f "$LOGS/STOP" ]; then echo ">> STOP ditemukan, berhenti."; break; fi
  echo "=================== RUN $i / $N  (level $LEVEL) $(date '+%H:%M') ==================="
  bash "$KIBO/04_Solusi/pasang_solusi.sh" || exit 1
  VER=$(grep -m1 "PROGRAM_VERSION =" "$SIM/users/catkin_ws/src/krpc_user/scripts/user_template.py" | cut -d"'" -f2)
  cd "$SIM" || exit 1
  execution_time=30m LEVELS="$LEVEL" bash ./simulation.sh > /tmp/krpc_sim_console.log 2>&1
  D=$(ls -td "$SIM"/share/logs/2* | head -1)
  NAME=$(basename "$D")
  # salin semua kecuali rosbag (besar); rosbag tetap ada di folder simulator
  mkdir -p "$LOGS/$NAME"
  (cd "$D" && find . -type f ! -name 'simulation.bag' -exec cp --parents {} "$LOGS/$NAME/" \;)
  cp /tmp/krpc_sim_console.log "$LOGS/$NAME/sim_console.log" 2>/dev/null
  mkdir -p "$LOGS/runs"; cp "$D/judge_module_logs/result.json" "$LOGS/runs/$NAME.result.json" 2>/dev/null
  SUM=$(grep -h "SUMMARY" "$D/user_programs_logs/user_program_output.log" 2>/dev/null | tail -1 | sed 's/.*SUMMARY/SUMMARY/')
  RES=$(python3 - "$D/judge_module_logs/result.json" <<'EOF'
import json, sys
try:
    r = json.load(open(sys.argv[1]))["JudgeReport"]
    det = r.get("checkpoint_detail", [])
    lv = [d["level"] for d in det if d.get("success")]
    print("passed=%d lv3=%d goal=%s dur=%s" % (len(lv), lv.count(3), r.get("goal", {}).get("success"),
                                              r.get("mission", {}).get("duration_sec")))
except Exception as e:
    print("result.json tidak terbaca: %s" % e)
EOF
)
  WHO="$(git -C "$KIBO" config user.name 2>/dev/null || whoami)"
  LINE="$NAME  $VER  $LEVEL  $RES  | by=$WHO | $SUM"
  echo "$LINE" | tee -a "$LOGS/RINGKASAN.txt"
done
echo ">> selesai. Ringkasan: $LOGS/RINGKASAN.txt"
