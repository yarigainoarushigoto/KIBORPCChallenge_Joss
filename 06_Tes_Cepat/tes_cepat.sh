#!/bin/bash
# Tes cepat TANPA Docker: program asli + modul juri ASLI + model gerak Int-Ball2 tiruan.
# Cocok untuk cek logika/rute/skor dalam hitungan detik. Hasil final tetap cek di simulator asli.
#   bash 06_Tes_Cepat/tes_cepat.sh                 # 1 run veryhard, log lengkap
#   bash 06_Tes_Cepat/tes_cepat.sh --batch 10      # 10 run, ringkasan
#   --batch N selalu memakai seed 1..N -> hasil sebelum/sesudah bisa dibandingkan langsung
#   opsi: --level easy|normal|hard|veryhard  --no-dist (tanpa gangguan)  --bias 0.05
export PYTHONDONTWRITEBYTECODE=1
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export KRPC_SIM="${KRPC_SIM:-$HOME/krpc/krpc7th-simulator}"
# default: tes program di branch git kamu saat ini (04_Solusi), bukan salinan di folder simulator
export KRPC_USER="${KRPC_USER:-$HERE/../04_Solusi/user_template.py}"
if [ ! -d "$KRPC_SIM" ]; then echo "!! simulator belum diekstrak ke $KRPC_SIM"; exit 1; fi
python3 -c "import numpy" 2>/dev/null || { echo "!! butuh numpy: sudo apt-get install -y python3-numpy"; exit 1; }
if [ "$#" -eq 0 ]; then set -- --level veryhard --seed 1 -v; fi
python3 "$HERE/run_mock.py" "$@"
