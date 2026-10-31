#!/bin/bash
# =====================================================================
#  Kibo-RPC 7 - setup otomatis di Ubuntu (WSL2)
#  Jalankan dari terminal Ubuntu:
#     bash 05_Setup/1_setup_ubuntu.sh      (dari folder repo)
#  Isi: install Docker (persis Programming Manual 2.4.2), git, python3-numpy,
#       ekstrak simulator ke ~/krpc, pasang program solusi kita.
# =====================================================================
set -e
# lokasi repo = folder induk script ini (bisa di drive mana saja / di home Ubuntu)
KIBO_WIN="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# simulator tidak ikut di git: taruh di 02_Simulator/ (download_semua.bat) atau set SIM_TAR=/path/ke/file
SIM_TAR="${SIM_TAR:-$KIBO_WIN/02_Simulator/krpc7th-simulator.tar.gz}"
SIM_DIR="$HOME/krpc/krpc7th-simulator"

echo "== 1/4 Install Docker Engine (Programming Manual 2.4.2) =="
if ! command -v docker >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y ca-certificates curl gnupg
  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  sudo chmod a+r /etc/apt/keyrings/docker.gpg
  echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  sudo apt-get update
  sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
  sudo usermod -aG docker "$USER"
  echo ">> Docker terpasang. Setelah script selesai: TUTUP terminal Ubuntu lalu buka lagi."
else
  echo ">> Docker sudah ada: $(docker --version)"
fi

echo "== 2/4 Tools tambahan (git, python3-numpy untuk tes cepat) =="
sudo apt-get install -y git python3 python3-numpy

echo "== 3/4 Ekstrak simulator ke ~/krpc =="
if [ ! -f "$SIM_TAR" ]; then
  echo "!! $SIM_TAR tidak ada. Jalankan download_semua.bat (Windows) atau unduh dari https://jaxa.krpc.jp/download ke 02_Simulator/."; exit 1
fi
mkdir -p "$HOME/krpc"
if [ -d "$SIM_DIR" ]; then
  echo ">> $SIM_DIR sudah ada, tidak ditimpa (hapus manual kalau mau ekstrak ulang)."
else
  tar -xzf "$SIM_TAR" -C "$HOME/krpc"
  echo ">> diekstrak ke $SIM_DIR"
fi

echo "== 4/4 Pasang program solusi =="
bash "$KIBO_WIN/04_Solusi/pasang_solusi.sh"

echo
echo "SELESAI. Langkah berikutnya:"
echo "  1) tutup & buka lagi terminal Ubuntu (supaya grup docker aktif), lalu: docker run hello-world"
echo "  2) cd ~/krpc/krpc7th-simulator && bash ./build.sh      (lama: bisa 30-90 menit, sekali saja)"
echo "  3) bash ./simulation.sh   lalu buka http://localhost di browser"
