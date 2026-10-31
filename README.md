# Kibo-RPC ke-7 — Tim Joss

Program misi Int-Ball2 untuk [7th Kibo Robot Programming Challenge](https://jaxa.krpc.jp/) (Python 3.6 / ROS Melodic).
Guide lengkap tim: [`docs/Guide_Kibo-RPC_7_Tim_Kita.pdf`](docs/Guide_Kibo-RPC_7_Tim_Kita.pdf) · versi online yang bisa dikomentari: https://claude.ai/artifact/2nFMuDJhbWhRMqdvEbCZTz

## Status

| | |
|---|---|
| Versi program | `krpc7-route-v1.2` (`04_Solusi/user_template.py`) |
| Gazebo lokal v1.1, veryhard | 3 run: 8–9 checkpoint semua Level 3, Goal 2/3 (run ke-3: navigasi simulator membeku di t≈223 s) |
| Belum diketahui | Rumus pengali waktu skor → menentukan jumlah checkpoint ideal |

Riwayat run: [`08_Logs/RINGKASAN.txt`](08_Logs/RINGKASAN.txt) · catatan analisis: [`08_Logs/CATATAN_CLAUDE.md`](08_Logs/CATATAN_CLAUDE.md)

## Tanggal penting

| Kapan | Apa |
|---|---|
| **4 Nov 2026 14:59 WIB** | Batas pendaftaran tim ke POC BRIN |
| ±pertengahan Jan 2027 | Target freeze versi final (tag `final`) |
| **20 Jan 2027 21:59 WIB** (23:59 JST) | Batas submit preliminary |

## Setup pertama (sekali per laptop)

1. Windows: klik 2x `05_Setup/0_install_wsl_ubuntu.bat`, restart kalau diminta, buat user Ubuntu.
2. Clone repo (di Ubuntu lebih cepat daripada di `/mnt/c` atau `/mnt/e`):
   ```bash
   git clone https://github.com/yarigainoarushigoto/KIBORPCChallenge_Joss.git ~/kibo
   cd ~/kibo
   ```
3. Simulator **tidak** ada di git (ukurannya besar). Unduh `krpc7th-simulator.tar.gz` dari https://jaxa.krpc.jp/download ke `02_Simulator/` (atau jalankan `download_semua.bat` di Windows dari folder repo).
4. `bash 05_Setup/1_setup_ubuntu.sh` → tutup-buka terminal → `cd ~/krpc/krpc7th-simulator && bash ./build.sh` (30–90 menit).

Semua script mencari lokasi repo sendiri, jadi foldernya boleh ada di mana saja.

## Perintah harian (dari folder repo, di Ubuntu)

| Tujuan | Perintah |
|---|---|
| Tes cepat program di branch-mu (detik, tanpa Docker) | `bash 06_Tes_Cepat/tes_cepat.sh --batch 20` |
| Pasang program ke simulator | `bash 04_Solusi/pasang_solusi.sh` |
| Simulator asli 1x | `cd ~/krpc/krpc7th-simulator && execution_time=30m LEVELS=veryhard bash ./simulation.sh` |
| Simulator asli berulang + catat log | `bash 05_Setup/auto_run.sh 3 veryhard` |
| Bikin file submit | `bash 04_Solusi/bikin_submission.sh v1.3` |

## Cara kerja tim

### Alur Git
1. `git checkout main && git pull`
2. `git checkout -b nama/topik` (contoh: `budi/kalibrasi-vmax`)
3. Ubah **satu hal** saja → commit → `git push -u origin nama/topik`
4. Buka Pull Request ke `main`, isi template-nya (tabel batch sebelum/sesudah wajib ada).
5. Minimal 1 anggota lain review → merge.

`main` harus **selalu** bisa dipasang dan lolos tes cepat. Eksperimen besar tetap di branch sampai terbukti lebih baik.

### Setiap perubahan harus ada buktinya
Skor penyisihan adalah rata-rata 10 run acak, jadi satu run bagus belum membuktikan apa-apa.
- Bandingkan `main` dan branch-mu dengan perintah yang **sama persis**: `bash 06_Tes_Cepat/tes_cepat.sh --batch 20 --level veryhard --speed 0.9`.
  `--batch N` selalu memakai seed 1..N, jadi perbandingannya apel dengan apel.
- Tempel baris ringkasan `runs=… all-lv3=… goal=… mean_dur=…` dari kedua versi di PR.
- Kalau mengubah `user_template.py`, naikkan `PROGRAM_VERSION`.
- Setiap versi yang di-upload ke Web Simulator diberi tag: `git tag v1.3 && git push --tags`.

### Pembagian peran
| Peran | Fokus | Parameter / file utama |
|---|---|---|
| **A — Kalibrasi & data asli** | Gazebo lokal, cocokkan waktu leg nyata vs estimasi, selidiki navigasi membeku t≈223 s | `V_MAX`, `A_MAX`, `START_DELAY`, `STOP_OVERHEAD`, `PLAN_SAFETY` |
| **B — Strategi skor** | Gali Rulebook/FAQ/`judge.py`, tanya POC soal pengali waktu, atur jumlah checkpoint & rute | `FINISH_MARGIN`, perencana rute |
| **C — Eksperimen** | Waypoint tanpa berhenti (`/ctl/wrench`), ArUco cadangan — selalu di branch terpisah | branch `exp/*` |
| **Integrator** | Review PR, jaga `main` stabil, bikin tag & file submit | — |

Tugas dicatat di **Issues** (pakai template "Tugas" atau "Eksperimen").

### Kuota Web Simulator (milik tim, bukan per orang)
- Hanya versi yang sudah lolos tes cepat **dan** simulator lokal yang boleh di-upload.
- Setiap run dicatat di [`08_Logs/web_simulator.csv`](08_Logs/web_simulator.csv) (siapa, tag, level, skor, waktu, catatan). Web Simulator berjalan di ARM64 seperti robot asli, jadi datanya paling berharga untuk kalibrasi.

### Log
- `08_Logs/RINGKASAN.txt` dan `08_Logs/runs/*.result.json` masuk git (kecil, berguna untuk dibandingkan).
- Folder log mentah `08_Logs/2026…/` (rosbag, system.log ratusan MB) **tidak** masuk git. Kalau perlu dibagikan, pakai Google Drive lalu taruh link-nya di PR/Issue.

### Sync mingguan (15–30 menit)
Hasil batch minggu ini · PR yang menunggu review · sisa kuota Web Simulator · rencana minggu depan.

## Struktur folder

```
04_Solusi/        program misi (user_template.py) + script pasang & bikin submission
  arsip/          versi lama
05_Setup/         install WSL, setup Ubuntu, auto_run
06_Tes_Cepat/     simulasi tiruan cepat (pakai modul juri asli dari folder simulator)
07_Submission/    hasil bikin_submission.sh (tar.gz diabaikan git → simpan di GitHub Releases)
08_Logs/          ringkasan run, result.json, log Web Simulator
docs/             guide tim
01_Dokumen/ 02_Simulator/ 03_Sample_Code/   file resmi JAXA, diunduh lokal, tidak masuk git
```
