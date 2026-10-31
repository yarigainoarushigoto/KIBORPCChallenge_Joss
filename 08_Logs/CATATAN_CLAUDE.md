# Catatan Claude — run otomatis malam 31 Okt 2026

## 3 run Gazebo (v1.1, veryhard, laptop i3-N305)

| Run | Checkpoint Level 3 | Goal | Durasi (sim) |
|---|---|---|---|
| 20261030175215 | 8 checkpoint (1,7,6,2,3,4,8,9) + Goal 12 | ✅ | 276 s |
| 20261030180444 | 8 checkpoint (9,8,4,3,2,6,7,1) + Goal 12 | ✅ | 252 s |
| 20261030181635 | 8 checkpoint (9,6,7,1,2,3,4,8) | ❌ | 241 s |

- Semua checkpoint yang dilaporkan = Level 3. Tidak ada yang gagal karena akurasi.
- Tiap run hanya ±12 menit jam dinding (lebih cepat dari run pertama).
- Run 3, Goal gagal: node navigasi simulator berhenti publish di t≈223 s tanpa error
  (posisi membeku di (4.33, 0.21, 1.30)), lalu ctl: "Navigation message timed out" →
  semua perintah ditolak (INVALID_NAV / INVALID_CMD). Robot sedang bergerak pelan & lurus,
  jadi kemungkinan besar gangguan simulator lokal (laptop berat), bukan dipicu program.
  Program tetap mengirim mission_finished. Perlu dipantau: kalau terjadi lagi di Web Simulator,
  baru dicari pola penyebabnya.

## Perubahan: v1.1 → v1.2
- Bug: setelah satu koreksi singkat (5 s), faktor kecepatan naik ke ×1.54, sehingga
  perencana membuang checkpoint yang sebenarnya masih muat (run 3: rencana turun dari 6 ke 2).
  Sekarang kecepatan hanya dipelajari dari leg tanpa koreksi.
- Uji tiruan: cheat sheet run 3 → 10 checkpoint Level 3 + Goal = 9 checkpoint + Goal, 275 s; seed acak 1–6
  sama persis dengan v1.1 (tidak ada yang memburuk).
- v1.1 diarsipkan: 04_Solusi/arsip/user_template_v1.1.py

## Saran berikutnya
- Jalankan auto_run.sh lagi (otomatis memakai v1.2): bash 05_Setup/auto_run.sh 3 veryhard
- Pengali waktu belum diketahui rumusnya → kalau POC/JAXA menjelaskan, sesuaikan strategi jumlah checkpoint.
