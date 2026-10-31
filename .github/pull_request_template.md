## Apa yang diubah
<!-- Satu perubahan per PR. Parameter apa, dari berapa ke berapa, kenapa. -->

- PROGRAM_VERSION: `krpc7-route-v…`

## Bukti: tes cepat (perintah sama persis untuk dua versi)
`bash 06_Tes_Cepat/tes_cepat.sh --batch 20 --level veryhard --speed 0.9`

| | runs | all-lv3 | goal | in_time | mean_dur | max_dur |
|---|---|---|---|---|---|---|
| main (sebelum) | | | | | | |
| branch ini (sesudah) | | | | | | |

## Simulator asli (opsional, wajib untuk perubahan gerak/kalibrasi)
<!-- tempel baris dari 08_Logs/RINGKASAN.txt -->

## Checklist
- [ ] `bash 04_Solusi/pasang_solusi.sh` → "sintaks OK"
- [ ] Tidak ada seed yang memburuk (atau dijelaskan kenapa)
- [ ] PROGRAM_VERSION dinaikkan
