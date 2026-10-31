#!/bin/bash
# Salin program solusi ke workspace simulator + backup template asli.
#   bash 04_Solusi/pasang_solusi.sh      (dari folder repo)
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST="$HOME/krpc/krpc7th-simulator/users/catkin_ws/src/krpc_user/scripts"
if [ ! -d "$DST" ]; then
  echo "!! $DST tidak ada. Ekstrak simulator dulu (05_Setup/1_setup_ubuntu.sh)."; exit 1
fi
BACKUP="$HOME/krpc/user_template.original.py"   # di luar folder src supaya tidak ikut ke file submit
if [ ! -f "$BACKUP" ]; then
  cp "$DST/user_template.py" "$BACKUP"
  echo ">> template asli dibackup: $BACKUP"
fi
# tr -d '\r' : buang line ending Windows kalau file sempat diedit di Notepad
tr -d '\r' < "$HERE/user_template.py" > "$DST/user_template.py"
chmod +x "$DST/user_template.py"
python3 -c "import ast,sys; ast.parse(open(sys.argv[1]).read())" "$DST/user_template.py" && echo ">> sintaks OK"
echo ">> terpasang: $DST/user_template.py"
