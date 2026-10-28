#!/bin/bash
# Run whole PSPLIB sets and summarise them:   ./run_full.sh <fwd|bwd|hbae> <set> [<set> ...]
# Each set runs in its own process (in parallel). CSVs go to results/, logs to logs/. Flags are passed through
# the environment, e.g.   RCPSP_TT2_TT=1 ./run_full.sh fwd j60 j90
# On the lab server start it detached:   nohup ./run_full.sh fwd j30 j60 j90 > run_fwd.txt 2>&1 &
# Memory: one 300 s process can reach about 50 GB on j60 and 63 GB on j90 (fwd without compact nodes; hbae
# 45 / 56 GB) - check that the machine has that much free for every set you start, plus room for the other users.
set -u
cd "$(dirname "$0")"
SEARCH=${1:-}; shift || true
case "$SEARCH" in fwd|bwd|hbae) ;; *) echo "usage: ./run_full.sh <fwd|bwd|hbae> <set> [<set> ...]"; exit 1 ;; esac
[ $# -ge 1 ] || { echo "usage: ./run_full.sh <fwd|bwd|hbae> <set> [<set> ...]"; exit 1; }
./build.sh || exit 1
mkdir -p logs
declare -A LOG
for set in "$@"; do
  LOG[$set]="logs/${SEARCH}_${set}_$(date +%Y%m%d-%H%M%S).txt"
  echo "start $SEARCH $set -> ${LOG[$set]}"
  ./rcpsp "$SEARCH" "$set" > "${LOG[$set]}" 2>&1 &
done
wait

echo
echo "== summary ($SEARCH) =="
for set in "$@"; do
  csv=$(grep -o '^CSV: .*' "${LOG[$set]}" | head -1 | cut -c6-)
  if grep -q 'WRONG RESULT' "${LOG[$set]}"; then echo "$set: STOPPED ON A WRONG RESULT, see ${LOG[$set]}"; continue; fi
  [ -n "$csv" ] && [ -f "$csv" ] || { echo "$set: did not finish, see ${LOG[$set]}"; continue; }
  awk -F, -v SET="$set" -v CSV="$csv" '
    NR == 1 { for (i = 1; i <= NF; i++) c[$i] = i; next }
    { n++; rs = ($(c["group"]) - 1) % 4
      if ($(c["solved"]) == "True") { s++; S[rs]++ }
      t += $(c["time"]) }
    END { printf "%s: %d instances, %d proven optimal (%.1f%%) | by RS 0.2/0.5/0.7/1.0: %d/%d/%d/%d of 120 | mean time %.1f s\n  %s\n",
                 SET, n, s, n ? 100 * s / n : 0, S[0], S[1], S[2], S[3], n ? t / n : 0, CSV }' "$csv"
done
