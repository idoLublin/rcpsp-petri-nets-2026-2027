#!/bin/bash
# Run whole PSPLIB sets and summarise them:   ./run_full.sh <tt2|tt2rev|tt2hbae> <set> [<set> ...]
#   tt2 = forward search, tt2rev = backward search, tt2hbae = bidirectional search; set = j30 | j60 | j90
# Each set runs in its own process (in parallel). CSVs go to results/, logs to logs/. Flags are passed through
# the environment, e.g.   RCPSP_TT2_TT=1 ./run_full.sh tt2 j60 j90
# On the lab server start it detached:   nohup ./run_full.sh tt2 j30 j60 j90 > run_tt2.txt 2>&1 &
# Memory: one 300 s process peaks at about 50 GB on j60 and 63 GB on j90 (TT2; HBAE 45 / 56 GB) - check that the
# machine has that much free for every set you start, plus a margin for the other users.
set -u
cd "$(dirname "$0")"
MODE=${1:-}; shift || true
case "$MODE" in tt2|tt2rev|tt2hbae) ;; *) echo "usage: ./run_full.sh <tt2|tt2rev|tt2hbae> <set> [<set> ...]"; exit 1 ;; esac
[ $# -ge 1 ] || { echo "usage: ./run_full.sh <tt2|tt2rev|tt2hbae> <set> [<set> ...]"; exit 1; }
./build.sh || exit 1
mkdir -p logs
declare -A LOG
for set in "$@"; do
  LOG[$set]="logs/${MODE}_${set}_$(date +%Y%m%d-%H%M%S).txt"
  echo "start $MODE $set -> ${LOG[$set]}"
  ./rcpsp "${MODE}_${set}" > "${LOG[$set]}" 2>&1 &
done
wait

echo
echo "== summary ($MODE) =="
for set in "$@"; do
  csv=$(grep -o -- '-> .*\.csv' "${LOG[$set]}" | head -1 | cut -c4-)
  if grep -q 'WRONG RESULT' "${LOG[$set]}"; then echo "$set: STOPPED ON A WRONG RESULT, see ${LOG[$set]}"; continue; fi
  [ -n "$csv" ] && [ -f "$csv" ] || { echo "$set: no CSV, see ${LOG[$set]}"; continue; }
  awk -F, -v SET="$set" -v CSV="$csv" '
    NR == 1 { for (i = 1; i <= NF; i++) c[$i] = i; next }
    { n++; rs = ($(c["group"]) - 1) % 4
      if ($(c["solved"]) == "True") { s++; S[rs]++
        if ($(c["knownOptimum"]) > 0 && $(c["makespan"]) != $(c["knownOptimum"])) w++ }
      t += $(c["time"]) }
    END { printf "%s: %d rows, %d solved (%.1f%%), wrong %d | solved by RS 0.2/0.5/0.7/1.0 = %d/%d/%d/%d | mean time %.1f s\n  %s\n",
                 SET, n, s, n ? 100 * s / n : 0, w, S[0], S[1], S[2], S[3], n ? t / n : 0, CSV }' "$csv"
done
