#!/bin/zsh
# Robustness matrix: one full pad mission (own ascent) per case, sequential, one variable changed per case from the
# baseline T-WorksFineGF 100/100/6. Andon: the first case without a "RESULT PASS" (FAIL, REFUSED, missing) stops it.
# usage: tools/matrix_run.sh [case ...]          (no args = all cases in order; or MATRIX_CASES="inc0 inc28")
#   detached: nohup tools/matrix_run.sh inc0 inc28 >/dev/null 2>&1 &     (progress: logs/matrix_run.log)
#   results : one line per case in logs/matrix_results.txt
#   dry run : MATRIX_DRYRUN=1 echoes the commands and fakes a new wd file per case in a temp dir (no game, no python);
#             MATRIX_DRYRUN_FAIL=<case> fakes a FAIL there, MATRIX_DRYRUN_NOFILE=<case> fakes a run with no new wd file
R="$(cd "$(dirname "$0")/.." && pwd)"
cd "$R"

# ---- case table: name -> "CRAFT PE AP INC [ENV=VAL ...]" (order = CASE_ORDER) ----
typeset -A CASES
CASES=(
  110km "T-WorksFineGF 110 110 6"
  inc0  "T-WorksFineGF 100 100 0"
  inc28 "T-WorksFineGF 100 100 28 KSP_MAX_DVN=60"
  80km  "T-WorksFineGF 80 80 6 KSP_MAX_DVN=60"
  inc45 "T-WorksFineGF 100 100 45 KSP_MAX_DVN=100"
  p45   "T-WorksFineGF-P45 100 100 6"
  inc90 "T-WorksFineGF 100 100 90 KSP_MAX_DVN=130"
  p60   "T-WorksFineGF-P60 100 100 6"
)
CASE_ORDER=(110km inc0 inc28 80km inc45 p45 inc90 p60)

DRY=${MATRIX_DRYRUN:-}
if [[ -n $DRY ]]; then
  LOGDIR=${MATRIX_LOGDIR:-$(mktemp -d -t matrix_dry)}   # everything (wd files, lock, logs) in a temp dir
else
  LOGDIR=logs
fi
mkdir -p $LOGDIR
RUNLOG=$LOGDIR/matrix_run.log
RESULTS=$LOGDIR/matrix_results.txt
LOCK=$LOGDIR/series.lock           # same lock as tools/series.sh: one flight series at a time

say() { echo "$(date '+%F %T') $*" | tee -a $RUNLOG; }
stop() { say "MATRIX STOP: $*"; exit 1; }

# ---- which cases ----
if (( $# )); then sel=("$@"); elif [[ -n $MATRIX_CASES ]]; then sel=(${=MATRIX_CASES}); else sel=($CASE_ORDER); fi
for c in $sel; do [[ -n ${CASES[$c]} ]] || { echo "unknown case '$c' (known: $CASE_ORDER)"; exit 2; }; done

# ---- pre-flight checks ----
power_ok() {        # on AC, or battery >= 20 % (macOS pmset; always passes where pmset is absent)
  command -v pmset >/dev/null 2>&1 || return 0
  local b=$(pmset -g batt)
  echo $b | grep -q "AC Power" && return 0
  local pct=$(echo $b | grep -o '[0-9]*%' | head -1 | tr -d '%')
  [ "${pct:-0}" -ge 20 ]
}
flight_pids() {     # running orbit_recover.py processes (watchdog children included), never ourselves:
  # the [.] makes the pattern text differ from the string it matches, so a process whose own command line carries
  # this pattern (this shell, a `zsh -c` wrapper, pgrep) cannot match itself
  pgrep -f 'tools/orbit_recover[.]py' | grep -vx "$$"
}
lock_holder() {     # pid of a live series lock holder other than us
  [[ -f $LOCK ]] || return 1
  local p=$(cat $LOCK 2>/dev/null)
  [[ -n $p && $p != $$ ]] && kill -0 "$p" 2>/dev/null && { echo $p; return 0; }
  return 1
}
wait_clear() {      # wait (max MATRIX_WAIT_MAX s, default 4 h) for other series / flights to finish
  local t0=$SECONDS max=${MATRIX_WAIT_MAX:-14400} p f
  while :; do
    p=$(lock_holder); f=$(flight_pids | tr '\n' ' ')
    [[ -z $p && -z ${f// } ]] && return 0
    if [[ -n $DRY && -z $p ]]; then
      say "dry run: a flight is running (pids $f) - a real run would wait here; continuing"; return 0
    fi
    (( SECONDS - t0 > max )) && stop "still busy after ${max}s (lock pid '$p', flight pids '$f')"
    say "waiting: series lock pid '$p', flight pids '$f'"
    sleep 60
  done
}
newest_wd() { local f=($LOGDIR/wd_*.out(N.om)); echo ${f[1]}; }   # newest by mtime, empty if none

trap '[[ "$(cat $LOCK 2>/dev/null)" == "$$" ]] && rm -f $LOCK' EXIT
say "MATRIX START cases: $sel ${DRY:+(DRY RUN in $LOGDIR)}"

for c in $sel; do
  spec=(${=CASES[$c]})
  craft=$spec[1]; pe=$spec[2]; ap=$spec[3]; inc=$spec[4]; extra=(${spec[5,-1]})
  power_ok || stop "battery low and not on AC before case $c ($(pmset -g batt | grep -o '[0-9]*%'))"
  wait_clear
  echo $$ > $LOCK
  before=$(newest_wd)
  cmd=(env KSP_NOLEARN=1 KSP_SAVE_TERM=1 "KSP_REVIEW=matrix $c" $extra ${KSPILOT_PYTHON:-.venv/bin/python} tools/watchdog.py
       --stall 2400 --tries 1 -- tools/orbit_recover.py $craft $pe $ap $inc)
  say "case $c: ${cmd[*]}"
  if [[ -n $DRY ]]; then
    sleep 1                                          # wd file names have 1 s resolution
    if [[ $MATRIX_DRYRUN_NOFILE != $c ]]; then
      res=PASS; [[ $MATRIX_DRYRUN_FAIL == $c ]] && res=FAIL
      fake=$LOGDIR/wd_$(date +%Y%m%d_%H%M%S).out
      { echo "[  695.0s] orbit Ap=$ap.2km Pe=$pe.0km inc=$inc.027 (err Ap +169m Pe +9m)"
        echo "[ 1648.7s] RESULT $res miss=1.5m v_td=0.12m/s upright=1.000 fuel_left=372 parts=26 VesselSituation.landed"
      } > $fake
    fi
  else
    $cmd >> $RUNLOG 2>&1
  fi
  after=$(newest_wd)
  # only the wd file THIS run created counts; an unchanged newest file is an old run's record
  if [[ -z $after || $after == $before ]]; then
    echo "$(date '+%F %T') $c RESULT MISSING (no new wd file; newest still ${before:-none})" >> $RESULTS
    stop "case $c produced no new watchdog log"
  fi
  rline=$(grep -h 'RESULT ' $after | tail -1)
  oline=$(grep -h 'orbit Ap=.*(err' $after | tail -1)
  result=$(echo $rline | sed -nE 's/.*RESULT ([A-Z]+).*/\1/p')
  miss=$(echo $rline | sed -nE 's/.*miss=([^ ]+).*/\1/p')
  fuel=$(echo $rline | sed -nE 's/.*fuel_left=([^ ]+).*/\1/p')
  orbit=$(echo $oline | sed -E 's/^\[[^]]*\] *//')
  echo "$(date '+%F %T') $c ${craft} ${pe}/${ap}/${inc} ${extra} RESULT ${result:-MISSING} miss=${miss:--} fuel=${fuel:--} orbit=[${orbit:--}] wd=$after" | tee -a $RESULTS >> $RUNLOG
  say "case $c: ${rline:-no RESULT line}"
  [[ $result == PASS ]] || stop "case $c result ${result:-MISSING} (andon)"
done
say "MATRIX DONE"
