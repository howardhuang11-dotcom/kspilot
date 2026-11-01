#!/bin/zsh
# Unattended flight series (watchdog per flight, battery check between flights on macOS laptops).
# usage: tools/series.sh CRAFT N_TERM [full|orbit|entry]
#   full   : first a complete mission from the pad (own ascent, no recording) that saves g5_term for CRAFT
#   orbit  : the same from the g5_orbit checkpoint (saves 6 min of ascent; valid when the ascent did not change)
#   entry  : the same from g5_entry (valid only when nothing upstream of the entry interface changed)
#   N_TERM : then N flights from the 4 km checkpoint
# Each result line is appended to logs/series_<craft>.txt
R="$(cd "$(dirname "$0")/.." && pwd)"
cd "$R"
CRAFT=$1; N=${2:-3}; MODE=$3
OUT=logs/series_${CRAFT}.txt
LOCK=logs/series.lock          # one flight series at a time: two processes driving the game corrupt both runs
if [ -f $LOCK ] && kill -0 "$(cat $LOCK)" 2>/dev/null; then
  echo "another series is running (pid $(cat $LOCK)): not starting"; exit 1
fi
if pgrep -f "tools/orbit_recover.py" >/dev/null; then echo "a flight is running: not starting"; exit 1; fi
echo $$ > $LOCK
trap 'rm -f $LOCK' EXIT
batt_ok() {      # macOS only: without pmset (other platforms, desktops) the check always passes
  command -v pmset >/dev/null 2>&1 || return 0
  pmset -g batt | grep -q "AC Power" && return 0
  local pct=$(pmset -g batt | grep -o '[0-9]*%' | tr -d '%')
  [ "${pct:-0}" -ge 20 ]
}
run() {   # $@: extra args for orbit_recover
  batt_ok || { echo "$(date +%T) STOP: battery low" | tee -a $OUT; exit 1; }
  ${KSPILOT_PYTHON:-.venv/bin/python} tools/watchdog.py --stall 1500 --tries 1 -- tools/orbit_recover.py $CRAFT 100 100 6 "$@" > /dev/null 2>&1
  local f=$(ls -t logs/wd_*.out | head -1)
  echo "$(date +%T) $CRAFT $* : $(grep -hE 'RESULT|smoothness' $f | tr '\n' ' ') [$f]" | tee -a $OUT
}
if [ -n "$MODE" ]; then
  case $MODE in
    full)  KSP_NOLEARN=1 KSP_SAVE_TERM=1 run ;;
    orbit) KSP_NOLEARN=1 KSP_SAVE_TERM=1 run --from-orbit ;;
    entry) KSP_NOLEARN=1 KSP_SAVE_TERM=1 run --from-entry ;;
    *) echo "unknown mode $MODE"; exit 1 ;;
  esac
  # the 4 km checkpoint is only valid if this mission flew it: stop on REFUSED / FAIL
  tail -1 $OUT | grep -q "RESULT PASS" || { echo "$(date +%T) full mission did not pass: series stopped" | tee -a $OUT; exit 1; }
fi
for ((i = 1; i <= N; i++)); do     # (macOS `seq 1 0` counts DOWN: N=0 ran two extra flights)
  KSP_NOLEARN=1 run --from-term
done
echo "$(date +%T) SERIES DONE" | tee -a $OUT
