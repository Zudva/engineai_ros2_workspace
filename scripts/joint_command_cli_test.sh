#!/usr/bin/env bash
# Quick multi-mode CLI test tool for /hardware/joint_command
# SAFETY FEATURES ADDED:
#   - Global amplitude clip (env MAX_ABS_LIMIT, default 0.2)
#   - Per-step delta clip (env MAX_STEP_LIMIT, default 0.04)
#   - Optional smooth ramp in/out (RAMP_TIME seconds) for multi-joint waves
#   - Can disable safety with DISABLE_SAFETY=1 (not recommended on hardware)
# Patterns: zero, single, ramp, wave (single joint), all_wave, travel_wave,
#           multi_wave, cascade, random_pose, random_safe
# NOTE: This repeatedly invokes `ros2 topic pub --once`, which is fine for quick tests (<= ~20 Hz).
# For higher rates or long durations, implement a Python node instead.
#
# Usage examples:
#   ./scripts/joint_command_cli_test.sh zero 5            # 5 messages of all-zero pose
#   ./scripts/joint_command_cli_test.sh single 2 1.0 20 2 # joint 2 -> 1.0 rad, 20 Hz, 2 s hold
#   ./scripts/joint_command_cli_test.sh ramp 2 1.0 3 50   # ramp joint 2 from current(assumed 0) to 1.0 in 3 s @50 Hz
#   ./scripts/joint_command_cli_test.sh wave 2 0.8 1.0 5 50 # sine wave joint 2, amplitude 0.8, 1 Hz, 5 s @50 Hz
#   ./scripts/joint_command_cli_test.sh random_pose 0.5 4 10             # abrupt random pose each message (unsafe for hardware)
#   ./scripts/joint_command_cli_test.sh random_safe 0.5 4 10 0.05        # safe noise around current pose (amp 0.5, 4 s, 10 Hz, max step 0.05)
#   ./scripts/joint_command_cli_test.sh random_safe 0.5 4 10 0.05 0,1,2  # limit to joints 0,1,2
#   ./scripts/joint_command_cli_test.sh all_wave 0.4 0.5 6 60            # all joints same sine 0.4 rad, 0.5 Hz, 6 s @60 Hz
#   ./scripts/joint_command_cli_test.sh all_wave 0.4 0.5 6 60 0.1        # per-joint phase offset 0.1 rad (diagonal ripple)
#   ./scripts/joint_command_cli_test.sh travel_wave 0.3 0.7 6 50 1       # traveling wave 1 total 2π across robot
#   ./scripts/joint_command_cli_test.sh multi_wave 0.2 0.5 0.1 1.5 6 60 0.0 # superimpose two frequencies
#   ./scripts/joint_command_cli_test.sh cascade 0.5 1.0 6 40             # sequential joint activation
#
# Joint ordering assumed same as other examples (24 actuated joints).
# Adjust JOINT_COUNT if your robot differs.

set -euo pipefail

JOINT_COUNT=24
STIFFNESS_VAL=400
DAMPING_VAL=5
TOPIC="/hardware/joint_command"
MSG="interface_protocol/msg/JointCommand"

# Safety tunables (can override via environment)
MAX_ABS_LIMIT=${MAX_ABS_LIMIT:-0.2}
MAX_STEP_LIMIT=${MAX_STEP_LIMIT:-0.04}
RAMP_TIME=${RAMP_TIME:-0.5}   # seconds ramp in and ramp out for multi-joint waves
DISABLE_SAFETY=${DISABLE_SAFETY:-0}

# Internal previous command state for per-step limiting
declare -a LAST_POSITIONS
if (( ${#LAST_POSITIONS[@]} == 0 )); then
  for ((i=0;i<JOINT_COUNT;i++)); do LAST_POSITIONS+=(0); done
fi

safety_adjust_array() {
  # Args: list of positions -> prints adjusted CSV
  if [[ "$DISABLE_SAFETY" == "1" ]]; then
    local csv=$(IFS=,; echo "$*"); echo "$csv"; return 0
  fi
  local adjusted=()
  local i=0
  for val in "$@"; do
    local prev=${LAST_POSITIONS[$i]}
    # clamp per-step delta
    local delta=$(awk -v v="$val" -v p="$prev" 'BEGIN{print v-p}')
    local abs_delta=$(awk -v d="$delta" 'BEGIN{print (d<0?-d:d)}')
    if awk -v a="$abs_delta" -v lim="$MAX_STEP_LIMIT" 'BEGIN{exit (a>lim)?0:1}'; then
      # limit
      local sign=1
      if awk -v d="$delta" 'BEGIN{exit (d<0)?0:1}'; then sign=-1; fi
      val=$(awk -v p="$prev" -v lim="$MAX_STEP_LIMIT" -v s="$sign" 'BEGIN{print p + s*lim}')
    fi
    # global amplitude clamp
    if awk -v v="$val" -v m="$MAX_ABS_LIMIT" 'BEGIN{exit (v>m)?0:1}'; then val="$MAX_ABS_LIMIT"; fi
    if awk -v v="$val" -v m="$MAX_ABS_LIMIT" 'BEGIN{exit (v<-m)?0:1}'; then val=$(awk -v m="$MAX_ABS_LIMIT" 'BEGIN{print -m}'); fi
    adjusted+=("$val")
    LAST_POSITIONS[$i]="$val"
    ((i++))
  done
  local csv=$(IFS=,; echo "${adjusted[*]}")
  echo "$csv"
}

# Build repeated value list: value repeated JOINT_COUNT times, comma separated
repeat_values() {
  local val="$1"
  local out=""
  for ((i=0;i<JOINT_COUNT;i++)); do
    out+="$val"
    if (( i < JOINT_COUNT-1 )); then out+=","; fi
  done
  echo "$out"
}

STIFFNESS_LIST=$(repeat_values "$STIFFNESS_VAL")
DAMPING_LIST=$(repeat_values "$DAMPING_VAL")
ZERO_LIST=$(repeat_values 0)

publish_once() {
  local position_csv="$1"
  ros2 topic pub --once "$TOPIC" "$MSG" "{position: [${position_csv}], velocity: [${ZERO_LIST}], feed_forward_torque: [${ZERO_LIST}], torque: [${ZERO_LIST}], stiffness: [${STIFFNESS_LIST}], damping: [${DAMPING_LIST}], parallel_parser_type: 0}" >/dev/null
}

mode_zero() {
  local count=${1:-1}
  for ((k=0;k<count;k++)); do
    publish_once "$ZERO_LIST"
    sleep 0.05
  done
}

mode_single() {
  local joint_index=${1:?joint index}
  local angle=${2:?angle}
  local rate=${3:-10}
  local duration=${4:-2}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
  local steps=$(awk -v d="$duration" -v p="$period" 'BEGIN{print int(d/p)}')
  local arr=()
  for ((i=0;i<JOINT_COUNT;i++)); do arr+=(0); done
  arr[$joint_index]=$angle
  local csv=$(IFS=,; echo "${arr[*]}")
  for ((k=0;k<steps;k++)); do
    publish_once "$csv"
    sleep "$period"
  done
}

mode_ramp() {
  local joint_index=${1:?joint index}
  local target=${2:?target angle}
  local duration=${3:?seconds}
  local rate=${4:-50}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
  local steps=$(awk -v d="$duration" -v p="$period" 'BEGIN{print int(d/p)}')
  for ((k=0;k<=steps;k++)); do
    # linear interpolation alpha
    local alpha=$(awk -v k="$k" -v s="$steps" 'BEGIN{ if(s==0){print 1}else{print k/s}}')
    local val=$(awk -v a="$alpha" -v tgt="$target" 'BEGIN{print a*tgt}')
    local arr=()
    for ((i=0;i<JOINT_COUNT;i++)); do arr+=(0); done
    arr[$joint_index]=$val
    local csv=$(IFS=,; echo "${arr[*]}")
    publish_once "$csv"
    sleep "$period"
  done
}

mode_wave() {
  local joint_index=${1:?joint index}
  local amplitude=${2:?amplitude}
  local freq=${3:?Hz}
  local duration=${4:?seconds}
  local rate=${5:-50}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
  local steps=$(awk -v d="$duration" -v p="$period" 'BEGIN{print int(d/p)}')
  for ((k=0;k<=steps;k++)); do
    local t=$(awk -v k="$k" -v p="$period" 'BEGIN{print k*p}')
    # val = A * sin(2*pi*f*t)
    local val=$(python3 - <<EOF
import math
print(${amplitude}*math.sin(2*math.pi*${freq}*${t}))
EOF
)
    local arr=()
    for ((i=0;i<JOINT_COUNT;i++)); do arr+=(0); done
    arr[$joint_index]=$val
    local csv=$(IFS=,; echo "${arr[*]}")
    publish_once "$csv"
    sleep "$period"
  done
}

mode_random_pose() {
  # Full new random pose every tick (unsafe for real robot). Kept for simulation.
  local amplitude=${1:?amplitude}
  local duration=${2:?seconds}
  local rate=${3:-10}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
  local steps=$(awk -v d="$duration" -v p="$period" 'BEGIN{print int(d/p)}')
  for ((k=0;k<steps;k++)); do
    local arr=()
    for ((i=0;i<JOINT_COUNT;i++)); do
      val=$(python3 - <<EOF
import random
amp=${amplitude}
print((random.random()*2-1)*amp)
EOF
)
      arr+=("$val")
    done
    local csv=$(IFS=,; echo "${arr[*]}")
    publish_once "$csv"
    sleep "$period"
  done
}

mode_random_safe() {
  # Smooth bounded random walk around neutral 0 (or subset). Per-step delta limited.
  # Args: amplitude duration [rate=10] [max_step=0.05] [indices="all"]
  local amplitude=${1:?amplitude}
  local duration=${2:?seconds}
  local rate=${3:-10}
  local max_step=${4:-0.05}
  local indices_spec=${5:-all}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
  local steps=$(awk -v d="$duration" -v p="$period" 'BEGIN{print int(d/p)}')

  # Build active index set
  local -a active
  if [[ "$indices_spec" == "all" ]]; then
    for ((i=0;i<JOINT_COUNT;i++)); do active+=("$i"); done
  else
    IFS=',' read -r -a active <<< "$indices_spec"
  fi

  # State vector
  local -a current
  for ((i=0;i<JOINT_COUNT;i++)); do current+=(0); done

  for ((k=0;k<steps;k++)); do
    # Update active joints with bounded random step
    for idx in "${active[@]}"; do
      local delta=$(python3 - <<EOF
import random
print( (random.random()*2-1)*${max_step} )
EOF
)
      # new value clipped to [-amplitude, amplitude]
      local prev=${current[$idx]}
      local tentative=$(awk -v p="$prev" -v d="$delta" 'BEGIN{print p+d}')
      # clip
      local clipped=$(python3 - <<EOF
v=${tentative}
amp=${amplitude}
print(min(amp, max(-amp, v)))
EOF
)
      current[$idx]=$clipped
    done
    # Build CSV
    local csv=$(IFS=,; echo "${current[*]}")
    publish_once "$csv"
    sleep "$period"
  done
}

usage() {
  cat <<EOF
Usage: $0 <mode> [args...]
Modes:
  zero [count]
  single <joint_index> <angle> [rate=10] [duration=2]
  ramp <joint_index> <target_angle> <duration_s> [rate=50]
  wave <joint_index> <amplitude> <freq_hz> <duration_s> [rate=50]
  all_wave <amplitude> <freq_hz> <duration_s> [rate=50] [phase_step=0]
    phase_step: per-joint added phase (radians). 0 => all synchronous.
  travel_wave <amplitude> <freq_hz> <duration_s> [rate=50] [periods=1]
    periods: how many full 2π phase cycles spread across joints (>=1 float).
  multi_wave <amp1> <freq1> <amp2> <freq2> <duration_s> [rate=50] [phase_step=0]
  cascade <amplitude> <freq_hz> <duration_s> [rate=40] [hold=0.2]
    Activates joints one-by-one with sinusoid start times.
  random_pose <amplitude> <duration_s> [rate=10]              (abrupt; simulation only)
  random_safe <amplitude> <duration_s> [rate=10] [max_step=0.05] [indices=all]
    indices: comma-separated joint indices (e.g. 0,1,2) or 'all'.
EOF
}

main() {
  local mode=${1:-}
  shift || true
  case "$mode" in
    zero) mode_zero "$@" ;;
    single) mode_single "$@" ;;
    ramp) mode_ramp "$@" ;;
    wave) mode_wave "$@" ;;
  random) echo "Deprecated: use random_pose or random_safe" >&2; mode_random_pose "$@" ;;
  random_pose) mode_random_pose "$@" ;;
  random_safe) mode_random_safe "$@" ;;
  all_wave) mode_all_wave "$@" ;;
  travel_wave) mode_travel_wave "$@" ;;
  multi_wave) mode_multi_wave "$@" ;;
  cascade) mode_cascade "$@" ;;
    *) usage; exit 1 ;;
  esac
}

# --- Multi-joint animation implementations ---

mode_all_wave() {
  local amplitude=${1:?amplitude}
  local freq=${2:?freq_hz}
  local duration=${3:?seconds}
  local rate=${4:-50}
  local phase_step=${5:-0}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
  local steps=$(awk -v d="$duration" -v p="$period" 'BEGIN{print int(d/p)}')
  for ((k=0;k<=steps;k++)); do
    local t=$(awk -v k="$k" -v p="$period" 'BEGIN{print k*p}')
  # ramp factor in/out
  local ramp_in=$(awk -v tt="$t" -v rt="$RAMP_TIME" 'BEGIN{ if (tt<rt) print tt/rt; else print 1.0 }')
  local time_left=$(awk -v d="$duration" -v tt="$t" 'BEGIN{print d-tt}')
  local ramp_out=$(awk -v tl="$time_left" -v rt="$RAMP_TIME" 'BEGIN{ if (tl<rt) print tl/rt; else print 1.0 }')
  local ramp_factor=$(awk -v a="$ramp_in" -v b="$ramp_out" 'BEGIN{print (a<b)?a:b}')
    local arr=()
    for ((i=0;i<JOINT_COUNT;i++)); do
      val=$(python3 - <<EOF
import math
amp=${amplitude}
f=${freq}
base_t=${t}
phase_step=${phase_step}
i=${i}
rf=${ramp_factor}
print(rf*amp*math.sin(2*math.pi*f*base_t + phase_step*i))
EOF
)
      arr+=("$val")
    done
  local csv=$(safety_adjust_array "${arr[@]}")
    publish_once "$csv"
    sleep "$period"
  done
}

mode_travel_wave() {
  local amplitude=${1:?amplitude}
  local freq=${2:?freq_hz}
  local duration=${3:?seconds}
  local rate=${4:-50}
  local periods=${5:-1}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
  local steps=$(awk -v d="$duration" -v p="$period" 'BEGIN{print int(d/p)}')
  for ((k=0;k<=steps;k++)); do
    local t=$(awk -v k="$k" -v p="$period" 'BEGIN{print k*p}')
  local ramp_in=$(awk -v tt="$t" -v rt="$RAMP_TIME" 'BEGIN{ if (tt<rt) print tt/rt; else print 1.0 }')
  local time_left=$(awk -v d="$duration" -v tt="$t" 'BEGIN{print d-tt}')
  local ramp_out=$(awk -v tl="$time_left" -v rt="$RAMP_TIME" 'BEGIN{ if (tl<rt) print tl/rt; else print 1.0 }')
  local ramp_factor=$(awk -v a="$ramp_in" -v b="$ramp_out" 'BEGIN{print (a<b)?a:b}')
    local arr=()
    for ((i=0;i<JOINT_COUNT;i++)); do
      val=$(python3 - <<EOF
import math
amp=${amplitude}
f=${freq}
base_t=${t}
periods=${periods}
JOINT_COUNT=${JOINT_COUNT}
i=${i}
rf=${ramp_factor}
# phase distributed across joints: periods * 2*pi spans all joints
phase = (i/ (JOINT_COUNT-1) ) * periods * 2*math.pi if JOINT_COUNT>1 else 0
print(rf*amp*math.sin(2*math.pi*f*base_t + phase))
EOF
)
      arr+=("$val")
    done
  local csv=$(safety_adjust_array "${arr[@]}")
    publish_once "$csv"
    sleep "$period"
  done
}

mode_multi_wave() {
  local amp1=${1:?amp1}
  local freq1=${2:?freq1}
  local amp2=${3:?amp2}
  local freq2=${4:?freq2}
  local duration=${5:?seconds}
  local rate=${6:-50}
  local phase_step=${7:-0}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
  local steps=$(awk -v d="$duration" -v p="$period" 'BEGIN{print int(d/p)}')
  for ((k=0;k<=steps;k++)); do
    local t=$(awk -v k="$k" -v p="$period" 'BEGIN{print k*p}')
    local arr=()
    for ((i=0;i<JOINT_COUNT;i++)); do
      val=$(python3 - <<EOF
import math
amp1=${amp1}; f1=${freq1}; amp2=${amp2}; f2=${freq2}
t=${t}; phase_step=${phase_step}; i=${i}
phase = phase_step*i
val = amp1*math.sin(2*math.pi*f1*t + phase) + amp2*math.sin(2*math.pi*f2*t + 0.5*phase)
print(val)
EOF
)
      arr+=("$val")
    done
    local csv=$(IFS=,; echo "${arr[*]}")
          local ramp_in=$(awk -v tt="$t" -v rt="$RAMP_TIME" 'BEGIN{ if (tt<rt) print tt/rt; else print 1.0 }')
          local time_left=$(awk -v d="$duration" -v tt="$t" 'BEGIN{print d-tt}')
          local ramp_out=$(awk -v tl="$time_left" -v rt="$RAMP_TIME" 'BEGIN{ if (tl<rt) print tl/rt; else print 1.0 }')
          local ramp_factor=$(awk -v a="$ramp_in" -v b="$ramp_out" 'BEGIN{print (a<b)?a:b}')
    publish_once "$csv"
    sleep "$period"
  done
}

mode_cascade() {
  local amplitude=${1:?amplitude}
  local freq=${2:?freq_hz}
      rf=${ramp_factor}
      print(rf*val)
      EOF
      )
            arr+=("$val")
          done
          local csv=$(safety_adjust_array "${arr[@]}")
          publish_once "$csv"
          sleep "$period"
        done
      }

      mode_cascade() {
        local amplitude=${1:?amplitude}
        local freq=${2:?freq_hz}
        local duration=${3:?seconds}
        local rate=${4:-40}
        local hold=${5:-0.2}
        local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
        local steps=$(awk -v d="$duration" -v p="$period" 'BEGIN{print int(d/p)}')
        local offset_per_joint=$(awk -v h="$hold" 'BEGIN{print h}')
        for ((k=0;k<=steps;k++)); do
          local global_t=$(awk -v k="$k" -v p="$period" 'BEGIN{print k*p}')
          local arr=()
          for ((i=0;i<JOINT_COUNT;i++)); do
            local t_after=$(awk -v gt="$global_t" -v off="$offset_per_joint" -v idx="$i" 'BEGIN{val=gt-off*idx; if(val<0) val=0; print val}')
            val=$(python3 - <<EOF
      import math
      amp=${amplitude}; f=${freq}; t=${t_after}
      print(amp*math.sin(2*math.pi*f*t) if t>0 else 0.0)
  local duration=${3:?seconds}
  local rate=${4:-40}
  local hold=${5:-0.2}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
          local csv=$(safety_adjust_array "${arr[@]}")
  # time offset per joint so they activate sequentially
  local offset_per_joint=$(awk -v h="$hold" 'BEGIN{print h}')
  for ((k=0;k<=steps;k++)); do
    local global_t=$(awk -v k="$k" -v p="$period" 'BEGIN{print k*p}')
