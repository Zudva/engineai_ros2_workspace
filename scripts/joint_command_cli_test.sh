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

# Arm indexing (override via env to match actual joint ordering)
# Defaults were previously hard-coded (left_start=13,right_start=18,arm_len=5) and may not match your robot.
LEFT_ARM_START=${LEFT_ARM_START:-13}
RIGHT_ARM_START=${RIGHT_ARM_START:-18}
ARM_LEN=${ARM_LEN:-5}
AUTO_DETECT_ARMS=${AUTO_DETECT_ARMS:-1}  # try to infer arm start indices & length from joint_state names

detect_arm_indices() {
  [[ "$AUTO_DETECT_ARMS" == "1" ]] || return 0
  # Only attempt if ros2 is up and topic responds quickly
  local line
  line=$(timeout 2s ros2 topic echo -n 1 /hardware/joint_state 2>/dev/null | grep '^name:' || true)
  if [[ -z "$line" ]]; then
    [[ "${VERBOSE:-0}" == "1" ]] && echo "[auto-detect] joint_state names not available (skip)" >&2
    return 0
  fi
  # Extract names list inside brackets
  line=${line#name:}
  line=${line#*[}
  line=${line%]*}
  IFS=',' read -r -a NAMES <<< "$line"
  local idx=0
  local -a left_candidates=()
  local -a right_candidates=()
  for raw in "${NAMES[@]}"; do
    # trim spaces & quotes
    local n=${raw//\"/}
    n=$(echo "$n" | sed -E 's/^ *//;s/ *$//')
    if echo "$n" | grep -qi 'left'; then
      if echo "$n" | grep -Eqi 'shoulder|elbow|wrist|gripper'; then
        left_candidates+=("$idx")
      fi
    fi
    if echo "$n" | grep -qi 'right'; then
      if echo "$n" | grep -Eqi 'shoulder|elbow|wrist|gripper'; then
        right_candidates+=("$idx")
      fi
    fi
    ((idx++))
  done
  # Derive contiguous block start & length helper
  contiguous_block() {
    local -a arr=("$@")
    local best_start=-1 best_len=0
    local current_start=-1 current_prev=-1000 current_len=0
    for v in "${arr[@]}"; do
      if (( current_start == -1 )); then
        current_start=$v; current_prev=$v; current_len=1
      elif (( v == current_prev + 1 )); then
        current_prev=$v; ((current_len++))
      else
        if (( current_len > best_len )); then best_len=$current_len; best_start=$current_start; fi
        current_start=$v; current_prev=$v; current_len=1
      fi
    done
    if (( current_len > best_len )); then best_len=$current_len; best_start=$current_start; fi
    echo "$best_start $best_len"
  }
  # Only override if user did not explicitly set (detect default sentinel values OR user allowed)
  if (( ${#left_candidates[@]} )); then
    read -r lstart llen < <(contiguous_block "${left_candidates[@]}")
    if [[ -n "$lstart" && $lstart != -1 ]]; then
      if [[ "${LEFT_ARM_START_OVERRIDE:-}" != "1" ]]; then
        LEFT_ARM_START=$lstart
        [[ $llen -gt 1 ]] && ARM_LEN=$llen
        [[ "${VERBOSE:-0}" == "1" ]] && echo "[auto-detect] LEFT_ARM_START=$LEFT_ARM_START ARM_LEN=$ARM_LEN" >&2
      fi
    fi
  fi
  if (( ${#right_candidates[@]} )); then
    read -r rstart rlen < <(contiguous_block "${right_candidates[@]}")
    if [[ -n "$rstart" && $rstart != -1 ]]; then
      if [[ "${RIGHT_ARM_START_OVERRIDE:-}" != "1" ]]; then
        RIGHT_ARM_START=$rstart
        # If both arms same length keep, else choose min shared length
        if (( rlen < ARM_LEN )); then ARM_LEN=$rlen; fi
        [[ "${VERBOSE:-0}" == "1" ]] && echo "[auto-detect] RIGHT_ARM_START=$RIGHT_ARM_START (ARM_LEN now $ARM_LEN)" >&2
      fi
    fi
  fi
}

# Perform detection early (non-fatal)
detect_arm_indices

# Safety tunables (can override via environment)
MAX_ABS_LIMIT=${MAX_ABS_LIMIT:-0.2}
MAX_STEP_LIMIT=${MAX_STEP_LIMIT:-0.04}
RAMP_TIME=${RAMP_TIME:-0.5}   # seconds ramp in and ramp out for multi-joint waves
DISABLE_SAFETY=${DISABLE_SAFETY:-0}

# Internal previous command state for per-step limiting (safe init under set -u)
if [[ -z "${LAST_POSITIONS_INIT_DONE:-}" ]]; then
  declare -ag LAST_POSITIONS=()
  for ((i=0;i<JOINT_COUNT;i++)); do LAST_POSITIONS+=(0); done
  LAST_POSITIONS_INIT_DONE=1
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
  if [[ "${VERBOSE:-0}" == "1" ]]; then
    echo "[PUB] positions=[${position_csv}]"
  fi
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
  arms_wave <amplitude> <freq_hz> <duration_s> [rate=50] [side=both] [per_joint_phase=0] [lr_phase=0]
    side: left|right|both
    per_joint_phase: incremental phase per joint within an arm
    lr_phase: extra phase added to right arm
  arms_raise <side> <target_angle> <duration_s> [rate=40] [joint_offset=2] [hold_s=0]
    Smooth S-curve lift of one arm joint (default joint_offset 2 within arm block).
  arms_scan <side> <target_angle> [duration_s=1.5] [rate=20] [hold_s=0.3]
    Sequentially raises each arm joint (offset 0..4) to find which lifts the arm.
  arms_info
    Print current arm detection: LEFT_ARM_START, RIGHT_ARM_START, ARM_LEN (no motion).
  joint_scan <angle> <hold_s> [rate=10] [start=0] [end=23]
    Pulses each joint index (start..end) to that angle for hold_s seconds.
  travel_wave <amplitude> <freq_hz> <duration_s> [rate=50] [periods=1]
    periods: how many full 2π phase cycles spread across joints (>=1 float).
  multi_wave <amp1> <freq1> <amp2> <freq2> <duration_s> [rate=50] [phase_step=0]
  cascade <amplitude> <freq_hz> <duration_s> [rate=40] [hold=0.2]
    Activates joints one-by-one with sinusoid start times.
  random_pose <amplitude> <duration_s> [rate=10]              (abrupt; simulation only)
  random_safe <amplitude> <duration_s> [rate=10] [max_step=0.05] [indices=all]
    indices: comma-separated joint indices (e.g. 0,1,2) or 'all'.
Env overrides:
  MAX_ABS_LIMIT      Global absolute limit (default 0.2)
  MAX_STEP_LIMIT     Per-step delta limit (default 0.04)
  RAMP_TIME          Ramp time for wave modes (default 0.5)
  ARM_MAX_ABS_LIMIT  If set, overrides MAX_ABS_LIMIT only inside arms_wave / arms_raise
  AUTO_DETECT_ARMS   (1 default) Try infer LEFT_ARM_START/RIGHT_ARM_START/ARM_LEN from joint_state names
  VERBOSE=1          Debug prints of generated commands
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
  arms_wave) mode_arms_wave "$@" ;;
  arms_raise) mode_arms_raise "$@" ;;
  arms_scan) mode_arms_scan "$@" ;;
  arms_info) mode_arms_info "$@" ;;
  joint_scan) mode_joint_scan "$@" ;;
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
    local ramp_in=$(awk -v tt="$t" -v rt="$RAMP_TIME" 'BEGIN{ if (tt<rt) print tt/rt; else print 1.0 }')
    local time_left=$(awk -v d="$duration" -v tt="$t" 'BEGIN{print d-tt}')
    local ramp_out=$(awk -v tl="$time_left" -v rt="$RAMP_TIME" 'BEGIN{ if (tl<rt) print tl/rt; else print 1.0 }')
    local ramp_factor=$(awk -v a="$ramp_in" -v b="$ramp_out" 'BEGIN{print (a<b)?a:b}')
    local arr=()
    for ((i=0;i<JOINT_COUNT;i++)); do
      val=$(python3 - <<EOF
import math
amp1=${amp1}; f1=${freq1}; amp2=${amp2}; f2=${freq2}
t=${t}; phase_step=${phase_step}; i=${i}
phase = phase_step*i
val = amp1*math.sin(2*math.pi*f1*t + phase) + amp2*math.sin(2*math.pi*f2*t + 0.5*phase)
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
EOF
)
      arr+=("$val")
    done
    local csv=$(safety_adjust_array "${arr[@]}")
    publish_once "$csv"
    sleep "$period"
  done
}

mode_arms_wave() {
  local amplitude=${1:?amplitude}
  local freq=${2:?freq_hz}
  local duration=${3:?seconds}
  local rate=${4:-50}
  local side=${5:-both}
  local per_joint_phase=${6:-0}
  local lr_phase=${7:-0}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
  local steps=$(awk -v d="$duration" -v p="$period" 'BEGIN{print int(d/p)}')
  local left_start=$LEFT_ARM_START
  local right_start=$RIGHT_ARM_START
  local arm_len=$ARM_LEN
  local saved_MAX_ABS_LIMIT="$MAX_ABS_LIMIT"
  if [[ -n "${ARM_MAX_ABS_LIMIT:-}" ]]; then MAX_ABS_LIMIT="$ARM_MAX_ABS_LIMIT"; fi
  for ((k=0;k<=steps;k++)); do
    local t=$(awk -v k="$k" -v p="$period" 'BEGIN{print k*p}')
    local ramp_in=$(awk -v tt="$t" -v rt="$RAMP_TIME" 'BEGIN{ if (tt<rt) print tt/rt; else print 1.0 }')
    local time_left=$(awk -v d="$duration" -v tt="$t" 'BEGIN{print d-tt}')
    local ramp_out=$(awk -v tl="$time_left" -v rt="$RAMP_TIME" 'BEGIN{ if (tl<rt) print tl/rt; else print 1.0 }')
    local ramp_factor=$(awk -v a="$ramp_in" -v b="$ramp_out" 'BEGIN{print (a<b)?a:b}')
    local arr=()
    for ((i=0;i<JOINT_COUNT;i++)); do arr+=(0); done
    if [[ "$side" == "left" || "$side" == "both" ]]; then
      for ((j=0;j<arm_len;j++)); do
        val=$(python3 - <<EOF
import math
amp=${amplitude}; f=${freq}; t=${t}; j=${j}; pjp=${per_joint_phase}; rf=${ramp_factor}
print(rf*amp*math.sin(2*math.pi*f*t + pjp*j))
EOF
)
        arr[$((left_start+j))]="$val"
      done
    fi
    if [[ "$side" == "right" || "$side" == "both" ]]; then
      for ((j=0;j<arm_len;j++)); do
        val=$(python3 - <<EOF
import math
amp=${amplitude}; f=${freq}; t=${t}; j=${j}; pjp=${per_joint_phase}; rf=${ramp_factor}; lrp=${lr_phase}
print(rf*amp*math.sin(2*math.pi*f*t + lrp + pjp*j))
EOF
)
        arr[$((right_start+j))]="$val"
      done
    fi
    local csv=$(safety_adjust_array "${arr[@]}")
    if [[ "${VERBOSE:-0}" == "1" ]]; then
      echo "[arms_wave] step=$k t=$t ramp_factor=$ramp_factor side=$side csv=$csv"
    fi
    publish_once "$csv"
    sleep "$period"
  done
  MAX_ABS_LIMIT="$saved_MAX_ABS_LIMIT"
}

mode_arms_raise() {
  # Args: side target_angle duration_s [rate=40] [joint_offset=2] [hold_s=0]
  local side=${1:?side}
  local target=${2:?target_angle}
  local duration=${3:?seconds}
  local rate=${4:-40}
  local joint_offset=${5:-2}
  local hold=${6:-0}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
  local steps=$(awk -v d="$duration" -v p="$period" 'BEGIN{print int(d/p)}')
  local left_start=$LEFT_ARM_START
  local right_start=$RIGHT_ARM_START
  local arm_len=$ARM_LEN
  local saved_MAX_ABS_LIMIT="$MAX_ABS_LIMIT"
  if [[ -n "${ARM_MAX_ABS_LIMIT:-}" ]]; then MAX_ABS_LIMIT="$ARM_MAX_ABS_LIMIT"; fi
  for ((k=0;k<=steps;k++)); do
    local t=$(awk -v k="$k" -v p="$period" 'BEGIN{print k*p}')
    local tau=$(awk -v tt="$t" -v dur="$duration" 'BEGIN{ if(dur==0){print 1}else if(tt>dur){print 1}else{print tt/dur}}')
    local s=$(python3 - <<EOF
u=${tau}
if u<0: u=0
if u>1: u=1
print(10*u**3 - 15*u**4 + 6*u**5)
EOF
)
    local angle=$(awk -v s="$s" -v tgt="$target" 'BEGIN{print s*tgt}')
    local arr=()
    for ((i=0;i<JOINT_COUNT;i++)); do arr+=(0); done
    if [[ "$side" == "left" ]]; then
      local idx=$((left_start + joint_offset))
      if (( idx < left_start+arm_len )); then arr[$idx]="$angle"; fi
    elif [[ "$side" == "right" ]]; then
      local idx=$((right_start + joint_offset))
      if (( idx < right_start+arm_len )); then arr[$idx]="$angle"; fi
    else
      local lidx=$((left_start + joint_offset))
      local ridx=$((right_start + joint_offset))
      if (( lidx < left_start+arm_len )); then arr[$lidx]="$angle"; fi
      if (( ridx < right_start+arm_len )); then arr[$ridx]="$angle"; fi
    fi
    local csv=$(safety_adjust_array "${arr[@]}")
    if [[ "${VERBOSE:-0}" == "1" ]]; then
      echo "[arms_raise] step=$k t=$t tau=$tau angle=$angle csv=$csv"
    fi
    publish_once "$csv"
    sleep "$period"
  done
  if awk -v h="$hold" 'BEGIN{exit (h>0)?0:1}'; then
    local hold_steps=$(awk -v h="$hold" -v p="$period" 'BEGIN{print int(h/p)}')
    local arr=()
    for ((i=0;i<JOINT_COUNT;i++)); do arr+=(0); done
    if [[ "$side" == "left" ]]; then
      local idx=$((left_start + joint_offset))
      if (( idx < left_start+arm_len )); then arr[$idx]="$target"; fi
    elif [[ "$side" == "right" ]]; then
      local idx=$((right_start + joint_offset))
      if (( idx < right_start+arm_len )); then arr[$idx]="$target"; fi
    else
      local lidx=$((left_start + joint_offset))
      local ridx=$((right_start + joint_offset))
      if (( lidx < left_start+arm_len )); then arr[$lidx]="$target"; fi
      if (( ridx < right_start+arm_len )); then arr[$ridx]="$target"; fi
    fi
    for ((k=0;k<hold_steps;k++)); do
      local csv=$(safety_adjust_array "${arr[@]}")
      publish_once "$csv"
      sleep "$period"
    done
  fi
  MAX_ABS_LIMIT="$saved_MAX_ABS_LIMIT"
}

mode_arms_scan() {
  # Args: side target_angle [duration=1.5] [rate=20] [hold=0.3]
  local side=${1:?side}
  local target=${2:?target_angle}
  local duration=${3:-1.5}
  local rate=${4:-20}
  local hold=${5:-0.3}
  local left_start=$LEFT_ARM_START
  local right_start=$RIGHT_ARM_START
  local arm_len=$ARM_LEN
  for ((off=0; off<arm_len; off++)); do
    echo "[arms_scan] Testing offset $off" >&2
    ./scripts/joint_command_cli_test.sh arms_raise "$side" "$target" "$duration" "$rate" "$off" "$hold"
    sleep 0.2
  done
}

mode_arms_info() {
  echo "LEFT_ARM_START=$LEFT_ARM_START" >&2
  echo "RIGHT_ARM_START=$RIGHT_ARM_START" >&2
  echo "ARM_LEN=$ARM_LEN" >&2
  echo "AUTO_DETECT_ARMS=$AUTO_DETECT_ARMS" >&2
  # Optionally show snippet of joint_state names for confirmation
  if line=$(timeout 2s ros2 topic echo -n 1 /hardware/joint_state 2>/dev/null | grep '^name:' ); then
    echo "joint_state names: $line" >&2
  else
    echo "(joint_state names not available now)" >&2
  fi
}

mode_joint_scan() {
  local angle=${1:?angle}
  local hold=${2:?hold_s}
  local rate=${3:-10}
  local start_idx=${4:-0}
  local end_idx=${5:-$((JOINT_COUNT-1))}
  local period=$(awk -v r="$rate" 'BEGIN{print 1.0/r}')
  echo "[joint_scan] scanning joints $start_idx..$end_idx to angle $angle (hold $hold s)" >&2
  for ((j=start_idx; j<=end_idx; j++)); do
    echo "[joint_scan] joint $j" >&2
    local steps=$(awk -v h="$hold" -v p="$period" 'BEGIN{print int(h/p)}')
    for ((s=0;s<steps;s++)); do
      local arr=()
      for ((i=0;i<JOINT_COUNT;i++)); do arr+=(0); done
      arr[$j]="$angle"
      local csv=$(safety_adjust_array "${arr[@]}")
      publish_once "$csv"
      sleep "$period"
    done
    # return to zero once
    publish_once "$ZERO_LIST"
    sleep 0.2
  done
}

# Invoke dispatcher now that all functions are defined
main "$@"
