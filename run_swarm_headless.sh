#!/usr/bin/env bash
# Run from PX4-Autopilot: /path/to/run_swarm_headless.sh [drones] [spacing_m] [model] [autostart]
# SWARM_MODE=gui is used only by run_swarm_mission.sh.
set -eo pipefail
SWARM_MODE="${SWARM_MODE:-headless}"
export SWARM_MODE
if [[ "$SWARM_MODE" == headless ]]; then
    export HEADLESS=1
elif [[ "$SWARM_MODE" == gui ]]; then
    unset HEADLESS
else
    echo "SWARM_MODE must be headless or gui" >&2; exit 2
fi

NUM_DRONES="${1:-15}"
SPACING="${2:-25}"
MODEL="${3:-gz_x500_depth}"
AUTOSTART="${4:-4002}"
CODE_DIR="${CODE_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
VENV_PYTHON="${VENV_PYTHON:-python3}"
LOG_DIR="${SWARM_LOG_DIR:-$CODE_DIR/swarm_logs}"
EXTRA_PORT_BASE=14640
BIN="./build/px4_sitl_default/bin/px4"
COORD_SCRIPT="$CODE_DIR/swarm_mission.py"

[[ "$NUM_DRONES" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid drone count" >&2; exit 2; }
awk -v x="$SPACING" 'BEGIN { exit !(x+0>=20 && x ~ /^[0-9]+([.][0-9]+)?$/) }' || {
    echo "Spacing must be at least 20 m for safe takeoff" >&2; exit 2;
}
[[ -x "$BIN" ]] || { echo "Build PX4 SITL first; run this from the PX4 root." >&2; exit 1; }
[[ -f "$COORD_SCRIPT" ]] || { echo "Missing $COORD_SCRIPT" >&2; exit 1; }
command -v "$VENV_PYTHON" >/dev/null || { echo "Python not found: $VENV_PYTHON" >&2; exit 1; }
if [[ "$SWARM_MODE" == gui && -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo "Gazebo GUI needs DISPLAY or WAYLAND_DISPLAY." >&2; exit 1
fi

# Direct Gazebo Transport needs no ROS 2 setup/bridge.
mkdir -p "$LOG_DIR"
MODEL_OVERRIDE_DIR=""
if [[ "$MODEL" == gz_x500_depth ]]; then
    MODEL_OVERRIDE_DIR="$LOG_DIR/camera_model_override"
    if "$VENV_PYTHON" "$CODE_DIR/prepare_swarm_camera.py" "$MODEL_OVERRIDE_DIR" "$PWD"; then
        export GZ_SIM_RESOURCE_PATH="$MODEL_OVERRIDE_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
    else
        echo "Camera warning: could not namespace depth topics; RGB may work, depth will be marked ambiguous." >&2
    fi
fi
if (( NUM_DRONES > 10 )); then
    # PX4 normally sends every instance >=10 to UDP 14549. rcS resolves
    # px4-rc.mavlink from PATH, so override just that script for this run.
    RC_SOURCE="$PWD/build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink"
    [[ -f "$RC_SOURCE" ]] || RC_SOURCE="$PWD/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink"
    [[ -f "$RC_SOURCE" ]] || { echo "Cannot find PX4's px4-rc.mavlink." >&2; exit 1; }
    OVERRIDE_DIR=$(mktemp -d "$LOG_DIR/px4-ports.XXXXXXXX")
    if ! python3 - "$RC_SOURCE" "$OVERRIDE_DIR/px4-rc.mavlink" "$EXTRA_PORT_BASE" <<'PY'
import pathlib
import re
import sys

source, destination, base = sys.argv[1:]
original = pathlib.Path(source).read_text()
pattern = r'(?m)^\s*\[\s*"\$px4_instance"\s*-gt\s*9\s*\]\s*&&\s*udp_offboard_port_remote=14549[^\n]*$'
replacement = f'[ "$px4_instance" -gt 9 ] && udp_offboard_port_remote=$(({base}+px4_instance))'
updated, matches = re.subn(pattern, lambda _: replacement, original)
if matches != 1:
    sys.exit("Unsupported PX4 MAVLink startup format; expected the 14549 cap exactly once.")
pathlib.Path(destination).write_text(updated)
PY
    then
        rm -rf "$OVERRIDE_DIR"
        exit 1
    fi
    export PATH="$OVERRIDE_DIR:$PATH"
    echo "Extended offboard UDP ports: drone 10 -> $((EXTRA_PORT_BASE+10)), through drone $((NUM_DRONES-1)) -> $((EXTRA_PORT_BASE+NUM_DRONES-1))"
fi
COLS=$(awk -v n="$NUM_DRONES" 'BEGIN {c=int(sqrt(n)); if(c*c<n)c++; print c}')
ROWS=$(( (NUM_DRONES + COLS - 1) / COLS ))
echo "UAV-X | Gazebo $SWARM_MODE + PX4 | Python telemetry console | $NUM_DRONES drones | ${COLS}x${ROWS} grid | ${SPACING}m spacing"
echo "Model: $MODEL | Autostart: $AUTOSTART | Logs: $LOG_DIR"
PIDS=()
COORD_PID=""
cleanup() {
    trap - EXIT INT TERM
    [[ -z "$COORD_PID" ]] || kill "$COORD_PID" 2>/dev/null || true
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
    [[ -z "${OVERRIDE_DIR:-}" ]] || rm -rf "$OVERRIDE_DIR"
}
on_signal() {
    trap - INT TERM
    echo "Interrupt received; requesting vehicle landing..."
    [[ -z "$COORD_PID" ]] || {
        kill -INT "$COORD_PID" 2>/dev/null || true
        wait "$COORD_PID" 2>/dev/null || true
    }
    exit 130
}
trap cleanup EXIT
trap on_signal INT TERM

for ((i=0; i<NUM_DRONES; i++)); do
    POSE=$(awk -v i="$i" -v cols="$COLS" -v rows="$ROWS" -v s="$SPACING" '
      BEGIN {printf "%.2f,%.2f", (i%cols-(cols-1)/2)*s, (int(i/cols)-(rows-1)/2)*s}')
    echo "Starting drone $i at $POSE"
    if ((i==0)); then
        PX4_SYS_AUTOSTART="$AUTOSTART" PX4_SIM_MODEL="$MODEL" PX4_GZ_MODEL_POSE="$POSE" \
            "$BIN" -i "$i" >"$LOG_DIR/drone_$i.log" 2>&1 &
        PIDS+=("$!")
        sleep 5
    else
        PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART="$AUTOSTART" PX4_SIM_MODEL="$MODEL" \
            PX4_GZ_MODEL_POSE="$POSE" "$BIN" -i "$i" >"$LOG_DIR/drone_$i.log" 2>&1 &
        PIDS+=("$!")
        sleep 2
    fi
done

echo "Waiting for PX4/MAVSDK readiness; the mission clock starts after connection."
(cd "$CODE_DIR" && SWARM_NUM_DRONES="$NUM_DRONES" SWARM_SPACING_M="$SPACING" \
    SWARM_LOG_DIR="$LOG_DIR" SWARM_EXTRA_PORT_BASE="$EXTRA_PORT_BASE" \
    exec "$VENV_PYTHON" "$COORD_SCRIPT") &
COORD_PID=$!
set +e
wait "$COORD_PID"
STATUS=$?
set -e
exit "$STATUS"