#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-nav2pp}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
export NAV2PP_BROWSER_PORT="${NAV2PP_BROWSER_PORT:-6080}"
export NAV2PP_VNC_PORT="${NAV2PP_VNC_PORT:-5900}"
export NAV2PP_NOVNC_WEB="${NAV2PP_NOVNC_WEB:-/usr/share/novnc}"

mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"

nav2pp_safe_source() {
  local target="$1"
  local had_u=0
  case $- in
    *u*) had_u=1 ;;
  esac
  set +u
  # shellcheck disable=SC1090
  source "${target}"
  if [ "${had_u}" -eq 1 ]; then
    set -u
  fi
}

nav2pp_safe_source /opt/ros/jazzy/setup.bash

nav2pp_tb3_initial_pose_payload() {
  printf '%s\n' '{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25,0.0,0.0,0.0,0.0,0.0,0.0,0.25,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.06853891945200942]}}'
}

nav2pp_seed_initial_pose() {
  local attempt=0
  local payload=""

  payload="$(nav2pp_tb3_initial_pose_payload)"
  until ros2 topic list 2>/dev/null | grep -qx '/initialpose'; do
    attempt=$((attempt + 1))
    if [ "${attempt}" -ge 60 ]; then
      echo "Timed out waiting for /initialpose; skipping AMCL bootstrap." >&2
      return 1
    fi
    sleep 1
  done

  sleep 2
  echo "Publishing initial pose for containerized AMCL bootstrap." >&2
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$(nav2pp_tb3_initial_pose_payload)" >/dev/null
}

cleanup() {
  local code=$?
  pkill -TERM -P $$ >/dev/null 2>&1 || true
  wait >/dev/null 2>&1 || true
  exit "${code}"
}
trap cleanup EXIT INT TERM

Xvfb "${DISPLAY}" -screen 0 1680x1050x24 -ac +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &

for _ in $(seq 1 30); do
  if [ -S "/tmp/.X11-unix/X${DISPLAY#:}" ]; then
    break
  fi
  sleep 1
done

fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display "${DISPLAY}" -forever -shared -nopw -rfbport "${NAV2PP_VNC_PORT}" >/tmp/x11vnc.log 2>&1 &
websockify --web="${NAV2PP_NOVNC_WEB}" "${NAV2PP_BROWSER_PORT}" "localhost:${NAV2PP_VNC_PORT}" >/tmp/novnc.log 2>&1 &

echo "Nav2++ browser UI ready at http://127.0.0.1:${NAV2PP_BROWSER_PORT}/vnc.html?autoconnect=1&resize=scale"

dbus-run-session -- bash -lc '
  set -euo pipefail
  nav2pp_safe_source() {
    local target="$1"
    local had_u=0
    case $- in
      *u*) had_u=1 ;;
    esac
    set +u
    source "${target}"
    if [ "${had_u}" -eq 1 ]; then
      set -u
    fi
  }
  nav2pp_safe_source /opt/ros/jazzy/setup.bash
  export DISPLAY='"${DISPLAY}"'
  export QT_X11_NO_MITSHM=1
  export LIBGL_ALWAYS_SOFTWARE=1
  export TURTLEBOT3_MODEL='"${TURTLEBOT3_MODEL}"'
  export NAV2PP_REPO_ROOT='"${NAV2PP_REPO_ROOT:-/work/Nav2++}"'
  ros2 launch "${NAV2PP_REPO_ROOT}/docker/nav2pp-desktop/launch/nav2pp_tb3_browser_launch.py"
' &
launch_pid=$!

nav2pp_seed_initial_pose || true
wait "${launch_pid}"
