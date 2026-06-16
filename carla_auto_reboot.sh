#!/usr/bin/env bash
# carla_auto_reboot.sh — keep a Carla server alive. Whenever Carla stops responding (the
# Carla-0.9.16-on-Blackwell PhysX segfault, "length > epsilon" / Signal 11), relaunch it. Re-launch the
# control stack (run_pid.sh / run_mpc.sh) once Carla is back up.
#
#   CARLA_ROOT=/path/to/CARLA_0.9.16 ./carla_auto_reboot.sh           # -RenderOffScreen
#   CARLA_ROOT=/path/to/CARLA_0.9.16 ./carla_auto_reboot.sh --ros2    # + Carla's native ROS2 publisher
#
# CARLA_ROOT  : Carla install dir containing CarlaUE4.sh           (required env var)
# CARLA_PORT  : RPC port (default 2000)                            (optional env var)
# Always launches with -RenderOffScreen; --ros2 is the only accepted CLI flag.
set -u
: "${CARLA_ROOT:?set CARLA_ROOT to your Carla install dir (the folder with CarlaUE4.sh)}"
CARLA_SH="$CARLA_ROOT/CarlaUE4.sh"
[ -x "$CARLA_SH" ] || { echo "carla_auto_reboot: not found / not executable: $CARLA_SH" >&2; exit 1; }
PORT="${CARLA_PORT:-2000}"

EXTRA=""
for a in "$@"; do
  case "$a" in
    --ros2) EXTRA="--ros2" ;;
    *) echo "carla_auto_reboot: ignoring unknown arg '$a' (only --ros2 is accepted)" >&2 ;;
  esac
done

# is the RPC port accepting connections? (pure bash, no carla wheel needed)
carla_up() { timeout 2 bash -c "exec 3<>/dev/tcp/localhost/$PORT" 2>/dev/null; }

CARLA_PGID=""
shutdown() {
  echo; echo "carla_auto_reboot: stopping -> shutting Carla down"
  # kill the whole launched process group (CarlaUE4.sh + the Shipping binary child)
  [ -n "$CARLA_PGID" ] && kill -TERM -- "-$CARLA_PGID" 2>/dev/null
  fuser -k "${PORT}/tcp" >/dev/null 2>&1 || true   # belt-and-suspenders: free the port
  sleep 1
  [ -n "$CARLA_PGID" ] && kill -KILL -- "-$CARLA_PGID" 2>/dev/null
  exit 0
}
trap shutdown INT TERM
LOGDIR="${CARLA_LOGDIR:-logs/errors/carla}"; mkdir -p "$LOGDIR"
echo "carla_auto_reboot: maintaining '$CARLA_SH -RenderOffScreen $EXTRA' on port $PORT — logs in $LOGDIR/ (Ctrl-C stops the loop AND shuts Carla down)"
boots=0
while true; do
  if ! carla_up; then
    boots=$((boots + 1))
    CARLA_LOG="$LOGDIR/carla_$(date +%Y%m%d-%H%M%S)_boot$boots.log"
    echo "carla_auto_reboot: [$(date +%T)] Carla down -> launching (boot #$boots) -> $CARLA_LOG"
    fuser -k "${PORT}/tcp" >/dev/null 2>&1 || true   # clear a stale/hung instance (no-op if already gone)
    sleep 2
    # Strip NUL bytes UE4's crash handler emits: they make text editors misdetect the ASCII log as
    # UTF-16 and render it as "asian letters". tr -d keeps it clean ASCII. The bash -c wrapper is the
    # setsid group leader, so $! / kill -- -PGID still own the whole tree (Carla + tr).
    setsid bash -c '"$0" -RenderOffScreen $1 2>&1 | tr -d "\000" > "$2"' "$CARLA_SH" "$EXTRA" "$CARLA_LOG" &
    CARLA_PGID=$!   # bash -c is the group leader (setsid), so PGID == PID
    t=0
    until carla_up; do
      sleep 3; t=$((t + 3))
      [ "$t" -gt 180 ] && { echo "carla_auto_reboot: [$(date +%T)] still not up after 180s (see $CARLA_LOG); will retry"; break; }
    done
    carla_up && echo "carla_auto_reboot: [$(date +%T)] Carla up"
  fi
  sleep 5
done
