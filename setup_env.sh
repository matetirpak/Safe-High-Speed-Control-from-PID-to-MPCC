#!/usr/bin/env bash
# Source this (don't execute) to set up a shell for the Carla MPC/PID testbed:
#   source setup_env.sh
#
# It sources ROS2 Humble + this workspace's install overlay, activates the venv
# (Carla wheel + numpy/pyyaml/networkx), and puts CARLA's PythonAPI `agents`
# package on PYTHONPATH (needed for the PID + GlobalRoutePlanner). Keep
# ROS_DOMAIN_ID identical in the shell that launches Carla.

# --- Resolve this workspace's root regardless of where it's sourced from ------
_WS="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

# --- ROS2 Humble --------------------------------------------------------------
source /opt/ros/humble/setup.bash

# --- This workspace overlay ---------------------------------------------------
if [ -f "$_WS/install/setup.bash" ]; then
  source "$_WS/install/setup.bash"
else
  echo "[setup_env] note: workspace not built yet -- run: (cd $_WS && colcon build)"
fi

# --- venv (Carla wheel + numpy + pyyaml + networkx) ---------------------------
# colcon-installed console scripts run under /usr/bin/python3 (their shebang),
# which does NOT consult the venv -- so we ALSO expose the venv's site-packages on
# PYTHONPATH. That lets launched nodes import the venv-only `carla` wheel + deps
# while using the system rclpy (system py3.10 == venv py3.10 -> ABI compatible).
if [ -f "$_WS/.venv/bin/activate" ]; then
  source "$_WS/.venv/bin/activate"
  _VENV_SP="$_WS/.venv/lib/python3.10/site-packages"
  [ -d "$_VENV_SP" ] && export PYTHONPATH="$_VENV_SP:$PYTHONPATH"
else
  echo "[setup_env] note: venv missing -- create it (see README) for the Carla wheel."
fi

# --- CARLA PythonAPI `agents` package (PID + GlobalRoutePlanner) ---------------
# `import agents.navigation...` and the GlobalRoutePlanner live in the CARLA
# distribution, NOT the wheel. Add PythonAPI/carla to PYTHONPATH.
export CARLA_ROOT="${CARLA_ROOT:-$HOME/autonomousdriving/Carla/CARLA_0.9.16}"
if [ -d "$CARLA_ROOT/PythonAPI/carla/agents" ]; then
  export PYTHONPATH="$CARLA_ROOT/PythonAPI/carla:$PYTHONPATH"
else
  echo "[setup_env] note: CARLA agents not found at $CARLA_ROOT/PythonAPI/carla -- set CARLA_ROOT."
fi

# --- acados (MPC/MPCC solver backend) -----------------------------------------
# Built once into ~/acados (see docs/mpc.md). The Python interface imports
# without these, but AcadosOcpSolver(...) fails at runtime without them.
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}"
if [ -d "$ACADOS_SOURCE_DIR/lib" ]; then
  export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH"
  # acados_template is installed *editable* (a .pth import hook). The MPC node runs
  # under system python3 with the venv on PYTHONPATH, and PYTHONPATH dirs don't
  # execute .pth hooks -- so expose the acados_template SOURCE directly.
  [ -d "$ACADOS_SOURCE_DIR/interfaces/acados_template" ] && \
    export PYTHONPATH="$ACADOS_SOURCE_DIR/interfaces/acados_template:$PYTHONPATH"
else
  echo "[setup_env] note: acados not found at $ACADOS_SOURCE_DIR -- MPC needs it (see docs/mpc.md)."
fi

# --- DDS / discovery (Carla 0.9.16 is hardcoded to Fast-DDS) -------------------
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[setup_env] ROS_DISTRO=$ROS_DISTRO  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "[setup_env] CARLA_ROOT=$CARLA_ROOT"
echo "[setup_env] ready. Start Carla with the SAME ROS_DOMAIN_ID:  ./CarlaUE4.sh --ros2"
