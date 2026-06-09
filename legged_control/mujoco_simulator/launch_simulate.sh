#!/bin/bash
SIMULATE=/home/cxk/.mujoco/mujoco-3.4.0/bin/simulate
MODEL_DIR=/home/cxk/quadruped_mpc_ws/src/Quadruped-Control-OCS2-ROS2/legged_control/mujoco_simulator/models

ROBOT="${1:-creeper}"

case "$ROBOT" in
  creeper) XML="$MODEL_DIR/creeper/urdf/creeper.xml" ;;
  b1)      XML="$MODEL_DIR/b1/urdf/b1_original.xml" ;;
  a1)      XML="$MODEL_DIR/a1/urdf/a1_revised.xml" ;;
  *) echo "用法: $0 [creeper|b1|a1]"; exit 1 ;;
esac

exec "$SIMULATE" "$XML"
