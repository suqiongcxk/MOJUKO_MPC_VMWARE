#!/bin/bash
set -e
cd ~/creeper_ws/src/Quadruped-Control-OCS2-ROS2/mujoco/mujoco-3.2.2

g++ -std=c++17 -O2 \
  -I include \
  simulate/main.cc \
  simulate/simulate.cc \
  simulate/glfw_adapter.cc \
  simulate/glfw_dispatch.cc \
  simulate/platform_ui_adapter.cc \
  simulate/lodepng.cpp \
  -L lib -lmujoco \
  $(pkg-config --cflags --libs glfw3) \
  -lpthread -ldl \
  -o bin/simulate

echo "编译完成: bin/simulate"
