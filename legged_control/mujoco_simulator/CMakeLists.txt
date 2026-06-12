cmake_minimum_required(VERSION 3.8)
project(mujoco_simulator)

set(MUJOCO_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco/mujoco-3.2.2)
set(MUJOCO_INCLUDE_DIRS ${MUJOCO_BASE_DIR}/include)
set(MUJOCO_LIBRARIES ${MUJOCO_BASE_DIR}/lib/libmujoco.so)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(glfw3 REQUIRED)

include_directories(
  include
  ${MUJOCO_INCLUDE_DIRS}
)

add_executable(mujoco_simulator src/SimulatorNode.cpp src/MujocoSimulation.cpp)
ament_target_dependencies(mujoco_simulator rclcpp)
target_link_libraries(mujoco_simulator ${MUJOCO_LIBRARIES} glfw)

install(TARGETS mujoco_simulator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/)

ament_package()
