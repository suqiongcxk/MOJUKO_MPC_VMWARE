cmake_minimum_required(VERSION 3.8)
project(user_command)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

ament_package()
