cmake_minimum_required(VERSION 3.8)
project(omx_replay)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config ros2_control
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/record_trajectory.py
    scripts/replay_trajectory.py
    scripts/replay_dual_followers.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
