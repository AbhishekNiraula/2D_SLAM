cmake_minimum_required(VERSION 3.8)
project(esp32_bot)
 
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
 
# ── Install directories ──────────────────────────────────────
install(DIRECTORY urdf   DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz   DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
 
# ── Install tf_relay as a runnable script ────────────────────
install(PROGRAMS
  scripts/tf_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tf_relay
)

install(PROGRAMS
  scripts/explorer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME explorer
)

install(PROGRAMS
  scripts/rotate_scan.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rotate_scan
)
install(PROGRAMS
  scripts/stepped_rotate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stepped_rotate
)

install(PROGRAMS
  scripts/tof_mapper.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tof_mapper
)

install(PROGRAMS
  scripts/wall_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wall_follower
)
 
ament_package()
