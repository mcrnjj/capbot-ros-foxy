cmake_minimum_required(VERSION 3.5)
project(test_bot)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

# Assets del robot real (sin worlds/aruco_assets de simulacion).
# meshes: STL del chasis v2 referenciado como package://test_bot/meshes/...
install(
  DIRECTORY config description launch maps meshes
  DESTINATION share/${PROJECT_NAME}
)

# Nodos Python (instalados como ejecutables del paquete; RENAME quita el .py).
install(PROGRAMS
  scripts/aruco_localizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aruco_localizer
)
install(PROGRAMS
  scripts/csi_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME csi_camera_node
)
install(PROGRAMS
  scripts/esp32_serial_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME esp32_serial_bridge
)
install(PROGRAMS
  scripts/fake_odom.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_odom
)
install(PROGRAMS
  scripts/teleop_gateway.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_gateway
)
install(PROGRAMS
  scripts/gui_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gui_bridge_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
