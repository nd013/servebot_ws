cmake_minimum_required(VERSION 3.8)
project(servebot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface QUIET)
find_package(pluginlib QUIET)

# ── Hardware Interface Plugin (optional — needs ros-jazzy-ros2-control) ──

if(hardware_interface_FOUND AND pluginlib_FOUND)
  add_library(servebot_interface SHARED src/servebot_interface.cpp)

  target_include_directories(servebot_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(servebot_interface
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
  )

  pluginlib_export_plugin_description_file(hardware_interface servebot_firmware.xml)

  install(TARGETS servebot_interface
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY include/
    DESTINATION include
  )
else()
  message(WARNING "hardware_interface not found — skipping servebot_interface plugin build. "
                  "Install with: sudo apt install ros-jazzy-ros2-control")
endif()

# ── Python scripts ────────────────────────────────────────────────────────

install(PROGRAMS
  servebot_firmware/simple_serial_transmitter.py
  servebot_firmware/simple_serial_receiver.py
  servebot_firmware/mpu6050_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Launch and config ─────────────────────────────────────────────────────

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── Testing ───────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
