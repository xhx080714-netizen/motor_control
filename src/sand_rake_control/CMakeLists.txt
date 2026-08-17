cmake_minimum_required(VERSION 3.8)
project(sand_rake_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sand_rake_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelRpm.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(diff_drive_kinematics
  src/diff_drive_kinematics.cpp
)

target_include_directories(diff_drive_kinematics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
add_library(safety_state_machine
  src/safety_state_machine.cpp
)

target_include_directories(safety_state_machine
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(modbus_rtu
  src/modbus_rtu.cpp
)

target_include_directories(modbus_rtu
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(rs232_transport
  src/modbus_rtu_client.cpp
  src/rs232_transport.cpp
)

target_include_directories(rs232_transport
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rs232_transport
  modbus_rtu
  Threads::Threads
)

add_executable(teleop_node
  src/teleop_node.cpp
)
ament_target_dependencies(teleop_node
  rclcpp
  geometry_msgs
)

add_executable(safety_controller
  src/safety_controller.cpp
)

ament_target_dependencies(safety_controller
  rclcpp
  geometry_msgs
  sand_rake_interfaces
  std_srvs
)

target_link_libraries(
  safety_controller
  safety_state_machine
  "${cpp_typesupport_target}"
)

add_executable(chassis_controller_node
  src/chassis_controller_node.cpp
)
ament_target_dependencies(chassis_controller_node
  geometry_msgs
  rclcpp
)
target_link_libraries(chassis_controller_node
  diff_drive_kinematics
  "${cpp_typesupport_target}"
)

add_executable(chassis_mock_node
  src/chassis_mock_node.cpp
)
ament_target_dependencies(chassis_mock_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)
target_link_libraries(chassis_mock_node
  diff_drive_kinematics
  "${cpp_typesupport_target}"
)

install(
  TARGETS
    teleop_node
    safety_controller
    chassis_controller_node
    chassis_mock_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS
    safety_state_machine
    diff_drive_kinematics
    modbus_rtu
    rs232_transport
  EXPORT export_safety_state_machine
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  # This repository is also used directly as a colcon workspace root. Keep
  # linters scoped to source files so generated build/install trees are not
  # inspected as project source.
  ament_lint_cmake(CMakeLists.txt)
  ament_uncrustify(include src
    test/test_diff_drive_kinematics.cpp
    test/test_modbus_rtu.cpp
    test/test_rs232_transport.cpp
    test/test_safety_state_machine.cpp
  )
  ament_xmllint(
    package.xml
    launch/chassis_mock.launch.xml
  )
  ament_add_gtest(test_safety_state_machine
  test/test_safety_state_machine.cpp
  )

  target_link_libraries(test_safety_state_machine
  safety_state_machine
  )

  ament_add_gtest(test_diff_drive_kinematics
    test/test_diff_drive_kinematics.cpp
  )
  target_link_libraries(test_diff_drive_kinematics
    diff_drive_kinematics
  )

  ament_add_gtest(test_modbus_rtu
    test/test_modbus_rtu.cpp
  )
  target_link_libraries(test_modbus_rtu
    modbus_rtu
  )

  ament_add_gtest(test_rs232_transport
    test/test_rs232_transport.cpp
  )
  target_link_libraries(test_rs232_transport
    rs232_transport
    Threads::Threads
  )

  ament_add_pytest_test(test_safety_integration
    test/test_safety_integration.py
    TIMEOUT 90
    ENV
      "SAND_RAKE_SAFETY_CONTROLLER=$<TARGET_FILE:safety_controller>"
  )

endif()

ament_export_include_directories(include)
ament_export_targets(export_safety_state_machine)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
