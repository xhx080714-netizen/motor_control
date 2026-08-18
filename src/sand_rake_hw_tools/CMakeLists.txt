cmake_minimum_required(VERSION 3.8)
project(sand_rake_hw_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sand_rake_control REQUIRED)
find_package(Threads REQUIRED)

add_executable(small_car_four_wheel_jog_once
  src/small_car_four_wheel_jog_once.cpp
)
target_link_libraries(small_car_four_wheel_jog_once
  sand_rake_control::modbus_rtu
  sand_rake_control::rs232_transport
  Threads::Threads
)

install(
  TARGETS
    small_car_four_wheel_jog_once
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_flake8(test/test_small_car_four_wheel_jog.py)
  ament_lint_cmake(CMakeLists.txt)
  ament_uncrustify(src)
  ament_xmllint(package.xml)
  enable_testing()
  add_executable(small_car_four_wheel_jog_once_pty_test
    src/small_car_four_wheel_jog_once.cpp
  )
  target_compile_definitions(small_car_four_wheel_jog_once_pty_test PRIVATE
    SAND_RAKE_HW_TOOLS_TEST_ALLOW_PTY=1
  )
  target_link_libraries(small_car_four_wheel_jog_once_pty_test
    sand_rake_control::modbus_rtu
    sand_rake_control::rs232_transport
    Threads::Threads
  )

  add_test(
    NAME small_car_four_wheel_jog_pty
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_small_car_four_wheel_jog.py
      $<TARGET_FILE:small_car_four_wheel_jog_once>
      $<TARGET_FILE:small_car_four_wheel_jog_once_pty_test>
  )
  set_tests_properties(small_car_four_wheel_jog_pty PROPERTIES TIMEOUT 15)
endif()

ament_package()
