cmake_minimum_required(VERSION 3.8)
project(roborio_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Protobuf
include(FindProtobuf)
find_package(Protobuf REQUIRED)

# ZMQ
find_package(PkgConfig)
pkg_check_modules(PC_ZeroMQ QUIET zmq)

find_path(ZeroMQ_INCLUDE_DIR
  NAMES zmq.hpp
  PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
  )
find_library(ZeroMQ_LIBRARY
  NAMES zmq
  PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
  )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

file(GLOB mesages_src CONFIGURE_DEPENDS "messages_include/*.pb.cc")

add_library(roborio_talonsrx SHARED src/roborio_talonsrx.cpp ${mesages_src})

target_include_directories(
  roborio_talonsrx
  PRIVATE
  include
  messages_include
  PUBLIC
  ${ZeroMQ_INCLUDE_DIR}
  ${PROTOBUF_INCLUDE_DIR}
)

target_link_libraries(roborio_talonsrx ${ZeroMQ_LIBRARY} ${PROTOBUF_LIBRARY})

ament_target_dependencies(
  roborio_talonsrx
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface roborio_talonsrx.xml)

install(
  TARGETS roborio_talonsrx
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_load_roborio_talonsrx test/test_load_roborio_talonsrx)
  ament_target_dependencies(test_load_roborio_talonsrx hardware_interface pluginlib ros2_control_test_assets)
endif()

ament_export_libraries(roborio_talonsrx)

ament_package()
