cmake_minimum_required(VERSION 3.12)
project(humanoid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_path(ZMQ_INCLUDE_DIR zmq.h REQUIRED)
find_library(ZMQ_LIBRARY zmq REQUIRED)
find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp REQUIRED)

find_path(HUMANOID_COMMON_INCLUDE_DIR
  NAMES robot_base.h transport_executor.h
  HINTS
    "$ENV{SDK_ROOT}/output/staging"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../output/staging"
  PATH_SUFFIXES include
)
find_library(ROBOT_BASE_LIB robot_base
  HINTS
    "$ENV{SDK_ROOT}/output/staging"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../output/staging"
  PATH_SUFFIXES lib
)
find_library(TRANSPORT_EXECUTOR_LIB transport_executor
  HINTS
    "$ENV{SDK_ROOT}/output/staging"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../output/staging"
  PATH_SUFFIXES lib
)
find_library(OPERATOR_CLIENT_LIB operator_client
  HINTS
    "$ENV{SDK_ROOT}/output/staging"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../output/staging"
  PATH_SUFFIXES lib
)

if(NOT HUMANOID_COMMON_INCLUDE_DIR OR NOT ROBOT_BASE_LIB OR
  NOT TRANSPORT_EXECUTOR_LIB OR NOT OPERATOR_CLIENT_LIB)
  message(FATAL_ERROR
    "humanoid_common libraries not found. Build/install application/native/humanoid_common first "
    "and add its install prefix to CMAKE_PREFIX_PATH.")
endif()

get_filename_component(HUMANOID_COMMON_LIB_DIR "${TRANSPORT_EXECUTOR_LIB}" DIRECTORY)

add_executable(humanoid_nav_rl_bridge_node src/humanoid_nav_rl_bridge_node.cpp)
target_include_directories(humanoid_nav_rl_bridge_node PRIVATE ${HUMANOID_COMMON_INCLUDE_DIR})
ament_target_dependencies(humanoid_nav_rl_bridge_node
  geometry_msgs
  rclcpp
  std_msgs
)
target_link_libraries(humanoid_nav_rl_bridge_node
  ${OPERATOR_CLIENT_LIB}
  ${ROBOT_BASE_LIB}
  pthread
)
set_target_properties(humanoid_nav_rl_bridge_node PROPERTIES
  BUILD_RPATH "${HUMANOID_COMMON_LIB_DIR}"
  INSTALL_RPATH "${HUMANOID_COMMON_LIB_DIR}"
)

# ROS velocity and ZMQ compatibility client for the native operator service.
add_executable(humanoid_cmd_vel_hmi_node
  src/humanoid_cmd_vel_hmi_node.cpp
)
target_include_directories(humanoid_cmd_vel_hmi_node PRIVATE
  ${HUMANOID_COMMON_INCLUDE_DIR}
  ${ZMQ_INCLUDE_DIR}
  ${NLOHMANN_JSON_INCLUDE_DIR}
)
ament_target_dependencies(humanoid_cmd_vel_hmi_node
  geometry_msgs
  rclcpp
)
target_link_libraries(humanoid_cmd_vel_hmi_node
  ${OPERATOR_CLIENT_LIB}
  ${ROBOT_BASE_LIB}
  ${ZMQ_LIBRARY}
  pthread
)
set_target_properties(humanoid_cmd_vel_hmi_node PROPERTIES
  BUILD_RPATH "${HUMANOID_COMMON_LIB_DIR}"
  INSTALL_RPATH "${HUMANOID_COMMON_LIB_DIR}"
)

set(HUMANOID_TRANSPORT_PACKET_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../application/native/humanoid_common/src/transport/transport_executor"
)
add_executable(humanoid_head_tf_node src/humanoid_head_tf_node.cpp)
target_include_directories(humanoid_head_tf_node PRIVATE
  ${HUMANOID_COMMON_INCLUDE_DIR}
  ${HUMANOID_TRANSPORT_PACKET_DIR}
)
set_property(SOURCE src/humanoid_head_tf_node.cpp APPEND PROPERTY
  OBJECT_DEPENDS "${HUMANOID_TRANSPORT_PACKET_DIR}/transport_packet.h"
)
ament_target_dependencies(humanoid_head_tf_node
  geometry_msgs
  rclcpp
  tf2_ros
)
target_link_libraries(humanoid_head_tf_node
  ${ROBOT_BASE_LIB}
  pthread
  rt
)
set_target_properties(humanoid_head_tf_node PROPERTIES
  BUILD_RPATH "${HUMANOID_COMMON_LIB_DIR}"
  INSTALL_RPATH "${HUMANOID_COMMON_LIB_DIR}"
)

add_executable(tf_to_odom_node src/tf_to_odom_node.cpp)
ament_target_dependencies(tf_to_odom_node
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)

install(TARGETS humanoid_nav_rl_bridge_node humanoid_cmd_vel_hmi_node
  humanoid_head_tf_node tf_to_odom_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/asr_action_control.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/asr_backend.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY assets
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # scripts/lint/lint_cpp.sh enforces the repository's four-space C++ style.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
