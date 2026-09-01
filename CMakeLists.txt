cmake_minimum_required(VERSION 3.12)
project(humanoid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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

if(NOT HUMANOID_COMMON_INCLUDE_DIR OR NOT ROBOT_BASE_LIB OR NOT TRANSPORT_EXECUTOR_LIB)
  message(FATAL_ERROR
    "robot_base/transport_executor not found. Build/install application/native/humanoid_common first "
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
  ${TRANSPORT_EXECUTOR_LIB}
  ${ROBOT_BASE_LIB}
  pthread
)
set_target_properties(humanoid_nav_rl_bridge_node PROPERTIES
  BUILD_RPATH "${HUMANOID_COMMON_LIB_DIR}"
  INSTALL_RPATH "${HUMANOID_COMMON_LIB_DIR}"
)

install(TARGETS humanoid_nav_rl_bridge_node
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
