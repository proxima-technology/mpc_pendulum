cmake_minimum_required(VERSION 3.5)
project(mpc_pendulum)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 )
endif()


# Find dependencies: casadi
set(CMAKE_MODULE_PATH $ENV{HOME}/casadi/cmake)
find_package(CASADI REQUIRED)
find_package(IPOPT REQUIRED)
include_directories( ${CASADI_INCLUDE_DIR} ${IPOPT_LIBRARY_DIRS})

# Find dependencies: ros and dynamixel
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(include)


# Build
add_executable(do_experiment_node src/do_experiment_node.cpp)
ament_target_dependencies(do_experiment_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)


target_link_libraries(do_experiment_node
  ${CASADI_LIBRARIES}
  ${IPOPT_LIBRARIES}
)


# Install
install(TARGETS
  do_experiment_node
  DESTINATION lib/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
