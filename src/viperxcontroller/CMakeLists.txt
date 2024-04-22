cmake_minimum_required(VERSION 3.8)
project(viperxcontroller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS2 interfaces
set(srv_files
  "srv/MoveToPosition.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

# Add executables
add_executable(viperctrl src/main.cpp src/viperx_controller.cpp)
add_executable(simpleclient src/simple_client.cpp)


# Add include directories
target_include_directories(viperctrl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen_INCLUDE_DIRS})

target_compile_features(viperctrl PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_export_dependencies(rosidl_default_runtime)

# Specify dependencies for targets
ament_target_dependencies(
  viperctrl
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  simpleclient
  "rclcpp"
)
# Get type support target for ROS2 interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Link the type support target to the executables
target_link_libraries(viperctrl "${cpp_typesupport_target}")
target_link_libraries(simpleclient "${cpp_typesupport_target}")


# Install launch files and executables
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS viperctrl
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS simpleclient
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
