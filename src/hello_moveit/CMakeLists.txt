cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(gazebo_msgs REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(one_grasp src/one_grasp.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(one_grasp PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)
ament_target_dependencies(
  one_grasp
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "gazebo_msgs"
)

install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS one_grasp
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


# Install additional directories
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)