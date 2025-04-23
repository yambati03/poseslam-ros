cmake_minimum_required(VERSION 3.8)
project(poseslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(cxxopts REQUIRED)
find_package(nanoflann REQUIRED)
find_package(GTSAM REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED
  src/simpleicp.cpp
  src/pointcloud.cpp
  src/corrpts.cpp
  src/poseslam.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  cxxopts
  nanoflann
  sensor_msgs
  nav_msgs
  pcl_conversions
  geometry_msgs
  tf2
  GTSAM
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen gtsam ${PCL_LIBRARIES})

# Install launch files.
install(
  DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install library
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "poseslam::PoseSlam"
  EXECUTABLE ${PROJECT_NAME}_PoseSlam
)

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
