cmake_minimum_required(VERSION 3.20)
project(create_driver)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(libcreate REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(create_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} PRIVATE
  create
  create_msgs::create_msgs
  diagnostic_msgs::diagnostic_msgs
  diagnostic_updater::diagnostic_updater
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)

if(BUILD_TESTING)
  # Skip copyright lint and uncrustify
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # Set language to C++, otherwise cppcheck assumes header file with .h is C
  set(ament_cmake_cppcheck_LANGUAGE c++)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()
