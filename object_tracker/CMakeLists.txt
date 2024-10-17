cmake_minimum_required(VERSION 3.8)
project(object_tracker)

add_subdirectory(dependencies/libobjecttracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED) # CPP RosNode needs this
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PCL REQUIRED)  # for libobjecttracker
find_package(pcl_conversions REQUIRED)
find_package(tf2)
find_package(tf2_ros)

find_package(object_tracker_interfaces REQUIRED)


add_executable(tracker src/tracker.cpp)
target_include_directories(tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tracker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(tracker
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  pcl_conversions
  tf2
  tf2_ros
  object_tracker_interfaces)



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

install(TARGETS tracker DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

include_directories(
  ${PCL_INCLUDE_DIRS}
  dependencies/libobjecttracker/include  
)



target_link_libraries(tracker libobjecttracker)

ament_package()
