cmake_minimum_required(VERSION 3.8)
project(search)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(target_bbox_msgs REQUIRED)
find_package(ros_ign_interfaces REQUIRED)

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


include_directories(
 	include
)


set(list
sUAV
manager
c47v
)

foreach(l ${list})
  add_executable(${l} src/${l}.cpp)
  ament_target_dependencies(${l} rclcpp sensor_msgs nav_msgs target_bbox_msgs ros_ign_interfaces)
  install(TARGETS
  ${l}
    DESTINATION lib/${PROJECT_NAME})
  
endforeach(l ${list})

ament_package()
