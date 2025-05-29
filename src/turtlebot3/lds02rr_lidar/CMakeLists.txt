cmake_minimum_required(VERSION 3.5)
project(lds02rr_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED system)

find_package(rmw_cyclonedds_cpp REQUIRED)


include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)



add_executable(lds02rr_node src/xv_11_driver.cpp src/xv11_laser.cpp)
target_link_libraries(lds02rr_node ${Boost_LIBRARIES})

ament_target_dependencies(lds02rr_node rclcpp std_msgs sensor_msgs rmw_cyclonedds_cpp)



install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  lds02rr_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()
