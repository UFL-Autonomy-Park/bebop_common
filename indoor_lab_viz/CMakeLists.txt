cmake_minimum_required(VERSION 3.8)
project(indoor_lab_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node src/indoor_lab_viz_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    geometry_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
