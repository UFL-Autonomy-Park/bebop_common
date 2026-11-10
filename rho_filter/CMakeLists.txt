cmake_minimum_required(VERSION 3.8)
project(rho_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} src/RhoFilter.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

add_executable(rho_filter_node src/rho_filter_node.cpp)

target_include_directories(rho_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(rho_filter_node ${PROJECT_NAME})

ament_target_dependencies(rho_filter_node
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2
  tf2_ros
  tf2_eigen 
  Eigen3
)

install(TARGETS ${PROJECT_NAME} rho_filter_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

ament_package()