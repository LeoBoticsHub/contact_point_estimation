cmake_minimum_required(VERSION 3.5)
project(contact_point_estimation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Libraries
add_library(contact_point_estimator
  src/ContactPointEstimatorParams.cpp
  src/SurfaceNormalEstimatorParams.cpp
  src/ContactPointEstimator.cpp
  src/SurfaceNormalEstimator.cpp
)
ament_target_dependencies(contact_point_estimator
  rclcpp
  geometry_msgs
  std_srvs
)

add_library(surface_normal_estimator
  src/SurfaceNormalEstimatorParams.cpp
  src/SurfaceNormalEstimator.cpp
)
ament_target_dependencies(surface_normal_estimator
  rclcpp
  geometry_msgs
  std_srvs
)

add_library(surface_tracing_controller
  src/SurfaceTracingController.cpp
)
ament_target_dependencies(surface_tracing_controller
  rclcpp
  geometry_msgs
  std_srvs
)

# Node executable
add_executable(contact_point_estimation_node
  src/ContactPointEstimatorParams.cpp
  src/SurfaceNormalEstimatorParams.cpp
  src/ContactPointEstimator.cpp
  src/SurfaceNormalEstimator.cpp
  src/contact_point_estimation_node.cpp
)
ament_target_dependencies(contact_point_estimation_node
  rclcpp
  geometry_msgs
  std_srvs
)
target_link_libraries(contact_point_estimation_node
  contact_point_estimator
  surface_normal_estimator
  surface_tracing_controller
  ${Boost_LIBRARIES}
)

# Install targets
install(TARGETS
  contact_point_estimation_node
  contact_point_estimator
  surface_normal_estimator
  surface_tracing_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Install config and launch folders
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export package
ament_package()
