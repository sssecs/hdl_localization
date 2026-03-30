cmake_minimum_required(VERSION 3.8)
project(hdl_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_compile_options(-O3)
else()
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
endif()

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(ndt_omp REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(hdl_global_localization REQUIRED)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
endif()

########################
## message generation ##
########################
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScanMatchingStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
)

# components
add_library(hdl_localization_component SHARED
  src/hdl_localization/pose_estimator.cpp
  apps/hdl_localization_component.cpp
)
ament_target_dependencies(hdl_localization_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  ndt_omp
  fast_gicp
  hdl_global_localization
)
target_link_libraries(hdl_localization_component
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}
)
rclcpp_components_register_nodes(hdl_localization_component
  "hdl_localization::HdlLocalizationComponent"
)


add_library(globalmap_server_component SHARED
  apps/globalmap_server_component.cpp
)
ament_target_dependencies(globalmap_server_component
  rclcpp
  rclcpp_components
  pcl_conversions
  fast_gicp
)
target_link_libraries(globalmap_server_component
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}
)
rclcpp_components_register_nodes(globalmap_server_component
  "hdl_localization::GlobalmapServerComponent"
)

# ========================
# Install
# ========================
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  hdl_localization_component
  globalmap_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# ========================
# Export
# ========================
ament_package()