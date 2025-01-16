cmake_minimum_required(VERSION 3.5)
project(spatio_temporal_voxel_layer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
# find_package(point_cloud_msg_wrapper REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
set(decomp_rviz_plugins_FOUND 1 )
find_package(decomp_rviz_plugins REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(kindr REQUIRED)
find_package(kindr_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(PCL REQUIRED COMPONENTS filters common_utility)
find_package(Threads REQUIRED)

if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # This package fails to build on Debian Stretch with a linking error against
  # 'Qt5::Widgets'.  This is a transitive dependency that comes in to PCL via
  # the PCL dependency on VTK.  However, we don't actually care about the Qt
  # dependencies for this package, so just remove them.  This is similar to the
  # workaround in https://github.com/ros-perception/perception_pcl/pull/151,
  # and can be removed when Stretch goes out of support.
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt")
endif()
find_package(octomap REQUIRED)
find_package(decomp_util REQUIRED)
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB/")
find_package(OpenVDB REQUIRED)
# https://www.openvdb.org/documentation/doxygen/build.html
remove_definitions(-DDISABLE_LIBUSB-1.0)

set(dependencies
  nav2_costmap_2d
  geometry_msgs
  pluginlib
  sensor_msgs
  std_msgs
  nav_msgs
  pcl_msgs
  std_srvs
  cv_bridge
  laser_geometry
  message_filters
  # point_cloud_msg_wrapper
  diagnostic_updater
  decomp_ros_msgs
  image_transport
  pcl_conversions
  rclcpp
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  visualization_msgs
  builtin_interfaces
  pcl_ros
  rclcpp_components
  decomp_rviz_plugins
  rclcpp_lifecycle
  grid_map_core
  grid_map_ros
  grid_map_msgs
  kindr
  kindr_ros
  rcl_interfaces
)

set(library_name spatio_temporal_voxel_layer_core)

add_definitions(${EIGEN3_DEFINITIONS})


add_library(${PROJECT_NAME}_pcl_types SHARED src/elevation_mapping/PointXYZRGBConfidenceRatio.cpp)
target_link_libraries(${PROJECT_NAME}_pcl_types ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_pcl_types ${dependencies})


set(srv_files "srv/SaveGrid.srv")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

include_directories(
    include
    ${BOOST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenVDB_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${DECOMP_UTIL_INCLUDE_DIRS}
)

add_library(${library_name} SHARED
            src/spatio_temporal_voxel_layer.cpp
            src/spatio_temporal_voxel_grid.cpp
            src/measurement_buffer.cpp
            src/frustum_models/depth_camera_frustum.cpp
            src/frustum_models/three_dimensional_lidar_frustum.cpp
            src/vdb2pc.cpp
            src/freespace_segmentation/free_space_segment.cpp 
            src/elevation_mapping/ElevationMapping.cpp
            src/elevation_mapping/ElevationMap.cpp
            src/elevation_mapping/RobotMotionMapUpdater.cpp
            src/postprocessing/PostprocessorPool.cpp
            src/postprocessing/PostprocessingWorker.cpp
            src/postprocessing/PostprocessingPipelineFunctor.cpp
            src/input_sources/InputSourceManager.cpp
            src/input_sources/Input.cpp
            src/sensor_processors/SensorProcessorBase.cpp
            src/sensor_processors/PerfectSensorProcessor.cpp
            src/tools/node_utils.cpp 
            src/tools/parameters_handler.cpp 
)

target_link_libraries(${library_name} "${cpp_typesupport_target}")
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(${library_name} ${dependencies})
target_link_libraries(${library_name}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenVDB_LIBRARIES}
  OpenVDB::openvdb
  ${OCTOMAP_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} dynamicedt3d
  ${PROJECT_NAME}_pcl_types
  pthread
  Boost::thread
)

# add_executable(elevation_mapping src/elevation_mapping/elevation_mapping_node.cpp)
# ament_target_dependencies(elevation_mapping ${dependencies})
# target_link_libraries(elevation_mapping ${library_name})
# install(TARGETS
#   elevation_mapping
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY launch config param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_pcl_types  ${library_name}  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  # RUNTIME DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin 
)

install(
  TARGETS ${library_name} ${PROJECT_NAME}_pcl_types
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)


install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# ament_export_dependencies(rosidl_default_runtime)
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${library_name} ${PROJECT_NAME}_pcl_types)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
# ament_export_include_directories(include/${PROJECT_NAME})
ament_package()
