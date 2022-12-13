cmake_minimum_required(VERSION 3.0.2)
project(ouster_flir_calibration)

set (pcl_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(catkin REQUIRED dynamic_reconfigure)
find_package(catkin REQUIRED COMPONENTS
  pcl
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  dynamic_reconfigure
)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/dynamic_params.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv
  CATKIN_DEPENDS dynamic_reconfigure
#  LIBRARIES ouster_flir_calibration
#  CATKIN_DEPENDS pcl pcl_ros roscpp sensor_msgs cv_bridge image_transport std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(dynamic_reconfigure src/main_dyn.cpp)
target_link_libraries(dynamic_reconfigure ${catkin_LIBRARIES})
add_dependencies(dynamic_reconfigure ${PROJECT_NAME}_gencfg)

add_executable(capture src/main_capture.cpp)
target_link_libraries(capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(extract_lidar_points src/main_extract_lidar_points.cpp)
target_link_libraries(extract_lidar_points ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(extract_lidar_points ${PROJECT_NAME}_gencfg)

add_executable(extract_image_points src/main_extract_image_points.cpp)
target_link_libraries(extract_image_points ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(calibrate src/main_calibrate.cpp)
target_link_libraries(calibrate ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(calibrate ${PROJECT_NAME}_gencfg)

add_executable(result src/main_result.cpp)
target_link_libraries(result ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(result ${PROJECT_NAME}_gencfg)