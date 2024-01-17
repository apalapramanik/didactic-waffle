cmake_minimum_required(VERSION 3.0.2)
project(monitoring)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  interactive_markers
  pluginlib
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  cv_bridge
  std_msgs
  message_generation
  geometry_msgs
  costmap_2d
)


find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS $(EIGEN3_INCLUDE_DIRS))
endif()

catkin_python_setup()



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  H_detection.msg
  stop.msg
  Meannn.msg
  Deviation.msg 
  position.msg
  BoundingBoxes.msg
  Boundingbox.msg
  Plot.msg
  distance.msg
  BooleanStamped.msg
  FloatStamped.msg
  IntegerStamped.msg
  Sample.msg
  ListStamped.msg
  yolodepth.msg
  pcl_pred.msg
  ttc.msg
)


## Generate added messages and services with any dependencies listed here

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )


catkin_package(
#  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
 LIBRARIES  interactivity_utils
 CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs message_runtime interactive_markers costmap_2d
 DEPENDS  EIGEN3

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_LIBRARY_DIRS}  
  ${PCL_LIBRARY_DIRS}
)
include_directories(${catkin_INCLUDE_DIRS} include/Scheduler)
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)


catkin_install_python(PROGRAMS 

    src/yolo_depth.py 
    src/prediction.py 
    src/kf_predictors.py 
    src/marker_publisher.py
    src/move.py 
    src/monitor.py
    src/nav.py

   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


add_executable(organize src/organize.cpp)
target_link_libraries(organize ${catkin_LIBRARIES} pcl_visualization)
add_dependencies(organize monitoring_generate_messages_cpp)



## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


