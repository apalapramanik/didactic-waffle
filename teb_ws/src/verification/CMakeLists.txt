cmake_minimum_required(VERSION 3.0.2)
project(verification)


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
  sensor_msgs
  message_generation
  geometry_msgs
)
find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS $(EIGEN3_INCLUDE_DIRS))
endif()

catkin_python_setup()

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

 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs

 )


catkin_package(
#INCLUDE_DIRS include
#LIBRARIES verification
CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge sensor_msgs message_runtime interactive_markers
#DEPENDS system_lib
DEPENDS  EIGEN3
)

include_directories(
# include
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
src/robot_reachability.py
src/human_reachability.py
src/marker_pub.py

src/collision_tb3_0.py
src/collision_tb3_1.py
src/collision_tb3_2.py

src/clustering_tb3_0.py
src/clustering_tb3_1.py
src/clustering_tb3_2.py

src/multi_yolo_depth.py

src/goal_tb3_0.py
src/goal_tb3_1.py
src/goal_tb3_2.py

src/case1.py
src/case2.py
src/case3.py
src/case3b.py
src/case4.py

src/monitor.py
src/prediction.py



DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(multi_process_pcl_tb3_0 src/multi_process_pcl_tb3_0.cpp)
target_link_libraries(multi_process_pcl_tb3_0 ${catkin_LIBRARIES} pcl_visualization)
add_dependencies(multi_process_pcl_tb3_0 verification_generate_messages_cpp)

add_executable(multi_process_pcl_tb3_1 src/multi_process_pcl_tb3_1.cpp)
target_link_libraries(multi_process_pcl_tb3_1 ${catkin_LIBRARIES} pcl_visualization)
add_dependencies(multi_process_pcl_tb3_1 verification_generate_messages_cpp)

add_executable(multi_process_pcl_tb3_2 src/multi_process_pcl_tb3_2.cpp)
target_link_libraries(multi_process_pcl_tb3_2 ${catkin_LIBRARIES} pcl_visualization)
add_dependencies(multi_process_pcl_tb3_2 verification_generate_messages_cpp)


