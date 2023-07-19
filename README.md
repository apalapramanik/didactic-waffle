# didactic-waffle
```
mkdir -p ~/apala_ws/src
catkin_make
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
cd src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
catkin_make
```
Add world file to tb3_simunlations → tb3_gazebo → worlds

Add launch file to tb3_simunlations → tb3_gazebo → launch

Modify launch file (position of tb3, world file name)

Launch gazebo environment: ``` roslaunch turtlebot3_gazebo scene1.launch ```

Launch navigation:``` roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/apramani/didactic-waffle/apala_ws/src/turtlebot3/turtlebot3_navigation/maps/map.yaml```

Launch yolov5 : ``` rosrun tb_apala yolo_depth.py ```

Launch cloud processing: ``` rosrun tb_apala organize ```


