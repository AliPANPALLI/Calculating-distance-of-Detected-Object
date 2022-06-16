# Calculating-distance-of-Detected-Object
 Measuring the distance of objects recognized by the ros-based YOLO model using the zed 2i camera.
## Prerequisites
Ubuntu 20.04
ZED SDK ≥ 3.7 and its dependency CUDA
ROS Noetic

or

Ubuntu 18.04
ZED SDK ≥ 3.7 and its dependency CUDA
ROS Melodic

## Build the repository
The zed_ros_wrapper is a catkin package. It depends on the following ROS packages:

nav_msgs
tf2_geometry_msgs
message_runtime
catkin
roscpp
stereo_msgs
rosconsole
robot_state_publisher
urdf
sensor_msgs
image_transport
roslint
diagnostic_updater
dynamic_reconfigure
tf2_ros
message_generation
nodelet
Open a terminal, clone the repository, update the dependencies and build the packages:
```
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
$ cd ../
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source ./devel/setup.bash
```
## Update the local repository
To update the repository to the latest release you must use the following command to retrieve the latest commits of zed-ros-wrapper and of all the submodules:
```
$ git checkout master # if you are not on the main branch  
$ git pull --recurse-submodules # update recursively all the submodules
```
Remember to always clean the cache of your catkin workspace before compiling with the catkin_make command to be sure that everything will work as expected:
```
$ roscd
$ cd ..
$ rm -rf build
$ rm -rf devel
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```
## Run the ZED wrapper
To launch the ZED node use

ZED camera:
```
$ roslaunch zed_wrapper zed.launch
```
ZED Mini camera:
```
$ roslaunch zed_wrapper zedm.launch
```
ZED2 camera:
```
$ roslaunch zed_wrapper zed2.launch
```
ZED2i camera:
```
$ roslaunch zed_wrapper zed2i.launch    
```
To select the ZED from its serial number:
```
 $ roslaunch zed_wrapper zed.launch serial_number:=1010 #replace 1010 with the actual SN
 ```
 ## EXECUTE
 After opening it in terminal to our working directory. ROS is started by typing ```roscore``` in our terminal.
 We open a second terminal and start the ZED camera on the ROS network by typing ```roslaunch zed_wrapper zed2i.launch.```
 The Object_detection.py file must be started with the ```rosrun zed_wrapper Object_detection.py``` command.
 Depth.py file After running Object_detection.py file, ```rosrun zed_wrapper Depth.py``` should be run.
 Click to access [Yolo models](https://s2.dosya.tc/server17/sg2281/yolov3.zip.html).
