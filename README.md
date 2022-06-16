# Calculating-distance-of-Detected-Object
 Measuring the distance of objects recognized by the ros-based YOLO model using the zed 2i camera.
# Prerequisites
Ubuntu 20.04
ZED SDK ≥ 3.7 and its dependency CUDA
ROS Noetic

or

Ubuntu 18.04
ZED SDK ≥ 3.7 and its dependency CUDA
ROS Melodic

# Build the repository
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
'''
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
$ cd ../
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source ./devel/setup.bash
'''
