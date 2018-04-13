# darknet_ros_py

Get YOLO output into ROS.

# Explanation

Using the default `Multiple Images` code, the config and weights can be loaded only once for predicting the objects in multiple images in a row.

Here, instead of supplying an image on the command line, we use a ROS Python script to pipeline the images retrieved from a **rostopic**.

We adapted a fork of darknet to filter the command line output (we simply commented out the unwanted `printf`'s and `fprintf`'s).

# Prerequisites

# Quick-start
Clone our fork of darknet, make, and download the YOLO weights file:

1. `git clone git@github.com:Cartucho/darknet.git`
2. `cd darknet`
3. `make`
4. `wget https://pjreddie.com/media/files/yolov3.weights`

# Running the code

`cd catkin_ws/`
`source devel/setup.bash`
`roslaunch darknet_ros_py darknet_ros_py.launch`
