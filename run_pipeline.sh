#!/bin/bash

# Define the sourcing command
SOURCE_ALL="source ~/catkin_ws/devel/setup.bash && source ~/dv_ros/devel/setup.bash && source ~/metavision_ros_driver_ws/devel/setup.bash"

# Launch the driver node
gnome-terminal -- bash -c "$SOURCE_ALL && roslaunch metavision_ros_driver driver_node.launch; exec bash"
sleep 5  # Wait for the camera driver to fully initialize

# Launch the accumulator
gnome-terminal -- bash -c "$SOURCE_ALL && rosrun dv_ros_accumulation accumulator; exec bash"
sleep 3  # Wait for topics to be published

# Launch the image viewer
gnome-terminal -- bash -c "$SOURCE_ALL && rosrun rqt_image_view rqt_image_view; exec bash"
sleep 2

# Launch the reconfigure GUI
gnome-terminal -- bash -c "$SOURCE_ALL && rosrun rqt_reconfigure rqt_reconfigure; exec bash"

