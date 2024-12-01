#!/bin/bash
source /opt/ros/humble/setup.bash   
source ~/.bashrc && source /root/ROS_connector/install/setup.bash
cd /root/ROS_connector&& source /root/ROS_connector/install/setup.bash
ros2 run ros_connector_manager ros_connector_manager