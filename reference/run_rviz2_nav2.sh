#!/usr/bin/env bash
set -e
source /opt/ros/foxy/setup.bash
source /home/renity_admin/ros2_foxy_amr_avalue/install/setup.bash

CONFIG_FILE="/home/renity_admin/ros2_foxy_amr_avalue/avalue_robot_nav2/rviz/nav2.rviz"
exec rviz2 -d "$CONFIG_FILE"
