#!/usr/bin/env bash
set -e
source /opt/ros/foxy/setup.bash
source /home/renity_admin/ros2_foxy_amr_avalue/install/setup.bash

CONFIG_FILE="/home/renity_admin/ros2_foxy_amr_avalue/avalue_ros2_foxy.rviz"
exec rviz2 -d "$CONFIG_FILE"