# sudo chown renity_admin:renity_admin /home/renity_admin/ros2_ws/ros2_foxy_env.sh
# chmod 644 /home/renity_admin/ros2_ws/ros2_foxy_env.sh
# chmod +x /home/renity_admin/ros2_ws/ros2_foxy_env.sh

#!/usr/bin/env bash
# Avoid Python/ROS Localization Errors
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

# ROS Configuration (can be changed to your domain ID)
export ROS_DOMAIN_ID=0

# Display Environment: If GUI/RViz is not available, it can be removed
export DISPLAY=:0

# Basic PATH (sometimes non-interactive shells lack PATH)
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

# Setup ROS2 Foxy Environment
source /opt/ros/foxy/setup.bash
