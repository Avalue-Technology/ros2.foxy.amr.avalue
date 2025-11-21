# Introduction
This article primarily introduces how to install Avalue ROS2 Foxy AMR Nodes on the AIB-NW01 Ubuntu 20.04 environment provided by Avalue Technology Inc.

# Prerequisite
## ROS2 Foxy
Because of Avalue ROS2 Foxy AMR Nodes depends on ROS2 Foxy environment, please refer as follows GitHub Repository to complete preparing.


[ros2.foxy.AIB-NW01](https://github.com/Avalue-Technology/ros2.foxy.AIB-NW01)

## Intel® RealSense™ ROS2
Because of Avalue ROS2 Foxy AMR Nodes may depends on Intel® RealSense™ ROS2 Node, please refer as follows GitHub Repository to complete preparing.


[ros2.foxy.camera.intelrealsense](https://github.com/Avalue-Technology/ros2.foxy.camera.intelrealsense)

## SLAMTEC LIDAR ROS2
Because of Avalue ROS2 Foxy AMR Nodes depends on SLAMTEC LIDAR ROS2 Node, please refer as follows GitHub Repository to complete preparing.


[ros2.foxy.lidar.slamtec](https://github.com/Avalue-Technology/ros2.foxy.lidar.slamtec)

## udev rules (/etc/udev/rules.d)
Please refer to the file `avalue_udev.sh` in the `reference` folder to create symbolic link rules for peripheral devices.
E.g. Intel® RealSense™, SLAMTEC LIDAR, IMU...etc.

Because the Avalue ROS2 Foxy AMR Nodes establish connections with peripherals via symbolic links, avoiding issues caused by plugging and unplugging devices or changing locations that could lead to connection failures!

## Kernel Object (/etc/systemd/system/cp210x.service)
Please refer to the file `cp210x.service` in the `reference` folder to create service file for loading Silicon Labs CP210x USB-to-UART Driver (Kernel Module) automatically.

# Install Dependency
```bash
sudo apt update

sudo apt install -y ros-foxy-nav2-bringup ros-foxy-navigation2 ros-foxy-nav2-msgs
sudo apt install -y ros-foxy-usb-cam ros-foxy-cartographer ros-foxy-cartographer-ros ros-foxy-joy ros-foxy-teleop-twist-joy
sudo apt install -y ros-foxy-nav2-map-server
sudo apt install -y ros-foxy-nav2-amcl
sudo apt install -y ros-foxy-tf2-ros
sudo apt install -y ros-foxy-slam-toolbox
sudo apt install -y ros-foxy-filters
sudo apt install -y ros-foxy-image-geometry ros-foxy-image-transport ros-foxy-cv-bridge
sudo apt install -y ros-foxy-image-publisher
sudo apt install -y ros-foxy-camera-info-manager ros-foxy-image-proc ros-foxy-image-view
sudo apt install -y ros-foxy-pcl-conversions ros-foxy-pcl-ros
sudo apt install -y ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui ros-foxy-robot-state-publisher
sudo apt install -y ros-foxy-pointcloud-to-laserscan
sudo apt install -y ros-foxy-imu-tools
sudo apt install -y ros-foxy-robot-localization
sudo apt install -y ros-foxy-rviz2

sudo apt install -y nlohmann-json3-dev
sudo apt install -y libuvc-dev
sudo apt install -y libgoogle-glog-dev
sudo apt install -y libopenni2-dev libusb-1.0-0-dev
sudo apt install -y git cmake build-essential libgflags-dev
sudo apt install -y libpcl-dev libpcap0.8-dev libpng-dev libusb-1.0-0-dev
```

# Build & Install - glog
```bash
cd ~
# Clone - glog Source Code
git clone https://github.com/google/glog.git
git checkout v0.6.0 

# Compile - glog Source Code
cd glog
cmake -S . -B build -DBUILD_SHARED_LIBS=ON -DWITH_GFLAGS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
sudo cmake --install build

# Refresh - Library Cache
sudo ldconfig

# If your output differs, please replace it with the actual folder containing glog-config.cmake
export glog_DIR=/usr/local/lib/cmake/glog
```

# Build & Install - magic_enum
```bash
cd ~
# Clone - magic_enum Source Code
git clone https://github.com/Neargye/magic_enum.git

# Compile - magic_enum Source Code
cd magic_enum
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
sudo cmake --install build

# Refresh - Library Cache
sudo ldconfig

# If your output differs, please replace it with the actual folder
export CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH}"
export CPATH="/usr/local/include:${CPATH}"
```

# Build & Install - Avalue ROS2 Foxy AMR Nodes
```bash
cd ~
mkdir ros2_foxy_amr_avalue
cd ros2_foxy_amr_avalue
# Clone - ros2.foxy.amr.avalue Source Code
git clone https://github.com/Avalue-Technology/ros2.foxy.amr.avalue.git .
# Install Dependency 
rosdep install --from-paths . --ignore-src -r -y
# Clean - build, install, log
rm -rf build/ install/ log/
# Compile - ros2.foxy.amr.avalue Source Code - All ROS2 Nodes
colcon build --symlink-install
```

# Configure ROS2 Foxy, Intel® RealSense™ ROS2, SLAMTEC LIDAR ROS2 Environment
```bash
# Setup ROS2 Foxy Environment
source /opt/ros/foxy/setup.bash
# Setup Intel® RealSense™ ROS2, SLAMTEC LIDAR ROS2 Environment
cd ~/ros2_ws
source install/setup.bash
# Setup Avalue ROS2 AMR Environment
cd ~/ros2_foxy_amr_avalue
source install/setup.bash
```

# Usage
```bash
# Turn On - Intel® RealSense™ ROS2
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true'
```

```bash
# Turn On - IMU ROS2
ros2 launch yesense_std_ros2 yesense_node.launch.py
```

```bash
# Turn On - SLAMTEC LIDAR ROS2 (A2 M12)
ros2 launch sllidar_ros2 view_sllidar_a2m12_launch.py
```

```bash
# Turn On - AMR ROS2
ros2 launch avalue_robot turn_on_avalue_robot.launch.py
```

# Optional Installation - ROS2 WebSocket Bridge Server Node
If you would like to create ROS2 Web Application, you should consider to install this package.
It can help us to subscribe and operate ROS2 Node through WebSocket.

```bash
sudo apt update
# Install ROS2 Packages - Dependency
sudo apt install ros-foxy-launch-xml
# Install ROS2 Packages - ros-foxy-rosbridge-server
sudo apt install ros-foxy-rosbridge-server

# Setup ROS2 Foxy Environment
source /opt/ros/foxy/setup.bash

# Turn On - WebSocket Bridge Server ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

# Reference
## erase_ros_log.sh
Please refer to the file `erase_ros_log.sh` in the `reference` folder to erase ROS2 Log.

## realsense-info.sh
Please refer to the file `realsense-info.sh` in the `reference` folder to determine Intel® RealSense™: /dev/video0~/dev/video5 functionality.
E.g. RGB Camera, Depth Camera...etc.

## Ubuntu 20.04 Desktop Shortcut
- ***CreateMAP.desktop***
Please refer to the file `CreateMAP.desktop` in the `reference` folder to create Ubuntu 20.04 desktop shortcut.
- ***RVIZ2-MAP.desktop***
Please refer to the file `RVIZ2-MAP.desktop` in the `reference` folder to create Ubuntu 20.04 desktop shortcut.
- ***SaveMAP.desktop***
Please refer to the file `SaveMAP.desktop` in the `reference` folder to create Ubuntu 20.04 desktop shortcut.
- ***StartNavigation.desktop***
Please refer to the file `StartNavigation.desktop` in the `reference` folder to create Ubuntu 20.04 desktop shortcut.
- ***RVIZ2-NAVIGATION.desktop***
Please refer to the file `RVIZ2-NAVIGATION.desktop` in the `reference` folder to create Ubuntu 20.04 desktop shortcut.
- ***FollowWaypointsClick.desktop***
Please refer to the file `FollowWaypointsClick.desktop` in the `reference` folder to create Ubuntu 20.04 desktop shortcut.
Because of ROS Foxy RViz2 does not natively support Waypoint panels/buttons, nor does it have an official Waypoint Panel Plugin.
So we have created Python application to help us to send waypoints to the topic /waypoints manually. 

**CAUTION.** After creating the shortcut, please remember to right-click it on your desktop and select "Allow Launching".

## TF Tree (AMCL/SLAM - ROS 2 Navigation Framework)
```
map
└── odom_combined (From EKF: IMU + odom0)
    └── base_footprint     (IMU filter or URDF)
	        └── base_link			
            └── wheels (or Left and right motors joint)

map (AMCL or slam)
└── odom_combined  (IMU dead reckoning)
    └── base_footprint	
         └── base_link
```

## Execution Screenshot - Follow Waypoint
You can refer to our method to create a desktop shortcut for Ubuntu 20.04, and follow the steps below to perform AMR: Waypoint Following Operation.

The desktop shortcut is as follows.
![AIB-NW01.FOLLOW_WAYPOINT.01.png](https://raw.githubusercontent.com/Avalue-Technology/ros2.foxy.amr.avalue/refs/heads/main/MarkdownDocumentImages/AIB-NW01.FOLLOW_WAYPOINT.01.png "AIB-NW01.FOLLOW_WAYPOINT.01.png")

Step 01. Please click shortcut: ***Create Map***.
![AIB-NW01.FOLLOW_WAYPOINT.02.png](https://raw.githubusercontent.com/Avalue-Technology/ros2.foxy.amr.avalue/refs/heads/main/MarkdownDocumentImages/AIB-NW01.FOLLOW_WAYPOINT.02.png "AIB-NW01.FOLLOW_WAYPOINT.02.png")

Step 02. Please click shortcut: ***RVIZ2 - MAP***.
This allows us to operate the **AMR** and create a map using the **ROS2 Visualization 2** graphical interface, paired with the remote controller. 
P.S. The **Step 01.** we opened previously was Gmapping, which will perform the SLAM mapping.
![AIB-NW01.FOLLOW_WAYPOINT.03.png](https://raw.githubusercontent.com/Avalue-Technology/ros2.foxy.amr.avalue/refs/heads/main/MarkdownDocumentImages/AIB-NW01.FOLLOW_WAYPOINT.03.png "AIB-NW01.FOLLOW_WAYPOINT.03.png")

Step 03. Please click shortcut: ***Save MAP***.
Once we have completed the map creation, we can save the map through this way.
![AIB-NW01.FOLLOW_WAYPOINT.04.png](https://raw.githubusercontent.com/Avalue-Technology/ros2.foxy.amr.avalue/refs/heads/main/MarkdownDocumentImages/AIB-NW01.FOLLOW_WAYPOINT.04.png "AIB-NW01.FOLLOW_WAYPOINT.04.png")

Step 04. Please close all windows from **Step 01.** to **Step 03**.
Please click ***Save MAP*** Terminal Window and press Enter to close it.
Please click **RViz2**: File > Quit and press Enter to close RViz2.
Please click ***Create Map*** Terminal Window and press Ctrl+C to stop Gmapping and press Enter to close it.

Step 05. Please click shortcut: ***Start Navigation***.
It will start ROS 2 Navigation Framework for us to run: Following Waypoint.
![AIB-NW01.FOLLOW_WAYPOINT.05.png](https://raw.githubusercontent.com/Avalue-Technology/ros2.foxy.amr.avalue/refs/heads/main/MarkdownDocumentImages/AIB-NW01.FOLLOW_WAYPOINT.05.png "AIB-NW01.FOLLOW_WAYPOINT.05.png")

Step 06. Please click shortcut: ***RVIZ2-NAVIGATION***.
We open the RViz graphical interface again, but this configuration is for use with the ROS 2 Navigation Framework.
![AIB-NW01.FOLLOW_WAYPOINT.06.png](https://raw.githubusercontent.com/Avalue-Technology/ros2.foxy.amr.avalue/refs/heads/main/MarkdownDocumentImages/AIB-NW01.FOLLOW_WAYPOINT.06.png "AIB-NW01.FOLLOW_WAYPOINT.06.png")

Step 07. Please click shortcut: ***Follow Waypoints Click***.
Since ROS2 Foxy does not natively support the RViz Waypoint-related plugin, although the panel can be used, we need to use an additional **Python Application** to record the waypoints and send them to the corresponding ROS2 Node.
![AIB-NW01.FOLLOW_WAYPOINT.07.png](https://raw.githubusercontent.com/Avalue-Technology/ros2.foxy.amr.avalue/refs/heads/main/MarkdownDocumentImages/AIB-NW01.FOLLOW_WAYPOINT.07.png "AIB-NW01.FOLLOW_WAYPOINT.07.png")

![AIB-NW01.FOLLOW_WAYPOINT.08.png](https://raw.githubusercontent.com/Avalue-Technology/ros2.foxy.amr.avalue/refs/heads/main/MarkdownDocumentImages/AIB-NW01.FOLLOW_WAYPOINT.08.png "AIB-NW01.FOLLOW_WAYPOINT.08.png")
