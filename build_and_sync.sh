#!/bin/bash

# Bước 1: Build ROS 2 workspace
echo "Building ROS 2 workspace..."
colcon build --merge-install  --cmake-clean-cache --cmake-force-configure   --cmake-args -DRMW_IMPLEMENTATION=rmw_cyclonedds_cpp -DCMAKE_TOOLCHAIN_FILE=/home/huy/nav2_ws/Vaccum-Robot-LDS02RR/toolchain_aarch64.cmake

# Bước 2: Chạy script fix setup bash
echo "Running fix setup bash..."
./fix_setup_bash.sh

# Bước 3: Đồng bộ hóa thư mục install tới Pi
echo "Syncing install folder to Pi..."
sshpass -p 'huy' rsync -avz --progress install/ huy@192.168.2.15:/home/huy/ros2_ws/install

echo "Process completed successfully!"
