#!/bin/bash

echo "Khởi động Map Visualizer cho Robot..."
echo "Đảm bảo rằng các node ROS2 sau đã chạy:"
echo "- ros2 launch slam_toolbox online_async_launch.py"
echo "- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo "- ros2 run autonomous_exploration control"
echo ""

# Kiểm tra Python dependencies
echo "Kiểm tra dependencies..."
python3 -c "import matplotlib, numpy, PyQt5, pyqtgraph, yaml" 2>/dev/null

if [ $? -ne 0 ]; then
    echo "Một số dependencies bị thiếu. Cài đặt bằng lệnh:"
    echo "pip3 install -r requirements.txt"
    exit 1
fi

echo "Đang khởi động GUI Visualizer..."

# Chọn visualizer để chạy
echo "Chọn visualizer:"
echo "1. Matplotlib Visualizer (đơn giản)"
echo "2. PyQt5 Visualizer (giao diện đẹp nhất - khuyến nghị)"
echo "3. OpenCV Visualizer (nhẹ nhất, dễ cài đặt)"
read -p "Nhập lựa chọn (1, 2, hoặc 3): " choice

case $choice in
    1)
        echo "Khởi động Matplotlib Visualizer..."
        python3 autonomous_exploration/map_visualizer.py
        ;;
    2)
        echo "Khởi động PyQt5 Visualizer..."
        python3 autonomous_exploration/qt_visualizer.py
        ;;
    3)
        echo "Khởi động OpenCV Visualizer..."
        python3 autonomous_exploration/opencv_visualizer.py
        ;;
    *)
        echo "Lựa chọn không hợp lệ. Chạy mặc định (OpenCV)..."
        python3 autonomous_exploration/opencv_visualizer.py
        ;;
esac 