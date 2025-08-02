





# ROS 2 Turtlebot 3 Map Explorer
## Description
In this repo we use Turtlebot 3 along with ROS 2 and Gazebo to explore an unknown csv environment, navigate through it and create a map. 

The map is created using SLAM with the package [Google Cartographer](https://github.com/cartographer-project/cartographer) and navigation is achieved with [Nav2](https://github.com/ros-planning/navigation2) package. We have developed two exploring algorithyms:

>**Wanderer Exploration** explores the map doing random turns when it detects an obstacle. It's a convenient way to explore small maps but time consuming for bigger ones.
  
>**Discoverer Exploration** prioritizes specific unknown hotspots of the map convoluting the occupancy grid. It's a better way to explore bigger maps in exchange of a higher computational cost.

### [Youtube Video](https://youtu.be/UNiCngwE_Zo)
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/UNiCngwE_Zo/maxresdefault.jpg)](https://youtu.be/UNiCngwE_Zo)

## Installation (tested on Ubuntu 22.04 - ROS 2 Humble)

[Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html)

Don't forget to install colcon:
```
sudo apt install python3-colcon-common-extensions
```
Install Gazebo:
```
sudo apt install gazebo
```
Install Python libraries:
```
sudo apt install python3-pip
pip3 install pandas
```
Create a ROS2 workspace:
```
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
```
Clone the repository:
```
git clone https://github.com/DaniGarciaLopez/ros2_explorer.git
```
Compile packages and get dependencies:
```
cd ~/turtlebot3_ws/src
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

cd ~/turtlebot3_ws/
colcon build
```
Include the following lines in ~/.bashrc:
```
source /opt/ros/humble/local_setup.bash
source ~/turtlebot3_ws/install/local_setup.bash

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=~/turtlebot3_ws/src/ros2_explorer/explorer_gazebo/models
```
## How to run
Execute the launch file and pass the map name (Opens Gazebo simulation, Rviz, Cartographer, Nav2 and exploration servers):
```
ros2 launch explorer_bringup explorer.launch.py map_name:=map10
```
Execute manager node and select the desired exploring algorithm:
```
ros2 run explorer_bringup manager
```
## Add your own CSV Map
Add your own csv maps in this folder:
```
cd ~/turtlebot3_ws/src/ros2_explorer/explorer_gazebo/maps/
```
Run Python script:
```
cd ~/turtlebot3_ws/src/ros2_explorer/explorer_gazebo/
python3 gazebo-map-from-csv.py
```
Maps will be converted to Gazebo format in `/explorer_gazebo/models` folder. Create a new .world.xml file in `/explorer_gazebo/worlds` and modify the name of the map you want to use:
```
<include>
  <uri>model://map1</uri>
</include>
```
## Package structure
![image](https://github.com/DaniGarciaLopez/ros2_explorer/blob/main/explorer_bringup/data/explorer_graph.png)
![image](https://github.com/DaniGarciaLopez/ros2_explorer/blob/main/explorer_bringup/data/rosgraph.png)


huy@huy:~/turtlebot3_ws/src$ ros2 run explorer_bringup manager
[INFO] [1754155199.675229341] [manager]: Waiting for service server...
[INFO] [1754155199.675907575] [manager]: Sending discoverer service request...
[INFO] [1754155204.611613256] [manager]: Duration: 5 s - Distance: 0.24 m 
[INFO] [1754155209.614020373] [manager]: Duration: 10 s - Distance: 0.30 m 
[INFO] [1754155214.631913769] [manager]: Duration: 15 s - Distance: 0.30 m 
[INFO] [1754155219.617123700] [manager]: Duration: 20 s - Distance: 0.40 m 
[INFO] [1754155224.611876258] [manager]: Duration: 25 s - Distance: 0.57 m 
[INFO] [1754155229.618393855] [manager]: Duration: 30 s - Distance: 0.92 m 
[INFO] [1754155234.612942744] [manager]: Duration: 35 s - Distance: 1.50 m 
[INFO] [1754155234.845009352] [manager]: MAP SUCCESSFULLY EXPLORED
[INFO] [1754155234.848771102] [manager]: Service response: Exploration completed in 232.7s with 1 iterations
[INFO] [1754155234.849298190] [manager]: Duration: 35 s - Distance: 1.54 m 
[INFO] [1754155234.849661834] [navigation_client]: Waiting for action server...
[INFO] [1754155234.851414902] [navigation_client]: Returning to base...

