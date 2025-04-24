# orbbec-femto-ros2
sudo apt update && sudo apt install -y \
    ros-jazzy-depth-image-proc \
    ros-jazzy-tf2-ros \
    libgflags-dev \
    nlohmann-json3-dev
    
Structure
Home/ros2_ws/src
Put zip in there
    
cd ~/ros2_ws  

# Build dependencies in order  
colcon build --symlink-install --packages-select orbbec_camera_msgs  
colcon build --symlink-install --packages-select orbbec_camera  
colcon build --symlink-install --packages-select my_robot_launch  
colcon build --symlink-install --packages-select aruco_detector

# Source the workspace  
source install/setup.bash  
source ~/ros2_ws/install/setup.bash  
ros2 launch orbbec_camera femto_bolt.launch.py  

new terminal
source ~/ros2_ws/install/setup.bash  
ros2 launch my_robot_launch femto_cloud.launch.py  

new terminal
source /opt/ros/jazzy/setup.bash  
rviz2 -d ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/rviz/pointcloud.rviz  
