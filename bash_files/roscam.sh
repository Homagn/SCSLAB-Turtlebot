cd
export ROS_MASTER_URI=http://tegra-ubuntu:11311
export ROS_HOSTNAME=tegra-ubuntu
ssh -X@tegra-ubuntu
export DISPLAY=:0
cd turtlebot
cd devel
sudo bash setup.bash
roslaunch turtlebot_bringup 3dsensor.launch
