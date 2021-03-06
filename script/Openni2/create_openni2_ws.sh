#!/bin/bash
# Create_openni2_ws

echo -e "\e[31mShell Script for Ubuntu 16.04.6 LTS\e[0m"

echo -e "\e[32mCreating Workspace :\e[0m" 
source /opt/ros/kinetic/setup.bash 
mkdir -p ~/openni2_ws/src
cd ~/openni2_ws/
catkin build 
source devel/setup.bash

rosclean

echo -e "\e[32mDownloading rs_turn_table:\e[0m" 
cd ~/openni2_ws/src
git clone https://github.com/Vanessa-rin/rs_turn_table

echo -e "\e[32mDownloading robosherlock :\e[0m"
echo -e "\e[32mRobosherlock from Suturo1819 rs_v4r\e[0m"
cd ~/openni2_ws/src
git clone https://github.com/Suturo1819/robosherlock.git -b rs_v4r --recursive

cd ~/openni2_ws/ 
catkin build 
source devel/setup.bash

echo -e "\e[32mUpdates : \e[0m"
sudo apt-get update 
# sudo rosdep update

echo -e "(╯°Д°)╯ ┳━┳ \e[32mDone\e[0m (╯°Д°)╯ 彡┻━┻"
