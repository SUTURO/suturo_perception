#!/bin/bash
# Openni2 Install

echo -e "\e[31mShell Script for Ubuntu 16.04.6 LTS\e[0m"

echo -e "\e[32mInstalling Openni2_camera :\e[0m"
sudo apt-get install ros-kinetic-openni2-camera

echo -e "\e[32mInstalling Openni2_launch :\e[0m"
sudo apt-get install ros-kinetic-openni2-launch

echo -e "\e[32mInstalling GCC 4.x :\e[0m"
sudo apt-get install g++ 

echo -e "\e[32mInstalling Python 2.6+/3.x :\e[0m"
sudo apt-get install python

echo -e "\e[32mInstalling LibUSB 1.0.x :\e[0m"
sudo apt-get install libusb-1.0-0-dev

echo -e "\e[32mInstalling FreeGLUT3 :\e[0m"
sudo apt-get install freeglut3-dev

echo -e "\e[32mInstalling JDK 8.0 :\e[0m"
sudo apt-get install openjdk-8-jdk

echo -e "\e[32mInstalling Doxygen :\e[0m"
sudo apt-get install doxygen 

echo -e "\e[32mInstalling GraphViz :\e[0m"
sudo apt-get install graphviz 

echo -e "\e[32mChanges on Openni2_launch :\e[0m" 
echo -e "\e[32mDirectory :\e[0m $(rospack find openni2_launch)/launch/openni2.launch"
sudo sed -i -e 's/<arg name="depth_registration" default="false"/<arg name="depth_registration" default="true"/' $(rospack find openni2_launch)/launch/openni2.launch

echo -e "\e[32mUpdate :\e[0m"
sudo apt-get update
echo -e "(╯°Д°)╯ ┳━┳ \e[32mDone\e[0m (╯°Д°)╯ 彡┻━┻"



