### OpenNI 2 
OpenNI 2 is the driver for the Xtion Pro live. It can be installt using the shell script "Openni2_Install" or manually with the following commands

### Download the main software : 
```
Openni2_camera : sudo apt-get install ros-kinetic-openni2-camera

Openni2_launch : sudo apt-get install ros-kinetic-openni2-launch
```
### Download Requirements : 
```
GCC 4.x : sudo apt-get install g++ 

Python 2.6+/3.x : sudo apt-get install python 

LibUSB 1.0.x : sudo apt-get install libusb-1.0-0-dev

FreeGLUT3 : sudo apt-get install freeglut3-dev

JDK 8.0 :  sudo apt-get install openjdk-8-jdk

Doxygen : sudo apt-get install doxygen 

GraphViz : sudo apt-get install graphviz 
```

### Optional download Calibration file (Not Required) : 
```
Github : https://github.com/introlab/rtabmap_ros/blob/master/launch/calibration/rgb_PS1080_PrimeSense.yaml
```
Create a new folder in .ros name camera_info and put in the calibration file. 

### Necessary changes on the package : 

Get the package location with : rospack find openni2_lauch 

Go to the location and open the launch folder. 

Open openni2.launch with any editor make sure to use sudo. 

Search for name="depth_registration" default="false" and change dafault to true. 



