# suturo_perception
#### Dependencies:
You can execute the following command to clone the dependencies of this package:
```
git clone https://github.com/SUTURO/suturo_resources.git &&   
git clone https://github.com/evankapi/rs_hsrb_perception.git -b suturo20 &&   
git clone https://github.com/RoboSherlock/robosherlock.git --recursive &&   
git clone https://github.com/RoboSherlock/rs_resources.git &&
git clone https://github.com/Jastock/rs_addons.git
```

#### Start pipelines and action server:
roscd rs_perception   
roslaunch rs_perception hsrb_perception.launch   

#### Start action clients:  
rosrun actionserver ExtractObjectInfoClient   
rosrun actionserver ExtractPlaneInfoClient   

You can set \_region:=**_region name_** to limit the perception of objects to a specific region.

#### Notes:
If you set the region to **_robocup_default_** the RegionFilter will be disabled to allow the perception of objects placed on the ground.

#### Result topics:  
perception_actionserver/result   
perception_actionserver_plane/result   
