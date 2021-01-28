# suturo_perception
#### Dependencies:
You can execute the following command to clone the dependencies of this package:
```
git clone https://github.com/SUTURO/suturo_resources.git &&   
git clone https://github.com/RoboSherlock/robosherlock.git --recursive &&   
git clohe ttps://github.com/Paniago82/rs_resources.git &&
git clone https://github.com/Jastock/rs_addons.git
```

#### Start pipelines and action server:
roslaunch suturo_perception hsrb_perception.launch   

#### Start action clients:  
rosrun suturo_perception ExtractObjectInfoClient   
rosrun suturo_perception ExtractPlaneInfoClient   

You can set \_region:=**_region name_** to limit the perception of objects to a specific region.

#### Notes:
If you set the region to **_robocup_default_** the RegionFilter will be disabled to allow the perception of objects placed on the ground.

#### Result topics:  
perception_actionserver/result   
perception_actionserver_plane/result   
