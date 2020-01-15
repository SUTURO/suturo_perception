# suturo_perception
__Dependencies:__   
suturo_perception_msgs are required. (Found in suturo_ressources)   
https://github.com/evankapi/rs_hsrb_perception (master)   
https://github.com/Suturo1819/robosherlock/tree/rs_v4r (rs_v4r)   

__Start pipelines and action server with:__   
roslaunch rs_perception hsrb_perception.launch   

__Start action clients:__   
rosrun actionserver ExtractObjectInfoClient   
rosrun actionserver ExtractObjectInfoInRegionClient   
rosrun actionserver ExtractPlaneInfoClient   

__Relevant topics:__
perception_actionserver/result   
perception_actionserver_region/result   
perception_actionserver_plane/result   
