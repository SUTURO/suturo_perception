# suturo_perception
__Dependencies:__   
suturo_perception_msgs are required. (Found in suturo_ressources)   
https://github.com/evankapi/rs_hsrb_perception (suturo20)   
https://github.com/Suturo1819/robosherlock/tree/rs_v4r (rs_v4r)   

__Start pipelines and action server with:__   
roscd rs_perception   
roslaunch rs_perception hsrb_perception.launch   

__Start action clients:__   
rosrun actionserver ExtractObjectInfoClient   
rosrun actionserver ExtractPlaneInfoClient   

You can set _region:=%region% to limit the perception of objects to a specific region.

__Result topics:__   
perception_actionserver/result   
perception_actionserver_plane/result   
