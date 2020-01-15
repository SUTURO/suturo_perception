# suturo_perception
Dependencies:   
suturo_perception_msgs are required. (Found in suturo_ressources)   
https://github.com/evankapi/rs_hsrb_perception (master)   
https://github.com/Suturo1819/robosherlock/tree/rs_v4r (rs_v4r)   

Start pipelines and action server with:   
roslaunch rs_perception hsrb_perception.launch   

Start action clients:   
rosrun actionserver ExtractObjectInfoClient   
rosrun actionserver ExtractObjectInfoInRegionClient   
rosrun actionserver ExtractPlaneInfoClient   
