# suturo_perception
__Dependencies:__   
git clone https://github.com/SUTURO/suturo_resources.git
git clone https://github.com/evankapi/rs_hsrb_perception.git -b suturo20   
git clone https://github.com/RoboSherlock/robosherlock.git --recursive   

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
