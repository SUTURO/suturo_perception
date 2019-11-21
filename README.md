# suturo_perception
Start Pipeline with:
rosrun robosherlock run _ae:=hsr_demo _vis:=true

Set _vis to false if no visual feedback is needed.

Pipeline waits for trigger to start. "rosservice call /perception_pipeline/trigger" starts the pipeline and makes it run once. After that it waits for a new trigger. Results are published to "/perception_pipeline/result_advertiser".
