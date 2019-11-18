# suturo_perception
Start Pipeline with:
rosrun robosherlock run _ae:=hsr_demo _vis:=true

Set _vis to false if no visual feedback is needed.

Pipeline waits for trigger to start. "rosservice call /Robosherlock_your-username/trigger" starts the pipeline and makes it run once. After that it waits for a new trigger.
