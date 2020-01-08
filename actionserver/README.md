# Actionserver

At the moment, the Actionserver requires RoboSherlock to be started seperately with Trigger-Annotator.
When a Client sends the `ExtractObjectInfoAction` (which is difined in `suturo_perception_msgs`), the Actionserver
calls the Trigger-Service, listens to the `result_advertiser` and sends the neccessary data defined in `suturo_perception_msgs/ObjectDetectionData.msg`
to /perception_actionserver/result.

To start the Actionserver, run 
```
rosrun actionserver perception_server
```

To test the Results, you have to also run RoboSherlock and `rosrun actionserver perception_client`.
