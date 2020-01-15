#include <PerceptionServer.h>

void ExtractObjectInfoInRegionServer::execute(const suturo_perception_msgs::ExtractObjectInfoInRegionGoalConstPtr & goal) {
    std::map<std::string, boost::any> arguments = std::map<std::string, boost::any>();
    arguments["visualize"] = goal->visualize;
    std::vector<std::string> regions = std::vector<std::string>();
    arguments["regions"] = goal->regions;
    result.detectionData.clear();
    pm.run(arguments, result.detectionData);

    if(!result.detectionData.empty()) {
        feedback.feedback = "Object Feature detection was successful.";
        server.publishFeedback(feedback);
        server.setSucceeded(result);
    } else {
        feedback.feedback = "No object detection data was perceived. Check standard output for further information.";
        server.publishFeedback(feedback);
        server.setAborted();
    }
}

ExtractObjectInfoInRegionServer::ExtractObjectInfoInRegionServer(std::string name) :
        PerceptionServer(name, "hsrb_1ms"),
        server(nh, name, boost::bind(&ExtractObjectInfoInRegionServer::execute, this, _1), false)
{
    server.start();
    ROS_INFO("ExtractObjectInfoInRegionServer started..");
}