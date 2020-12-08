#include <suturo_perception/PerceptionServer.h>

void ExtractObjectInfoServer::execute(const suturo_perception_msgs::ExtractObjectInfoGoalConstPtr & goal) {
    std::map<std::string, boost::any> arguments = std::map<std::string, boost::any>();
    arguments["visualize"] = goal->visualize;

    if(!goal->regions.empty()) {
        arguments["regions"] = goal->regions;
    }
    
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

ExtractObjectInfoServer::ExtractObjectInfoServer(std::string name) :
PerceptionServer(name, "hsrb_1ms"),
server(nh, name, boost::bind(&ExtractObjectInfoServer::execute, this, _1), false)
{
    server.start();
    ROS_INFO("ExtractObjectInfoServer started..");
}
