#include <PerceptionServer.h>

void ExtractPlaneInfoServer::execute(const suturo_perception_msgs::ExtractPlaneInfoGoalConstPtr & goal) {
    std::map<std::string, boost::any> arguments = std::map<std::string, boost::any>();
    arguments["visualize"] = goal->visualize;
    std::vector<std::string> regions = std::vector<std::string>();
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

ExtractPlaneInfoServer::ExtractPlaneInfoServer(std::string name) :
        PerceptionServer(name, "hsrb_planes"),
        server(nh, name, boost::bind(&ExtractPlaneInfoServer::execute, this, _1), false)
{
    server.start();
    ROS_INFO("ExtractPlaneInfoServer started..");
}