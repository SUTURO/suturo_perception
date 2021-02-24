#include <suturo_perception/PerceptionServer.h>

void ExtractPlaneInfoServer::execute(const suturo_perception_msgs::ExtractPlaneInfoGoalConstPtr & goal) {
    std::map<std::string, boost::any> arguments = std::map<std::string, boost::any>();
    arguments["visualize"] = goal->visualize;
    std::vector<std::string> regions = std::vector<std::string>();
    std::vector<ObjectDetectionData> data;
    pm.run(arguments, data);

    if(!data.empty()) {
        feedback.feedback = "Door detection was successful.";

        result.detectionData.height = data[0].height;
        result.detectionData.width = data[0].width;
        result.detectionData.depth = data[0].depth;
        result.detectionData.confidence_class = data[0].confidence_class;
        result.detectionData.name = result.detectionData.obj_class = data[0].name;
        result.detectionData.pose = data[0].pose;
        result.detectionData.region = data[0].region;

        server.publishFeedback(feedback);
        server.setSucceeded(result);
    } else {
        feedback.feedback = "No door detection data was perceived. Check standard output for further information.";
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