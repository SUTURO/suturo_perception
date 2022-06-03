#include <suturo_perception/PerceptionServer.h>

void ExtractTableInfoServer::execute(const suturo_perception_msgs::ExtractPlaneInfoGoalConstPtr & goal) {
    std::map<std::string, boost::any> arguments = std::map<std::string, boost::any>();
    arguments["visualize"] = goal->visualize;

    if(!goal->regions.empty()) {
        arguments["regions"] = goal->regions;
    }

    std::vector<std::string> regions = std::vector<std::string>();
    std::vector<ObjectDetectionData> data;

    //result.detectionData.clear();
    pm.runTableDetection(arguments, data);

    if(!data.empty()) {
        feedback.feedback = "Table detection was successful.";

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
        feedback.feedback = "No table data was perceived. Check standard output for further information.";
        server.publishFeedback(feedback);
        server.setAborted();
    }
}

ExtractTableInfoServer::ExtractTableInfoServer(std::string name) :
        PerceptionServer(name, "hsrb_table"),
        server(nh, name, boost::bind(&ExtractTableInfoServer::execute, this, _1), false)
{
    server.start();
    ROS_INFO("ExtractTableInfoServer started..");
}
