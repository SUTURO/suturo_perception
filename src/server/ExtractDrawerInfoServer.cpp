#include <suturo_perception/PerceptionServer.h>

void ExtractDrawerInfoServer::execute(const suturo_perception_msgs::ExtractDrawerInfoGoalConstPtr & goal) {
    std::map<std::string, boost::any> arguments = std::map<std::string, boost::any>();
    arguments["visualize"] = goal->visualize;

//    if(!goal->goal.empty()) {
//        arguments["goal"] = goal->goal;
//    }

    std::vector<std::string> regions = std::vector<std::string>();
    std::vector<DrawerDetectionData> data;

    //result.detectionData.clear();
    pm.runDrawerDetection(arguments, data);

    if(!data.empty()) {
        feedback.feedback = "Drawer detection was successful.";

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
        feedback.feedback = "No drawer data was perceived. Check standard output for further information.";
        server.publishFeedback(feedback);
        server.setAborted();
    }
}

ExtractDrawerInfoServer::ExtractDrawerInfoServer(std::string name) :
        PerceptionServer(name, "hsrb_drawers"),
        server(nh, name, boost::bind(&ExtractDrawerInfoServer::execute, this, _1), false)
{
    server.start();
    ROS_INFO("ExtractDrawerInfoServer started..");
}
