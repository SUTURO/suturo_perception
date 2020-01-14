#include <PerceptionServer.h>

void ExtractObjectInfoServer::execute(const suturo_perception_msgs::ExtractObjectInfoGoalConstPtr & goal) {
    gotData = false;

    ros::Rate r(1);
    bool success = true;

    if (goal->visualize)
    {
        ROS_INFO("Visualization is upcoming");
    }

    if (server.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("%s: Preempted", action_name.c_str());
        server.setPreempted();
        success = false;
    }

    // Starting RoboSherlock-Pipeline by sending Trigger
    ros::ServiceClient triggerClient = nh.serviceClient<std_srvs::Trigger>("/perception_pipeline/trigger");
    std_srvs::Trigger srv;

    if (triggerClient.call(srv))
    {
        ROS_INFO("Pipeline started");
        feedback.feedback = "Pipeline started";
    }
    else
    {
        ROS_ERROR("Perception-Actionserver failed to connect to Robosherlock-Pipeline.");
        feedback.feedback = "Perception-Actionserver failed to connect to Robosherlock-Pipeline.";
    }
    server.publishFeedback(feedback);

    // Listening to RoboSherlock-TFBroadcaster in order to parse those data
    //PerceptionServer thisServer = this;
    ros::Subscriber tfBroadcastListener = nh.subscribe("/perception_pipeline/result_advertiser", 100, &ExtractObjectInfoServer::result_callback, this); // put here the Rostopic that TFBroadcaster publishes in

    while (!gotData) { // todo make this check during RoboSherlock
        if (server.isPreemptRequested() || !ros::ok())
        {
            ROS_ERROR("Call for Perception Pipeline got cancelled!");
            server.setPreempted();
            success = false;
            break;
        }
    }

    if(success)
    {
        ROS_INFO("Perception-Data successfully sent to /perception_actionserver/result");
        server.setSucceeded(result);
    }
}

void ExtractObjectInfoServer::result_callback(const robosherlock_msgs::RSObjectDescriptions::ConstPtr& tfBroadcast) {
    //processRSObjectData(tfBroadcast, result.detectionData);
    gotData = true;
}

ExtractObjectInfoServer::ExtractObjectInfoServer(std::string name) :
PerceptionServer(name, "hsrb_1ms"),
server(nh, name, boost::bind(&ExtractObjectInfoServer::execute, this, _1), false)
{
    server.start();
    ROS_INFO("ExtractObjectInfoServer started..");
}