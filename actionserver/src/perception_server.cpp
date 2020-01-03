#include <cstdlib>
#include <string>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <robosherlock_msgs/RSObjectDescriptions.h>

#include <suturo_perception_msgs/ExtractObjectInfoAction.h>
#include <suturo_perception_msgs/ObjectDetectionData.h>


class PerceptionServer
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<suturo_perception_msgs::ExtractObjectInfoAction> as_;
    std::string action_name_;
    suturo_perception_msgs::ExtractObjectInfoFeedback feedback_;
    suturo_perception_msgs::ExtractObjectInfoResult result_;
    bool gotData;

public:

    PerceptionServer(std::string name) :
            as_(nh_, name, boost::bind(&PerceptionServer::executeCB, this, _1), false),
            action_name_(name)
    {
        as_.start();
    }

    ~PerceptionServer(void)
    {
    }

    void executeCB(const suturo_perception_msgs::ExtractObjectInfoGoalConstPtr &goal)
    {
        gotData = false;

        ros::Rate r(1);
        bool success = true;

        if (goal->visualize)
        {
            ROS_INFO("Visualization is upcoming");
        }

        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
            success = false;
        }

        // Starting RoboSherlock-Pipeline by sending Trigger
        ros::ServiceClient triggerClient = nh_.serviceClient<std_srvs::Trigger>("/perception_pipeline/trigger");
        std_srvs::Trigger srv;

        if (triggerClient.call(srv))
        {
            ROS_INFO("Pipeline started");
            feedback_.feedback = "Pipeline started";
        }
        else
        {
            ROS_ERROR("Perception-Actionserver failed to connect to Robosherlock-Pipeline.");
            feedback_.feedback = "Perception-Actionserver failed to connect to Robosherlock-Pipeline.";
        }
        as_.publishFeedback(feedback_);

        // Listening to RoboSherlock-TFBroadcaster in order to parse those data
        //PerceptionServer thisServer = this;
        ros::Subscriber tfBroadcastListener = nh_.subscribe("/perception_pipeline/result_advertiser", 100, &PerceptionServer::processRSData, this); // put here the Rostopic that TFBroadcaster publishes in

        while (!gotData) { // todo make this check during RoboSherlock
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_ERROR("Call for Perception Pipeline got cancelled!");
                as_.setPreempted();
                success = false;
                break;
            }
        }

        if(success)
        {
            ROS_INFO("Perception-Data successfully sent to /perception_actionserver/result");
            as_.setSucceeded(result_);
        }
    }

    void processRSData(const robosherlock_msgs::RSObjectDescriptions::ConstPtr& tfBroadcast) {
        //ROS_INFO("I have got some info");
        //ROS_INFO("I heard: [%s]", tfBroadcast->obj_descriptions[0].c_str());
        //ROS_INFO("Second description: %s", tfBroadcast->obj_descriptions[1].c_str());

        // parsing JSON
        rapidjson::Document json;
        json.Parse(tfBroadcast->obj_descriptions[0].c_str()); // todo: this needs to be done with every element of obj_descriptions-Array

        suturo_perception_msgs::ObjectDetectionData &resultData = result_.detectionData; // creating reference to shorten statements

        // SemanticColor Annotator
        rapidjson::Value& semanticColor = json["rs.annotation.SemanticColor"][0]["color"];
        std::string colorString = semanticColor.GetString();
        float& colorR = resultData.color.r;
        float& colorG = resultData.color.g;
        float& colorB = resultData.color.b;
        float& colorA = resultData.color.a;

        if (colorString.compare("yellow") == 0) {
            colorR = 255;
            colorG = 255;
            colorB = 50;
        } else {
            colorA = 0.5;
        }

        // Geometry
        rapidjson::Value& geometry = json["rs.annotation.Geometry"][0]["boundingBox"]["rs.pcl.BoundingBox3D"];
        resultData.width = geometry["width"].GetDouble();
        resultData.height = geometry["height"].GetDouble();
        resultData.depth = geometry["depth"].GetDouble();


        // Dummy data /todo fill out with real data
        resultData.name = "unknown name";
        resultData.shape = 0;
        resultData.confidence = 1;
        //stampedPose
        //stampedTransformation

        gotData = true;
    }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perception_actionserver");

    PerceptionServer server("perception_actionserver");
    ros::spin();

    return 0;
}