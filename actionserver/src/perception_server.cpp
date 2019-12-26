#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_perception_msgs/ExtractObjectInfoAction.h>

#include <cstdlib>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

class PerceptionServer
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<suturo_perception_msgs::ExtractObjectInfoAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to publish feedback/result
    suturo_perception_msgs::ExtractObjectInfoFeedback feedback_;
    suturo_perception_msgs::ExtractObjectInfoResult result_;

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
        // helper variables
        ros::Rate r(1);
        bool success = true;

        if (goal->visualize)
        {
            ROS_INFO("Visualization is upcoming");
        }

        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
            success = false;
        }

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

        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep();


        if(success)
        {
            result_.detectionData.name = "Schrank";
            result_.detectionData.obj_class = "Pringles Dose";
            result_.detectionData.confidence = 0.1;
            result_.detectionData.shape = 2;
            result_.detectionData.width = 100.0;
            result_.detectionData.height = 200.0;
            result_.detectionData.depth = 50.0;
            result_.detectionData.region = "shelf";
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "perception_data");

    PerceptionServer server("perception_data");
    ros::spin();

    return 0;
}