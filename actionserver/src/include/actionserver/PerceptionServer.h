#ifndef PERCEPTION_SERVER
#define PERCEPTION_SERVER

#include <cstdlib>
#include <string>
#include <vector>
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
#include <suturo_perception_msgs/ExtractPlaneInfoAction.h>
#include <suturo_perception_msgs/ExtractObjectInfoInRegionAction.h>

#include <rs_hsrb_perception/SuturoProcessManager.h>

using namespace suturo_perception_msgs;

class PerceptionServer {
protected:
    ros::NodeHandle nh;
    std::string action_name;
    SuturoProcessManager pm;
    bool gotData;

public:
    PerceptionServer(std::string &name, std::string pipeline);
    ~PerceptionServer() {};
};

class ExtractObjectInfoServer : PerceptionServer {
protected:
    actionlib::SimpleActionServer<ExtractObjectInfoAction> server;
    ExtractObjectInfoFeedback feedback;
    ExtractObjectInfoResult result;

public:
    ExtractObjectInfoServer(std::string name);
    void execute(const ExtractObjectInfoGoalConstPtr &goal);
    void result_callback(const robosherlock_msgs::RSObjectDescriptions::ConstPtr& tfBroadcast);
};

class ExtractPlaneInfoServer : PerceptionServer {
protected:
    actionlib::SimpleActionServer<ExtractObjectInfoAction> server;
    ExtractPlaneInfoFeedback feedback;
    ExtractPlaneInfoResult result;

public:
    ExtractPlaneInfoServer(std::string name);
    void execute(const ExtractObjectInfoGoalConstPtr &goal);
    void result_callback(const robosherlock_msgs::RSObjectDescriptions::ConstPtr& tfBroadcast);
};

class ExtractObjectInfoInRegionServer : PerceptionServer {
protected:
    actionlib::SimpleActionServer<ExtractObjectInfoInRegionAction> server;
    ExtractObjectInfoInRegionFeedback feedback;
    ExtractObjectInfoInRegionResult result;

public:
    ExtractObjectInfoInRegionServer(std::string name);
    void execute(const ExtractObjectInfoInRegionGoalConstPtr &goal);
    void result_callback(const robosherlock_msgs::RSObjectDescriptions::ConstPtr& tfBroadcast);
};
#endif