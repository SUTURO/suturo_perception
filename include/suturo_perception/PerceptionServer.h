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
#include <suturo_perception_msgs/ExtractDrawerInfoAction.h>

#include <suturo_perception/SuturoProcessManager.h>

#define EOI_NAME        "perception_actionserver"
#define EPI_NAME        "perception_actionserver_plane"
#define EDI_NAME        "perception_actionserver_drawer"
#define ETI_NAME        "perception_actionserver_table"

using namespace suturo_perception_msgs;

class PerceptionServer {
protected:
    ros::NodeHandle nh;
    std::string action_name;
    SuturoProcessManager pm;

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
};

class ExtractPlaneInfoServer : PerceptionServer {
protected:
    actionlib::SimpleActionServer<ExtractPlaneInfoAction> server;
    ExtractPlaneInfoFeedback feedback;
    ExtractPlaneInfoResult result;

public:
    ExtractPlaneInfoServer(std::string name);
    void execute(const ExtractPlaneInfoGoalConstPtr &goal);
};

class ExtractDrawerInfoServer : PerceptionServer {
protected:
    actionlib::SimpleActionServer<ExtractDrawerInfoAction> server;
    ExtractDrawerInfoFeedback feedback;
    ExtractDrawerInfoResult result;

public:
    ExtractDrawerInfoServer(std::string name);
    void execute(const ExtractDrawerInfoGoalConstPtr &goal);
};

class ExtractTableInfoServer : PerceptionServer {
protected:
    actionlib::SimpleActionServer<ExtractPlaneInfoAction> server;
    ExtractPlaneInfoFeedback feedback;
    ExtractPlaneInfoResult result;

public:
    ExtractTableInfoServer(std::string name);
    void execute(const ExtractPlaneInfoGoalConstPtr &goal);
};
#endif
