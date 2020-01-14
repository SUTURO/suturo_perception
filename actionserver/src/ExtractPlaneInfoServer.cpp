#include <PerceptionServer.h>

void ExtractPlaneInfoServer::execute(const suturo_perception_msgs::ExtractObjectInfoGoalConstPtr & goal) {

}

void ExtractPlaneInfoServer::result_callback(const robosherlock_msgs::RSObjectDescriptions::ConstPtr & tfBroadcast) {

}

ExtractPlaneInfoServer::ExtractPlaneInfoServer(std::string name) :
        PerceptionServer(name, "hsrb_1ms"),
        server(nh, name, boost::bind(&ExtractPlaneInfoServer::execute, this, _1), false)
{

}