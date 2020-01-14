#include <PerceptionServer.h>

void ExtractPlaneInfoServer::execute(const suturo_perception_msgs::ExtractObjectInfoGoalConstPtr & goal) {

}

ExtractPlaneInfoServer::ExtractPlaneInfoServer(std::string name) :
        PerceptionServer(name, "hsrb_1ms"),
        server(nh, name, boost::bind(&ExtractPlaneInfoServer::execute, this, _1), false)
{

}