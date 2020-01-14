#include <PerceptionServer.h>

void ExtractObjectInfoInRegionServer::execute(const suturo_perception_msgs::ExtractObjectInfoInRegionGoalConstPtr & goal) {

}

void ExtractObjectInfoInRegionServer::result_callback(const robosherlock_msgs::RSObjectDescriptions::ConstPtr &tfBroadcast) {

}

ExtractObjectInfoInRegionServer::ExtractObjectInfoInRegionServer(std::string name) :
        PerceptionServer(name, "hsrb_1ms"),
        server(nh, name, boost::bind(&ExtractObjectInfoInRegionServer::execute, this, _1), false)
{

}