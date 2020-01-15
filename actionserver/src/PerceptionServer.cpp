#include <PerceptionServer.h>
#include <vector>

PerceptionServer::PerceptionServer(std::string &name, std::string pipeline) :
        action_name(name),
        pm(name, pipeline)
{
    pm.init(pipeline);
    ROS_INFO("Initialized RoboSherlock..");
}