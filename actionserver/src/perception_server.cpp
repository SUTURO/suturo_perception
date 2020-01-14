#include <PerceptionServer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perception_actionserver");

    ros::MultiThreadedSpinner spinner(4);
    ExtractObjectInfoServer server("perception_actionserver");
    //ExtractPlaneInfoServer server2("perception_actionserver_planes");

    spinner.spin();

    return 0;
}
