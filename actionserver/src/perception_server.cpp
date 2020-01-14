#include <PerceptionServer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, EOI_NAME);

    ros::MultiThreadedSpinner spinner(4);
    ExtractObjectInfoServer eoi_server(EOI_NAME);
    ExtractObjectInfoInRegionServer eoi_ir_server(EOI_IR_NAME);
    ExtractPlaneInfoServer epi_server(EPI_NAME);

    spinner.spin();

    return 0;
}
