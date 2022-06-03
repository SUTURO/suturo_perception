#include <suturo_perception/PerceptionServer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, EOI_NAME);

    ros::MultiThreadedSpinner spinner(4);

    ExtractObjectInfoServer eoi_server(EOI_NAME);
    ExtractDrawerInfoServer edi_server(EDI_NAME);
    ExtractTableInfoServer eti_server(ETI_NAME);
    //ExtractPlaneInfoServer epi(EDI_NAME);

    spinner.spin();

    return 0;
}
