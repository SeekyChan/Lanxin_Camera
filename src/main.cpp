#include <ros/ros.h>

#include "qr_code_identify_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_code_identify");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    qr_code_identify::QrCodeIdentifyNode node(nh, private_nh);
    if (!node.Start()) {
        ROS_ERROR("qr_code_identify failed to start because configuration is invalid");
        return 1;
    }

    ros::spin();
    return 0;
}
