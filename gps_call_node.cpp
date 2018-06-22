#include <iostream>
#include <ros/ros.h>
#include "include/gps_conversion.h"

external_info::gps_conversion *gps_info_;

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "gps_conversion_node");
    ros::NodeHandle n_private("~");
    ros::NodeHandle n_pub;

    gps_info_ = new external_info::gps_conversion(n_pub, n_private);

    ros::spin();

    return 0;
}