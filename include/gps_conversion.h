//
// Created by hd on 18-6-22.
//

#ifndef GPS_CALL_GPS_CONVERSION_H
#define GPS_CALL_GPS_CONVERSION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <string>

//#include "utils/CameraPose.hpp"

namespace external_info
{
    class gps_conversion
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        gps_conversion(ros::NodeHandle&, ros::NodeHandle&);

        // used to save the gps pose
//        CameraPose gpsPose_;

    private:
        std::string timestring, parent_frame, child_frame;

        tf::TransformBroadcaster br;

        Eigen::Quaterniond gps_frame_rot_0_inverse, bias, baseRgt;

        Eigen::Vector3d baseTgt;

        Eigen::Matrix3d rot_c0_to_g0;

        double theta, gamma1, off_x, off_y, off_z;

        ros::Publisher gps_pub_;

        ros::Subscriber gps_sub_;

        std::string gps_sub_topic, pose_pub_topic;

        bool align_pose_;

        unsigned int frame_num_;

        /**
        * @brief
        *   GPS infomation callback.
        */
        void gps_callback(const geometry_msgs::PoseWithCovarianceStamped & gps_pose);
    };


}

#endif //GPS_CALL_GPS_CONVERSION_H
