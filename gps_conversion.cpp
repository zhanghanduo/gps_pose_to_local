#include "include/gps_conversion.h"


external_info::gps_conversion::gps_conversion(ros::NodeHandle& nh_pub, ros::NodeHandle& nh_private)
{
    // Load launch file parameters
    nh_private.param<std::string>("gps_sub", gps_sub_topic, "/gps/pose");
    nh_private.param<std::string>("parent_frame", parent_frame, "world");
    nh_private.param<std::string>("child_frame", child_frame, "refined_gps");
    nh_private.param<bool>("align_positive", align_pose_, true);

    gps_sub_ = nh_pub.subscribe(gps_sub_topic, 1000, &gps_conversion::gps_callback, this);

    if(align_pose_)
        rot_c0_to_g0 << 0, 1, 0, 0, 0, -1, -1, 0, 0;
    else
        rot_c0_to_g0 << 0, -1, 0, 0, 0, -1, 1, 0, 0;

    ROS_INFO("GPS pose conversion node initialized.");
}

void external_info::gps_conversion::gps_callback(const geometry_msgs::PoseWithCovarianceStamped &gps_pose)
{
    Eigen::Quaterniond gps_frame_rot(gps_pose.pose.pose.orientation.w, gps_pose.pose.pose.orientation.x,
                           gps_pose.pose.pose.orientation.y, gps_pose.pose.pose.orientation.z);

    Eigen::Vector3d gps_frame_translation(gps_pose.pose.pose.position.x, gps_pose.pose.pose.position.y, gps_pose.pose.pose.position.z);

    if(frame_num_ == 0) {

        gps_frame_rot_0_inverse = gps_frame_rot.inverse();

    }

    Eigen::Isometry3d gps_tf = Eigen::Isometry3d::Identity();

    gps_tf.linear() = gps_frame_rot.toRotationMatrix();

    gps_tf.translation() = gps_frame_translation;

    Eigen::Isometry3d cam_frame_pose = Eigen::Isometry3d::Identity();

    cam_frame_pose = bias * rot_c0_to_g0 * gps_frame_rot_0_inverse * gps_tf;

    if(frame_num_ == 0){

        Eigen::Vector3d off_v = cam_frame_pose.translation();

        off_x = off_v(0);
        off_y = off_v(1);
        off_z = off_v(2);

        cam_frame_pose.translation() = Eigen::Vector3d(0, 0, 0);

    }
    else{
        Eigen::Vector3d t_v = cam_frame_pose.translation();

        t_v(0) -=  off_x;
        t_v(1) -=  off_y;
        t_v(2) -=  off_z;

        cam_frame_pose.translation() = t_v;
    }

    frame_num_ ++;

    Eigen::Quaterniond orientation(cam_frame_pose.matrix().topLeftCorner<3,3>());

    tf::Transform transform_gps;

    transform_gps.setOrigin( tf::Vector3(cam_frame_pose.matrix()(0,3), cam_frame_pose.matrix()(1,3), cam_frame_pose.matrix()(2,3)));
    transform_gps.setRotation( tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()));

    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.stamp = gps_pose.header.stamp;
    pose_msg.header.frame_id = parent_frame;

    tf::poseTFToMsg(transform_gps, pose_msg.pose.pose);

    gps_pub_.publish(pose_msg);

    br.sendTransform(tf::StampedTransform(transform_gps, ros::Time::now(), parent_frame, child_frame));

}