#include "include/gps_conversion.h"
#include "tf_conversions/tf_eigen.h"

external_info::gps_conversion::gps_conversion(ros::NodeHandle& nh_pub, ros::NodeHandle& nh_private):frame_num_(0)
{
    // Load launch file parameters
    nh_private.param<std::string>("gps_sub_topic", gps_sub_topic, "/gps/pose");
    nh_private.param<std::string>("pub_topic", pose_pub_topic, "/gps_aligned/pose");
    nh_private.param<std::string>("parent_frame", parent_frame, "world");
    nh_private.param<std::string>("child_frame", child_frame, "refined_gps");
    nh_private.param<bool>("align_positive", align_pose_, true);

    gps_sub_ = nh_pub.subscribe(gps_sub_topic, 100, &gps_conversion::gps_callback, this);

    if(align_pose_)
        rot_imu2cam << 1, 0, 0, 0, 0, 1, 0, -1, 0;
    else
        rot_imu2cam << 0, -1, 0, 0, 0, -1, 1, 0, 0;

    q_imu2cam = rot_imu2cam;

    gps_pub_ = nh_pub.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_pub_topic, 100);

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

    Eigen::Isometry3d pose2enu = Eigen::Isometry3d::Identity();

    pose2enu.linear() = gps_frame_rot.toRotationMatrix();

    pose2enu.translation() = gps_frame_translation;

    Eigen::Isometry3d cam_frame_pose = Eigen::Isometry3d::Identity();

    cam_frame_pose = rot_imu2cam * gps_frame_rot_0_inverse * pose2enu ;  //pose(k) to enu, enu to pose(0), pose(0) to cam(0)

//    cam_frame_pose = pose2enu * gps_frame_rot_0_inverse * q_imu2cam;

    Eigen::Quaterniond orientation(cam_frame_pose.matrix().topLeftCorner<3,3>());

    orientation = orientation * q_imu2cam.inverse();

    cam_frame_pose.linear() = orientation.toRotationMatrix();

    if(frame_num_ == 0){

        off_v = cam_frame_pose.translation();
        off_g = pose2enu.translation();

        pose2enu.translation() = cam_frame_pose.translation() = Eigen::Vector3d(0, 0, 0);

    }
    else{

        cam_frame_pose.translation() -= off_v;

        pose2enu.translation() -= off_g;
    }

    frame_num_ ++;

    tf::Transform transform_cam_frame, transform_gps_frame;
//
//    transform_cam_frame.setOrigin( tf::Vector3(cam_frame_pose.matrix()(0,3), cam_frame_pose.matrix()(1,3), cam_frame_pose.matrix()(2,3)));
//    transform_cam_frame.setRotation( tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()));

    tf::poseEigenToTF(cam_frame_pose, transform_cam_frame);
    tf::poseEigenToTF(pose2enu, transform_gps_frame);

    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.stamp = gps_pose.header.stamp;
    pose_msg.header.frame_id = parent_frame;

    tf::poseTFToMsg(transform_cam_frame, pose_msg.pose.pose);

    gps_pub_.publish(pose_msg);

    br.sendTransform(tf::StampedTransform(transform_cam_frame, gps_pose.header.stamp, parent_frame, child_frame));

    br_gps.sendTransform(tf::StampedTransform(transform_gps_frame, gps_pose.header.stamp, parent_frame, "gps"));

}