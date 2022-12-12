#include <ros/ros.h>
#include <string>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include "utils/kinetic_math.hpp"

static geometry_msgs::PoseStamped UGV_pose_sub,UGV_estimated_pose_pub;
static Vec7 UGV_lp; // xyz yaw
static double Last_time;

void ugv_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UGV_pose_sub.pose.position.x = pose->pose.position.x;
    UGV_pose_sub.pose.position.y = pose->pose.position.y;
    UGV_pose_sub.pose.position.z = pose->pose.position.z;
    UGV_pose_sub.pose.orientation.w = pose->pose.orientation.w;
    UGV_pose_sub.pose.orientation.x = pose->pose.orientation.x;
    UGV_pose_sub.pose.orientation.y = pose->pose.orientation.y;
    UGV_pose_sub.pose.orientation.z = pose->pose.orientation.z;
    UGV_lp << UGV_pose_sub.pose.position.x,UGV_pose_sub.pose.position.y,UGV_pose_sub.pose.position.z,
              UGV_pose_sub.pose.orientation.w,UGV_pose_sub.pose.orientation.x,
              UGV_pose_sub.pose.orientation.y,UGV_pose_sub.pose.orientation.z;
}

Vec7 PoseEstimator(Vec7 UGV_lp){
    Vec7 EstimatedPose;
    Quaterniond localq(UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6]);
    Vec3 localrpy = Q2rpy(localq);
    Vec4 pose_XYZyaw = Vec4(UGV_lp[0],UGV_lp[1],UGV_lp[2],localrpy[2]);
    double iteration_time = ros::Time::now().toSec() - Last_time;


    Last_time = ros::Time::now().toSec();
    return(EstimatedPose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Failsafe");
    ros::NodeHandle nh;
    ros::Subscriber ugvpose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("/vrpn_client_node/gh034_car/pose", 5, ugv_pose_sub);
    ros::Publisher  ugv_estpos_pub = nh.advertise<geometry_msgs::PoseStamped>
                    ("/mavros/UGV_estimated_pose/pose", 5);
    ros::Rate loop_rate(20); /* ROS system Hz */

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}
