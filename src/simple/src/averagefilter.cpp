#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "distance_angle_pose_to_point.h"

geometry_msgs::PoseWithCovarianceStamped amcl_pose;
nav_msgs::Odometry ekf_pose;
nav_msgs::OccupancyGrid map;

void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped &msg);
void ekf_callback(const nav_msgs::Odometry &msg);
void map_callback(const nav_msgs::OccupancyGrid &msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "averagefilter");
    ros::NodeHandle nh;

    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1000, &amcl_callback);
    ros::Subscriber ekf_sub = nh.subscribe("/odometry/filtered", 1000, &ekf_callback);
    ros::Subscriber map_sub = nh.subscribe("/map", 1000, &map_callback);
    ros::Publisher avg_pub = nh.advertise<geometry_msgs::Pose>("/averagepose", 1000);

    ros::Rate rate(1);
    geometry_msgs::Pose avg_pose;
    geometry_msgs::PoseStamped test_pose;
    while(ros::ok()) {
        avg_pose.position.x = (amcl_pose.pose.pose.position.x + ekf_pose.pose.pose.position.x) / 2;

        avg_pose.position.y = (amcl_pose.pose.pose.position.y + ekf_pose.pose.pose.position.y) / 2;

        avg_pose.orientation.z = (amcl_pose.pose.pose.orientation.z + ekf_pose.pose.pose.orientation.z) / 2;
        avg_pose.orientation.w = (amcl_pose.pose.pose.orientation.w + ekf_pose.pose.pose.orientation.w) / 2;
        avg_pub.publish(avg_pose);

        test_pose.pose = ekf_pose.pose.pose;
        test_pose.header = ekf_pose.header;
        
        if(ekf_pose.pose.pose.orientation.w != 0) {
            ROS_INFO_STREAM("transforming...");
            poseToMap(test_pose, map);
            ROS_INFO_STREAM("done");
        }

        rate.sleep();

        ros::spinOnce();
    }

    return 0;
}


void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    amcl_pose = msg;
}

void ekf_callback(const nav_msgs::Odometry &msg) {
    ekf_pose = msg;
}

void map_callback(const nav_msgs::OccupancyGrid &msg) {
    map = msg;
}
