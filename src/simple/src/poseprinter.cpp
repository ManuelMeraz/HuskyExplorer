#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

void pose_callback(const geometry_msgs::Pose &msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "poseprinter");
    ros::NodeHandle nh;


    ros::Subscriber sub = nh.subscribe("/averagepose", 1000, &pose_callback);

    ros::spin();

    return 0;
}


void pose_callback(const geometry_msgs::Pose &msg) {
    ROS_INFO_STREAM(msg);
}
