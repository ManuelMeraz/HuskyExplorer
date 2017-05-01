#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

sensor_msgs::PointCloud point_cloud;
sensor_msgs::LaserScan scan;

void cloud_callback(const sensor_msgs::PointCloud &msg);
void scan_callback(const sensor_msgs::LaserScan &msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_detector");
    ros::NodeHandle nh;

    ros::Subscriber point_sub = nh.subscribe("/cloud", 1000, &cloud_callback);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 1000, &scan_callback);
    ros::Publisher obstacle_pub = nh.advertise<geometry_msgs::PoseArray>("/obstacles", 1000);

    ros::Rate rate(0.5);
    while(ros::ok()) {
        geometry_msgs::PoseArray obstacles;
        int size = point_cloud.points.size();
        for(int i = 0; i < size; i++) {
            if(scan.ranges[i] <= 0.5) {
                geometry_msgs::Pose point;
                point.position.x = point_cloud.points[i].x;
                point.position.y = point_cloud.points[i].y;
                point.position.z = point_cloud.points[i].z;

                obstacles.poses.push_back(point);
            }
        }

        obstacle_pub.publish(obstacles);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


void cloud_callback(const sensor_msgs::PointCloud &msg) {
    point_cloud = msg;
}

void scan_callback(const sensor_msgs::LaserScan &msg) {
    scan = msg;
}
