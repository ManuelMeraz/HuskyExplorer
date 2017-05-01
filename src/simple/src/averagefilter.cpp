#include <ros/ros.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/optional.hpp>
#include "MapExplorer.h"

namespace utils = occupancy_grid_utils;

nav_msgs::Odometry ekf_pose;

//MapExplorer *map_helper;
nav_msgs::OccupancyGrid map;
utils::AStarResult path;
//std::vector<geometry_msgs::Point> adjusted_path;
bool map_initialized = false;
bool path_complete = true;
int path_index = 0;

void ekf_callback(const nav_msgs::Odometry &msg);
void map_callback(const nav_msgs::OccupancyGrid &msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "averagefilter");
    ros::NodeHandle nh;

    ros::Subscriber ekf_sub = nh.subscribe("/odometry/filtered", 1000, &ekf_callback);
    ros::Subscriber map_sub = nh.subscribe("/map", 1000, &map_callback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    //ros::Rate rate(0.01);
    while(ros::ok()) {
        // Spin first to acquire map
        ros::spinOnce();

        if (map_initialized) {

            if(path_complete) {
                path_complete = false;
                geometry_msgs::Point from;
                from.x = ekf_pose.pose.pose.position.x;
                from.y = ekf_pose.pose.pose.position.y;

                geometry_msgs::Point to;
                to.x = -6.0;
                to.y = -0.0;

                utils::Cell fromCell = utils::pointCell(map.info, from);
                utils::Cell toCell = utils::pointCell(map.info, to);

                boost::optional<utils::AStarResult> possible_path = 
                    utils::shortestPathAStar(map, fromCell, toCell).get();

                if(possible_path.is_initialized()) {
                    ROS_INFO_STREAM("Path Success!");
                    path = possible_path.get();

                    //ROS_INFO_STREAM("Adjusting Path...");
                    //int size = path.first.size();
                    //for(int i = 0; i < size; i++) {
                        //geometry_msgs::PoseStamped goal_pose;
                        //goal_pose.header.stamp = ros::Time::now();
                        //goal_pose.header.frame_id = "odom";
                        //goal_pose.pose.position.x = vertix.x;
                        //goal_pose.pose.position.y = vertix.y;
                        //goal_pose.pose.orientation.w = 1.0;

                        //std::pair<double, double> nearest_obstacle = map_helper->poseToMap(goal_pose);

                        //if(nearest_obstacle.first < 0.7) {
                            //vertix.x = vertix.x - 0.5 * cos(nearest_obstacle.second);
                            //vertix.y = vertix.y - 0.5 * sin(nearest_obstacle.second);
                        //}

                        //adjusted_path.push_back(vertix);

                    //}
                    //ROS_INFO_STREAM("Finished Adjusting Path");

                } else {
                    ROS_INFO_STREAM("Path Failed!");
                    path_complete = true;
                }

            } else {
                int size = path.first.size();
                while(path_index < size) {
                    geometry_msgs::Point vertix = utils::cellCenter(map.info, path.first[path_index]);


                    if(hypot(ekf_pose.pose.pose.position.x - vertix.x,
                                ekf_pose.pose.pose.position.y - vertix.y) > 0.1) {

                        ROS_INFO_STREAM("Distance to goal: " << hypot(ekf_pose.pose.pose.position.x - 4.0,
                                    ekf_pose.pose.pose.position.y - 4.0));

                        ROS_INFO_STREAM("Current Pose: X: " << ekf_pose.pose.pose.position.x << " Y: " << ekf_pose.pose.pose.position.y);

                        ROS_INFO_STREAM("Goal Pose: X: " << vertix.x << " Y: " << vertix.y);

                        double goal_heading = atan2(vertix.y - ekf_pose.pose.pose.position.y, 
                                vertix.x - ekf_pose.pose.pose.position.x);

                        tf::Quaternion q(
                                ekf_pose.pose.pose.orientation.x,
                                ekf_pose.pose.pose.orientation.y,
                                ekf_pose.pose.pose.orientation.z,
                                ekf_pose.pose.pose.orientation.w);

                        tf::Matrix3x3 m(q);
                        double roll, pitch, current_heading;
                        m.getRPY(roll, pitch, current_heading);

                        geometry_msgs::Twist drive_command;

                        double heading_diff = angles::shortest_angular_distance(current_heading, goal_heading);

                        if (fabs(heading_diff) > 0.4) {
                            if(heading_diff > 0) {
                                drive_command.angular.z = 0.2;
                            } else {
                                drive_command.angular.z = -0.2;
                            }
                        } else {
                            drive_command.linear.x = 0.5;
                        }

                        cmd_pub.publish(drive_command);
                        break;
                    } else {
                        path_index++;
                        if(path_index == size - 1) {
                            ROS_INFO_STREAM("Path Complete!");
                            path_complete = true;
                            path_index = 0;
                            break;
                        }

                    }
                }
            }
        }
        //rate.sleep();
    }

    //delete map_helper;
    return 0;
}


void ekf_callback(const nav_msgs::Odometry &msg) {
    ekf_pose = msg;
}

void map_callback(const nav_msgs::OccupancyGrid &msg) {
    if(!map_initialized) {
        map = msg;
        //map_helper = new MapExplorer(map);
        map_initialized = true;
    }
}
