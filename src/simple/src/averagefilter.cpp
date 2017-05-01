#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/optional.hpp>
#include "MapExplorer.h"

namespace utils = occupancy_grid_utils;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

nav_msgs::Odometry ekf_pose;

nav_msgs::OccupancyGrid map;
bool map_initialized = false;

void ekf_callback(const nav_msgs::Odometry &msg);
void map_callback(const nav_msgs::OccupancyGrid &msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "averagefilter");
    ros::NodeHandle nh;

    ros::Subscriber ekf_sub = nh.subscribe("/odometry/filtered", 1000, &ekf_callback);
    ros::Subscriber map_sub = nh.subscribe("/map", 1000, &map_callback);

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO_STREAM("Waiting for the move_base action server to come up");
    }


    ros::Rate rate(1);
    move_base_msgs::MoveBaseGoal goal;
    while(ros::ok()) {
        // Spin first to acquire map
        ros::spinOnce();

        if (map_initialized) {
            geometry_msgs::Point from;
            from.x = ekf_pose.pose.pose.position.x;
            from.y = ekf_pose.pose.pose.position.y;

            geometry_msgs::Point to;
            to.x = 4.0;
            to.y = 4.0;

            utils::Cell fromCell = utils::pointCell(map.info, from);
            utils::Cell toCell = utils::pointCell(map.info, to);

            boost::optional<utils::AStarResult> possible_path = 
                utils::shortestPathAStar(map, fromCell, toCell).get();

            utils::AStarResult path;
            if(possible_path.is_initialized()) {
                ROS_INFO_STREAM("Path Success!");
                path = possible_path.get();
                int size = path.first.size();
                for(int i = 0; i < size; i++) {
                    geometry_msgs::Point vertix = utils::cellCenter(map.info, path.first[i]);

                    if(hypot(ekf_pose.pose.pose.position.x - vertix.x,
                                ekf_pose.pose.pose.position.y - vertix.y) > 1) {
                        ROS_INFO_STREAM("Distance to goal: " << hypot(ekf_pose.pose.pose.position.x - 4.0,
                                    ekf_pose.pose.pose.position.y - 4.0));

                        // Goal Yaw
                        double heading = atan2(vertix.y - goal.target_pose.pose.position.y, 
                                vertix.x - goal.target_pose.pose.position.x);

                        tf::Quaternion q;
                        q.setEuler(0, 0, heading);
                        goal.target_pose.pose.orientation.x = q[0];
                        goal.target_pose.pose.orientation.y = q[1];
                        goal.target_pose.pose.orientation.z = q[2];
                        goal.target_pose.pose.orientation.w = q[3];
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.header.frame_id = "odom";
                        goal.target_pose.pose.position.x = vertix.x;
                        goal.target_pose.pose.position.y = vertix.y;


                        ac.sendGoal(goal);
                        ac.waitForResult();

                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                            ROS_INFO_STREAM("Reached Goal!");
                        } else {
                            ROS_INFO_STREAM("Failed!");
                        }
                    }
                }
            } else {
                ROS_INFO_STREAM("Path Failed!");
            }
        }
        rate.sleep();
    }

    //delete map;
    return 0;
}


void ekf_callback(const nav_msgs::Odometry &msg) {
    ekf_pose = msg;
}

void map_callback(const nav_msgs::OccupancyGrid &msg) {
    if(!map_initialized) {
        map = msg;
        //map =  new MapExplorer(msg);
        map_initialized = true;
    }
}
