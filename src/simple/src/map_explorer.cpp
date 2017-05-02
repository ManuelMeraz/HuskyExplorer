#include <ros/ros.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/optional.hpp>
#include <vector>
#include "MapExplorer.h"

namespace utils = occupancy_grid_utils;

nav_msgs::Odometry ekf_pose;
geometry_msgs::PoseWithCovarianceStamped amcl_pose;

//MapExplorer *map_helper;
nav_msgs::OccupancyGrid map;
utils::AStarResult path;
std::vector<geometry_msgs::Point> point_path;
std::vector<geometry_msgs::Point> vertices;
bool map_initialized = false;
bool path_complete = true;
bool new_pose = false;
int path_index = 0;
int vertix_index = 0;

void ekf_callback(const nav_msgs::Odometry &msg);
void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped &msg);
void map_callback(const nav_msgs::OccupancyGrid &msg);
void map_update_callback(const map_msgs::OccupancyGridUpdate &msg);
void obstacles_callback(const geometry_msgs::PoseArray &msg);
void adjust_path(const geometry_msgs::PoseArray &obstacles);

int main(int argc, char **argv) {
    ros::init(argc, argv, "averagefilter");
    ros::NodeHandle nh;

    ros::Subscriber ekf_sub = nh.subscribe("/odometry/filtered", 1000, &ekf_callback);
    ros::Subscriber map_sub = nh.subscribe("/move_base/global_costmap/costmap", 1000, &map_callback);
    ros::Subscriber map_update_sub = nh.subscribe("/move_base/global_costmap/costmap_updates", 1000, &map_update_callback);
    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1000, &amcl_callback);
    ros::Subscriber obstacles_sub = nh.subscribe("/obstacles", 1000, &obstacles_callback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    tf::TransformListener tf_listener(ros::Duration(10));

    geometry_msgs::Point to;
    to.x = -6;
    to.y = -2;

    vertices.push_back(to);

    to.x = 3.0;
    to.y = 3.0;

    vertices.push_back(to);

    //ros::Rate rate(0.01);
    while(ros::ok()) {
        // Spin first to acquire map
        ros::spinOnce();

        if (map_initialized) {

            if(path_complete && new_pose) {
                path_complete = false;

                geometry_msgs::Point to = vertices[vertix_index];
                geometry_msgs::Point from;
                from.x = amcl_pose.pose.pose.position.x;
                from.y = amcl_pose.pose.pose.position.y;

                utils::Cell fromCell = utils::pointCell(map.info, from);
                utils::Cell toCell = utils::pointCell(map.info, to);

                boost::optional<utils::AStarResult> possible_path = 
                    utils::shortestPathAStar(map, fromCell, toCell);

                if(possible_path.is_initialized()) {
                    ROS_INFO_STREAM("Path Success!");
                    path = possible_path.get();

                    int size = path.first.size();
                    for(int i = 0; i < size; i++) {
                        point_path.push_back(utils::cellCenter(map.info, path.first[i]));
                    }

                } else {
                    ROS_INFO_STREAM("Path Failed!");
                    path_complete = true;
                }

            } else {
                int size = point_path.size();
                while(path_index < size) {
                    geometry_msgs::Point vertix = point_path[path_index];


                    if(hypot(amcl_pose.pose.pose.position.x - vertix.x,
                                amcl_pose.pose.pose.position.y - vertix.y) > 0.5) {

                        double goal_heading = atan2(vertix.y - amcl_pose.pose.pose.position.y, 
                                vertix.x - amcl_pose.pose.pose.position.x);

                        tf::Quaternion q(
                                amcl_pose.pose.pose.orientation.x,
                                amcl_pose.pose.pose.orientation.y,
                                amcl_pose.pose.pose.orientation.z,
                                amcl_pose.pose.pose.orientation.w);

                        tf::Matrix3x3 m(q);
                        double roll, pitch, current_heading;
                        m.getRPY(roll, pitch, current_heading);

                        geometry_msgs::Twist drive_command;

                        double heading_diff = angles::shortest_angular_distance(current_heading, goal_heading);

                        if (fabs(heading_diff) > 0.2) {
                            if(heading_diff > 0) {
                                drive_command.angular.z = 0.1;
                            } else {
                                drive_command.angular.z = -0.1;
                            }
                        } else {
                            drive_command.linear.x = 0.2;
                        }

                        cmd_pub.publish(drive_command);
                        break;
                    } else {
                        path_index++;
                        if(path_index == size - 1) {
                            ROS_INFO_STREAM("Path Complete!");
                            ROS_INFO_STREAM("Distance to goal: " << hypot(amcl_pose.pose.pose.position.x - vertices[vertix_index].x,
                                        amcl_pose.pose.pose.position.y - vertices[vertix_index].y));

                            ROS_INFO_STREAM("Current Pose: X: " << amcl_pose.pose.pose.position.x << " Y: " << amcl_pose.pose.pose.position.y);

                            ROS_INFO_STREAM("Goal Pose: X: " << vertix.x << " Y: " << vertix.y);

                            path_complete = true;
                            path_index = 0;
                            vertix_index++;

                            int size= vertices.size();
                            if(vertix_index >= size) {
                                vertix_index = 0;
                            }

                            break;
                        }

                    }
                }
            }
        }
    }

    return 0;
}


void ekf_callback(const nav_msgs::Odometry &msg) {
    new_pose = true;
    ekf_pose = msg;
}

void map_callback(const nav_msgs::OccupancyGrid &msg) {
    if(!map_initialized) {
        map = msg;
        //map_helper = new MapExplorer(map);
        map_initialized = true;
    }
}

void map_update_callback(const map_msgs::OccupancyGridUpdate &msg) {
    int index = 0;
    for(int y=msg.y; y< msg.y+msg.height; y++){
        for(int x=msg.x; x< msg.x+msg.width; x++){
            utils::Cell gridCoord(x, y);
            int map_index = utils::cellIndex(map.info, gridCoord);
            if(map_index > -1)
                map.data[ map_index ] = msg.data[ index++ ]; 
        }

    }
}


void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    amcl_pose = msg;
}

void obstacles_callback(const geometry_msgs::PoseArray &msg) {
    int size = msg.poses.size();
    if(size > 0) {
        //ROS_INFO_STREAM("Obstacles detected. Adjusting Path...");
        //adjust_path(msg);
        //ROS_INFO_STREAM("Adjusted!");
    }
}

void adjust_path(const geometry_msgs::PoseArray &obstacles) {
    geometry_msgs::Point obstacle;
    int size = obstacles.poses.size();
    for(int i = 0; i < size; i++) {
        obstacle.x += obstacles.poses[i].position.x;
        obstacle.y += obstacles.poses[i].position.y;
        obstacle.z += obstacles.poses[i].position.z;
    }

    obstacle.x /= size;
    obstacle.y /= size;
    obstacle.z /= size;

    size = point_path.size();
    for(int i = path_index; i < size; i++) {
        if(hypot(point_path[i].x - obstacle.x, point_path[i].y - obstacle.y) < 1.0) {
            double move_away_direction = atan2(point_path[i].y - obstacle.y, point_path[i].x - obstacle.x);

            point_path[i].x = point_path[i].x + 1.0 * cos(move_away_direction);
            point_path[i].y = point_path[i].y + 1.0 * sin(move_away_direction);
        }
    }
}
