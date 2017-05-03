#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <boost/optional.hpp>
#include <vector>

namespace utils = occupancy_grid_utils;

nav_msgs::Odometry ekf_pose;
geometry_msgs::PoseWithCovarianceStamped amcl_pose;

random_numbers::RandomNumberGenerator* rng;
nav_msgs::OccupancyGrid map;
utils::AStarResult path;
std::vector<geometry_msgs::Point> point_path;
bool map_initialized = false;
bool path_complete = true;
bool new_pose = false;
bool obstacle_detected = true;
bool avoiding_obstacle = false;
bool turn_direction = true;
int path_index = 0;

void ekf_callback(const nav_msgs::Odometry &msg);
void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped &msg);
void map_callback(const nav_msgs::OccupancyGrid &msg);
void map_update_callback(const map_msgs::OccupancyGridUpdate &msg);
void scan_callback(const sensor_msgs::LaserScan &msg);

ros::Publisher cmd_pub;
int main(int argc, char **argv) {
    ros::init(argc, argv, "averagefilter");
    ros::NodeHandle nh;

    ros::Subscriber ekf_sub = nh.subscribe("/odometry/filtered", 1000, &ekf_callback);
    ros::Subscriber map_sub = nh.subscribe("/move_base/global_costmap/costmap", 1000, &map_callback);
    ros::Subscriber map_update_sub = nh.subscribe("/move_base/global_costmap/costmap_updates", 1000, &map_update_callback);
    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1000, &amcl_callback);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 1000, &scan_callback);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    while(ros::ok()) {
        // Spin first to acquire map
        ros::spinOnce();

        if (map_initialized && !obstacle_detected) {

            if(path_complete && new_pose) {
                path_complete = false;
                avoiding_obstacle = false;

                rng = new random_numbers::RandomNumberGenerator();
                geometry_msgs::Point to;

                double random_heading = rng->uniformReal(0, 2 * M_PI);
                to.x = amcl_pose.pose.pose.position.x + 8.5 * cos(random_heading);
                to.y = amcl_pose.pose.pose.position.y + 8.5 * sin(random_heading);

                while( map.data[utils::cellIndex(map.info,utils::pointCell(map.info,to))] != 0) {
                    random_heading = rng->uniformReal(0, 2 * M_PI);
                    to.x = amcl_pose.pose.pose.position.x + 3.5 * cos(random_heading);
                    to.y = amcl_pose.pose.pose.position.y + 3.5 * sin(random_heading);
                }

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
                    geometry_msgs::Twist drive_command;
                    drive_command.linear.x = 0.2;
                    cmd_pub.publish(drive_command);
                    path_complete = true;
                    path_index = 0;
                    point_path.clear();
                }
                delete rng;

            } else {
                int size = point_path.size();
                while(path_index < size) {
                    geometry_msgs::Point vertix = point_path[path_index];


                    if(hypot(amcl_pose.pose.pose.position.x - vertix.x,
                                amcl_pose.pose.pose.position.y - vertix.y) > 1.0) {

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

                        if (fabs(heading_diff) > 0.4) {
                            if(heading_diff > 0) {
                                drive_command.angular.z = 0.5;
                            } else {
                                drive_command.angular.z = -0.5;
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
                            ROS_INFO_STREAM("Distance to goal: " << hypot(amcl_pose.pose.pose.position.x - point_path[point_path.size() - 1].x,
                                        amcl_pose.pose.pose.position.y - point_path[point_path.size() - 1].y));

                            ROS_INFO_STREAM("Current Pose: X: " << amcl_pose.pose.pose.position.x << " Y: " << amcl_pose.pose.pose.position.y);

                            ROS_INFO_STREAM("Goal Pose: X: " << vertix.x << " Y: " << vertix.y);

                            path_complete = true;
                            point_path.clear();
                            path_index = 0;

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
    ekf_pose = msg;
}

void map_callback(const nav_msgs::OccupancyGrid &msg) {
    if(!map_initialized) {
        map = msg;
        map_initialized = true;
    }
}

void map_update_callback(const map_msgs::OccupancyGridUpdate &msg) {
    if(map_initialized) {
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
}


void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    new_pose = true;
    amcl_pose = msg;
}

void scan_callback(const sensor_msgs::LaserScan &msg) {
    int size = msg.ranges.size();
    double min = DBL_MAX;
    // Scan only the laser ranges in front of the husky
    for(int i = 0; i <= size; i++) {
        if(msg.ranges[i] > 0.3 && msg.ranges[i] < 0.9 && !isinf(msg.ranges[i])) {

            if(!avoiding_obstacle) {
                avoiding_obstacle = true;

                if(i < size/2) {
                    turn_direction = true;
                } else {
                    turn_direction = false;
                }
            }

            ROS_INFO_STREAM("Detected obstacle: " << msg.ranges[i] << "m away");
            obstacle_detected = true;

            // Get new path
            path_complete = true;
            point_path.clear();
            path_index = 0;

            geometry_msgs::Twist drive_command;

            // Obstacle is on the right side
            if(turn_direction) {
                // turn right
                drive_command.angular.z = -0.5;

            // Obstacle is on the left
            } else{
                //turn left
                drive_command.angular.z = 0.5;
            } 

            cmd_pub.publish(drive_command);
            return;
        }
    }
    obstacle_detected = false;
}

