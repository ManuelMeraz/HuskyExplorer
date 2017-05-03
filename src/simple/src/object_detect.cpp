#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>

void laser_handler(const sensor_msgs::LaserScan msg);

struct pole {
    int start_index, end_index;
    geometry_msgs::Point position;
};

std::vector<pole> poles;

sensor_msgs::LaserScan laser_scan;

int main(int argc, char **argv){
    ros::init(argc,argv, "objectdetect");
    ros::NodeHandle nh;
    const double pole_radius = 0.07;

    ros::Subscriber laser_sub = nh.subscribe("scan",1000,laser_handler);
    ros::Rate rate(1);
    //start obstacle detection
    while(ros::ok()){
        if(laser_scan.ranges.size() == 0){
            //ROS_INFO_STREAM(laser_scan);
            ROS_INFO_STREAM("Awaiting pole dancers.");
            ros::spinOnce();
            continue;

        }

        double distance = -1;
        double theta = -1;
        int n = -1;
        double expected_distance_to_pole_edge = -1;
        poles.clear(); 
        for(int i = 1; i < laser_scan.ranges.size() - 1; i++){
            if(isinf(laser_scan.ranges[i]) || laser_scan.ranges[i] > 9.0) continue;
            bool pole_detected = false;
            distance = laser_scan.ranges[i];
            theta = asin(pole_radius / (distance + pole_radius));
            n = floor(theta / laser_scan.angle_increment);
            if (i - n < 0 || i + n >= laser_scan.ranges.size()){
                continue;
            }
            expected_distance_to_pole_edge = sqrt(pow(distance + pole_radius, 2) + pow(pole_radius, 2));
            pole_detected = (0.9 < (expected_distance_to_pole_edge / laser_scan.ranges[i+n]) < 1.1 && 0.9 < (expected_distance_to_pole_edge / laser_scan.ranges[i-n]) <  1.1);
            
            //Check that right side of pole has decreasing values
            for(int j = i + 1; j <= i + n; j++){
                if(laser_scan.ranges[j] < laser_scan.ranges[j-1]){
                    pole_detected = false;
                }
            }

            //Check that ranges[i+n+2] are out of range for a pole
            if((laser_scan.ranges[i+n+2] < laser_scan.ranges[i] + (2*pole_radius) + 0.02) && laser_scan.ranges[i+n+2] > laser_scan.ranges[i] - 0.02){
                pole_detected = false;
            }
            //Check that left side of pole has decreasing values
            for(int j = i-1; j >= i-n; j--){
                if(laser_scan.ranges[j] < laser_scan.ranges[j+1]){
                    pole_detected = false;
                }
            }
            //Check that ranges[i-n-2] are out of range for a pole
            if(laser_scan.ranges[i-n-2] < laser_scan.ranges[i] + (2*pole_radius) + 0.02 && laser_scan.ranges[i-n-2] > laser_scan.ranges[i] - 0.02){
                pole_detected = false;
            }

            if(pole_detected){
                pole newpole;
                poles.push_back(newpole);
            }

        }
        if(poles.size() ==0){
            ROS_INFO_STREAM("No pole Dancing here, Sir!");
        }else{
            ROS_INFO_STREAM("Pole dancers have arrived!");
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

void laser_handler(const sensor_msgs::LaserScan msg){
    //Take in the information for the laser scanner
    //	ROS_INFO_STREAM((float)msg.ranges[0]);
    laser_scan = msg;
    //ROS_INFO_STREAM(msg);
}

