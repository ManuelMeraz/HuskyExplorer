#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <vector>
#include <cmath>

class LaserScanToPointCloud;

void laser_handler(const sensor_msgs::LaserScan msg);
void cloud_handler(const sensor_msgs::PointCloud msg);
//Structure to store any poles that have been found.
struct pole {
    int start_index, end_index, center_index;
    geometry_msgs::Point32 position;
};

std::vector<pole> poles;

//Local copy of laser scan data
sensor_msgs::LaserScan laser_scan;
sensor_msgs::PointCloud point_cloud;
int main(int argc, char **argv){

    ros::init(argc,argv, "objectdetect");
    ros::NodeHandle nh;
    
    const double pole_radius = 0.07;
    
    ros::Subscriber laser_sub = nh.subscribe("scan",1000,laser_handler);
    ros::Subscriber cloud_sub = nh.subscribe("cloud",1000,cloud_handler);
    ros::Rate rate(1);
    
    //start object detection
    while(ros::ok()){

        //This conditional will keep the node listening for data, and not making any calculations
        //as long as we have not recieved laser scan data.
        if(laser_scan.ranges.size() == 0){
            ROS_INFO_STREAM("Waiting for laser data.");
            rate.sleep();
            ros::spinOnce();
            continue;
        }

        //if(point_cloud.header.seq == laser_scan.header.seq)
        //{
            ////ROS_INFO_STREAM("Cloud and Scan are aligned.");
            ////ROS_INFO_STREAM(laser_scan.header.seq);
            ////ROS_INFO_STREAM(point_cloud.header.seq);
        //}else{
            ////ROS_INFO_STREAM("Cloud and Scan mismatch. Pole detection aborted.");
            ////ROS_INFO_STREAM(laser_scan.header.seq);
            ////ROS_INFO_STREAM(point_cloud.header.seq);
            //rate.sleep();
            //ros::spinOnce();
            //continue;
        //}

        //resets the pole data for new reading.
        poles.clear();
        
        //This variable will hold the distance to the suspected distance to the center of ap ole.
        double distance = -1;
        
        //This will hold the angle from the suspected center of the pole where it's edge should be.
        double theta = -1;
        
        //This will hold the number of indexes needed to jump from the index of the center of the
        //pole, to the index of the reading for the edge of the pole.
        int n = -1;
        //This will hold the theoretical reading that the reading for the edge of thep pole should be
        double expected_distance_to_pole_edge = -1; 
        

        for(int i = 1; i < laser_scan.ranges.size() - 1; i++){
            if(isinf(laser_scan.ranges[i]) || laser_scan.ranges[i] > 6.0) continue;
            bool pole_detected = false;
            distance = laser_scan.ranges[i];
            theta = atan(pole_radius / (distance + pole_radius));
            n = floor(theta / laser_scan.angle_increment);
            if (i - n < 0 || i + n >= laser_scan.ranges.size()){
                continue;
            }
            expected_distance_to_pole_edge = sqrt(pow(distance + pole_radius, 2) + pow(pole_radius, 2));
            pole_detected = (0.9 < (expected_distance_to_pole_edge / laser_scan.ranges[i+n]) < 1.1 && 0.9 < (expected_distance_to_pole_edge / laser_scan.ranges[i-n]) <  1.1);
            
            
            //Check that right side of pole has decreasing values. (A property of poles)
            for(int j = i + 1; j <= i + n; j++){
                if(laser_scan.ranges[j] < laser_scan.ranges[j-1]){
                    pole_detected = false;
                }
            }
    
            //Check that ranges[i+n+2] are out of range for a pole.
            if((laser_scan.ranges[i+n+2] < laser_scan.ranges[i] + (2*pole_radius)) && laser_scan.ranges[i+n+2] > laser_scan.ranges[i]){
                pole_detected = false;
            }
            //Check that left side of pole has decreasing values
            for(int j = i-1; j >= i-n; j--){
                if(laser_scan.ranges[j] < laser_scan.ranges[j+1]){
                    pole_detected = false;
                }
            }
            //Check that ranges[i-n-2] are out of range for a pole
            if(laser_scan.ranges[i-n-2] < laser_scan.ranges[i] + (2*pole_radius) && laser_scan.ranges[i-n-2] > laser_scan.ranges[i]){
                pole_detected = false;
            }

            if(pole_detected){
                pole newpole;
                
                //ROS_INFO_STREAM(point_cloud.points[i]);
                //newpole.position = point_cloud.points[i];
                poles.push_back(newpole);
            }

        }
        if(poles.size() ==0){
            ROS_INFO_STREAM("No poles detected.");
        }else{
            ROS_INFO_STREAM("Poles Detected: ");
            ROS_INFO_STREAM(poles.size());
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

void cloud_handler(const sensor_msgs::PointCloud msg){
    point_cloud = msg;
   // ROS_INFO_STREAM("cloud_size");
   // ROS_INFO_STREAM(msg.points.size());
}


