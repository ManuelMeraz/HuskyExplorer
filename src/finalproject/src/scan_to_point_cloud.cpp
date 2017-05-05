#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud{

    public:

        ros::NodeHandle n_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
        tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
        ros::Publisher scan_pub_;

        sensor_msgs::LaserScan laser_scan;
        sensor_msgs::PointCloud point_cloud;

        LaserScanToPointCloud(ros::NodeHandle n) : n_(n),laser_sub_(n_, "scan", 10),laser_notifier_(laser_sub_,listener_, "map", 10)
    {
        laser_notifier_.registerCallback(
                boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/cloud",1);
    }

        void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
        {
            sensor_msgs::PointCloud cloud;
            sensor_msgs::LaserScan scan = *scan_in;
            cloud.header = scan.header;
            try
            {
                projector_.transformLaserScanToPointCloud(
                        "map",*scan_in, cloud,listener_);
            }
            catch (tf::TransformException& e)
            {
                std::cout << e.what();
                return;
            }

            // Do something with cloud.

            scan_pub_.publish(cloud);
            point_cloud = cloud;
            laser_scan = *scan_in;
        }
};

struct pole {
    geometry_msgs::Point32 position;
};

std::vector<pole> poles;

int main(int argc, char** argv)
{

    ros::init(argc, argv, "scan_to_point_cloud");
    ros::NodeHandle n;
    LaserScanToPointCloud lstpc(n);


    const double pole_radius = 0.07;
    ros::Rate rate(1);

    //start object detection
    while(ros::ok()){

        //This conditional will keep the node listening for data, and not making any calculations
        //as long as we have not recieved laser scan data.
        if(lstpc.laser_scan.ranges.size() == 0){
            ROS_INFO_STREAM("Waiting for laser data.");
            rate.sleep();
            ros::spinOnce();
            continue;
        }

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
        
        //True when pole was detected on last check, false when previous check did not detect pole.
        bool prev_pole_detected = false;

        for(int i = 1; i < lstpc.laser_scan.ranges.size() - 1; i++){
            if(isinf(lstpc.laser_scan.ranges[i]) || lstpc.laser_scan.ranges[i] > 6.0) continue;
            bool pole_detected = false;
            distance = lstpc.laser_scan.ranges[i];
            theta = atan(pole_radius / (distance + pole_radius));
            n = floor(theta / lstpc.laser_scan.angle_increment);
            if (i - n < 0 || i + n >= lstpc.laser_scan.ranges.size()){
                continue;
            }
            expected_distance_to_pole_edge = sqrt(pow(distance + pole_radius, 2) + pow(pole_radius, 2));
            pole_detected = (0.9 < (expected_distance_to_pole_edge / lstpc.laser_scan.ranges[i+n]) < 1.1 && 0.9 < (expected_distance_to_pole_edge / lstpc.laser_scan.ranges[i-n]) <  1.1);


            //Check that right side of pole has decreasing values. (A property of poles)
            for(int j = i + 1; j <= i + n; j++){
                if(lstpc.laser_scan.ranges[j] < lstpc.laser_scan.ranges[j-1]){
                    pole_detected = false;
                }
            }

            //Check that ranges[i+n+2] are out of range for a pole.
            if((lstpc.laser_scan.ranges[i+n+2] < lstpc.laser_scan.ranges[i] + (2*pole_radius)) && lstpc.laser_scan.ranges[i+n+2] > lstpc.laser_scan.ranges[i]){
                pole_detected = false;
            }
            //Check that left side of pole has decreasing values
            for(int j = i-1; j >= i-n; j--){
                if(lstpc.laser_scan.ranges[j] < lstpc.laser_scan.ranges[j+1]){
                    pole_detected = false;
                }
            }
            //Check that ranges[i-n-2] are out of range for a pole
            if(lstpc.laser_scan.ranges[i-n-2] < lstpc.laser_scan.ranges[i] + (2*pole_radius) && lstpc.laser_scan.ranges[i-n-2] > lstpc.laser_scan.ranges[i]){
                pole_detected = false;
            }

            if(pole_detected){
                for(int g = 0; g < lstpc.point_cloud.channels[1].values.size(); g++){
                   if(lstpc.point_cloud.channels[1].values[g] == i){
                       pole newpole;
                       newpole.position = lstpc.point_cloud.points[g];
                       poles.push_back(newpole);
                       //ROS_INFO_STREAM(newpole.position);
                       break;
                   }
                }
            }

        }
        if(poles.size() ==0){
            //ROS_INFO_STREAM("No poles detected.");
            if(prev_pole_detected){
                ROS_INFO_STREAM("Pair of poles out of range.");
                prev_pole_detected = false;
            }
        }else{
            //ROS_INFO_STREAM("Poles Detected: ");
            //ROS_INFO_STREAM(poles.size());
            for(int i = 0; i<poles.size(); i++){
                for(int j = i+1; j<poles.size(); j++){
                    float pole_d = hypot(poles[i].position.x - poles[j].position.x,poles[i].position.y - poles[j].position.y);
                    //ROS_INFO_STREAM("Potential Pole Distance:");
                   //ROS_INFO_STREAM(pole_d);
                    if(pole_d < 1.2 && pole_d > .8){
                        ROS_INFO_STREAM("\n\n\n\n\n");
                        ROS_INFO_STREAM(poles[i].position);
                        ROS_INFO_STREAM(poles[j].position);
                        ROS_INFO_STREAM("Pair of poles found.");
                        prev_pole_detected = true;
                    }else if(pole_d < 1.4 && pole_d > .6){
                        ROS_INFO_STREAM("\n\n\n\n\n");
                        ROS_INFO_STREAM(poles[i].position);
                        ROS_INFO_STREAM(poles[j].position);
                        ROS_INFO_STREAM("Potentially found pair of poles.");
                        prev_pole_detected = true;
                    }else if(prev_pole_detected){
                        ROS_INFO_STREAM("Pair of poles out of range.");
                        prev_pole_detected = false;
                    }
                }
            }
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
