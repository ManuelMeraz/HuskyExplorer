#ifndef MAPEXPLORER_H
#define MAPEXPLORER_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <iterator>
#include <vector>


class MapExplorer {
    public: 
        MapExplorer(const nav_msgs::OccupancyGrid &map);

        // First is distance to closest object, Second is angle 
        std::pair<double, double> poseToMap(geometry_msgs::PoseStamped &pt);

    private:
        // information of the map
        struct MapData {
            std_msgs::Header header;
            std::vector<int8_t> data;
            double size_x; // resolution width
            double size_y; // resolution height
            double scale; //  scale in  meters/pixel

            // origin pose of map
            double origin_x; 
            double origin_y;
        } map_data;


        // Returns the index in map based off of row and column in grid
        int map_index(const MapData &map, const std::size_t &row,
                const std::size_t &column);

        // Converts grid values based off map resolution to a pose in meters
        double grid_to_pose_x(const MapData &map, const std::size_t &grid_x);
        double grid_to_pose_y(const MapData &map, const std::size_t &grid_y);
};

#endif
