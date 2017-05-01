#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_navigation_goals");

    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO_STREAM("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 5.0;
    goal.target_pose.pose.position.y = 5.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO_STREAM("Sending Goal");
    ac.sendGoal(goal);

    geometry_msgs::Point from;
    geometry_msgs::Point to;
    to.x = 5.0;
    to.y = 5.0;

    //occupancy_grid_utils::Cell fromCell = pointCell(from);
    //occupancy_grid_utils::Cell toCell = pointCell(to);

    //std::pair<occupancy_grid_utils::Path, double> AStarResult = shortestPathAStar(

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO_STREAM("Reached Goal!");
    } else {
        ROS_INFO_STREAM("Failed!");
    }

    return 0;
}


