#include <ros/ros.h>
#include <cartesian_interface/ros/RosClient.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_avoidance_issue");

    ros::NodeHandle nh;

    XBot::Cartesian::RosClient _ci_client;

    return 0;

}
