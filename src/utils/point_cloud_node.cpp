#include <ros/ros.h>
#include "point_cloud_manager.h"
#include <thread>
#include <chrono>

using namespace XBot::Planning;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    PointCloudManager pc_manager(nh);
    std::cout << "generating PointCloud..." << std::endl;
    pc_manager.generatePointCloud();

    while(!pc_manager.sendPlanningScene())
    {
        pc_manager.publicPointCloud();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

    while(ros::ok())
    {
        pc_manager.publicPointCloud();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
