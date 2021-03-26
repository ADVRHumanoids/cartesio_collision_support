#include <ros/ros.h>
#include "point_cloud_manager.h"
#include <thread>
#include <chrono>

using namespace XBot::Planning;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_node");
    ros::NodeHandle nh;
    
    PointCloudManager pc_manager(nh);
    std::cout << "generating PointCloud..." << std::endl;
    pc_manager.generatePointCloud();
    for (int i = 0; i < 100; i++)
    {
        pc_manager.publicPointCloud();
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    pc_manager.sendPlanningScene();

    ros::Rate rate(30);
    while(ros::ok())
    {
        pc_manager.publicPointCloud();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
