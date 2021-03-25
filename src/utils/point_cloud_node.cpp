#include <ros/ros.h>
#include "point_cloud_manager.h"

using namespace XBot::Planning;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_node");
    ros::NodeHandle nh;
    
    PointCloudManager pc_manager(nh);
//     pc_manager.generatePointCloud();
//     
//     ros::Rate rate(30);
//     while(ros::ok())
//     {
//         ros::spinOnce();
//         pc_manager.sendPlanningScene();
//         rate.sleep();
//     }
    
    return 0;
}
