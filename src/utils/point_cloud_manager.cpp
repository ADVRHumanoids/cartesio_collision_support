#include "point_cloud_manager.h"

using namespace XBot::Planning;

PointCloudManager::PointCloudManager ( ros::NodeHandle& nh ):
    _pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>),
    _nh(nh)
    
{
    Eigen::Vector3d center;
    double side_x, side_y, side_z;
    double resolution = 0.05;
     
    double x, y, z;
    unsigned int index = 0;
    _pcl_pointcloud->resize(10000);       

    // ARC OBSTACLE
    // right wall
    center << 0.9, -0.6, 0.5;
    side_x = 0.3;
    side_z = 1.0;
    for(int i = 1; i <= (int)(side_x/resolution); i++)
    {
        x = center(0) - (side_x/2.0) + i*resolution;
        y = center(1);
        z = center(1) - (side_z/2.0) + i*resolution;
        for(int j = 1; j <= (int)(side_z/resolution); j++)
        {
            z = center(2) - (side_z/2.0) + j*resolution;
            _pcl_pointcloud->points[index].x = x;
            _pcl_pointcloud->points[index].y = y;
            _pcl_pointcloud->points[index].z = z;
            index ++;
        }
    }

    center << 0.9, 0.4, 0.5;
    side_x = 0.3;
    side_z = 1.0;
    for(int i = 1; i <= (int)(side_x/resolution); i++)
    {
        x = center(0) - (side_x/2.0) + i*resolution;
        y = center(1);
        z = center(1) - (side_z/2.0) + i*resolution;
        for(int j = 1; j <= (int)(side_z/resolution); j++)
        {
            z = center(2) - (side_z/2.0) + j*resolution;
            _pcl_pointcloud->points[index].x = x;
            _pcl_pointcloud->points[index].y = y;
            _pcl_pointcloud->points[index].z = z;
            index ++;
        }
    }

    center << 1.0, -0.1, 1.0;
    side_x = 0.3;
    side_y = 1.0;
    for(int i = 1; i <= (int)(side_x/resolution)-1; i++)
    {
        x = center(0) - (side_x/2.0) + i*resolution;
        for(int j = 1; j <= (int)(side_y/resolution); j++)
        {
            y = center(1) - (side_y/2.0) + j*resolution;
            z = center(2) + 0.0;
            _pcl_pointcloud->points[index].x = x;
            _pcl_pointcloud->points[index].y = y;
            _pcl_pointcloud->points[index].z = z;
            index ++;
        }
    }

    _pcl_pointcloud->header.frame_id = "world";
    
}


pcl::PointCloud< pcl::PointXYZ > PointCloudManager::getPointCloud()
{
    return *_pcl_pointcloud;
}




