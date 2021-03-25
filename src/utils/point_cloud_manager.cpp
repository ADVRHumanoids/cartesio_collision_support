#include "point_cloud_manager.h"

using namespace XBot::Planning;

PointCloudManager::PointCloudManager ( ros::NodeHandle& nh ):
    _pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>),
    _nh(nh),
    _callbackDone(false)
    
{
    _pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("input_cloud", 10, true);
    _sub = _nh.subscribe("octomap_binary", 10, &PointCloudManager::callback, this);
    
    _srv = _nh.serviceClient<moveit_msgs::ApplyPlanningScene>("cartesian/collision/apply_planning_scene");
    _srv.waitForExistence();
}

void PointCloudManager::callback ( const octomap_msgs::OctomapPtr& msg ) {
    _map.header = msg->header;
    _map.binary = msg->binary;
    _map.resolution = msg->resolution;
    _map.id = msg->id;
    _map.data = msg->data;
    
    _callbackDone = true;
}

void PointCloudManager::generatePointCloud()
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
    _pub.publish(_pcl_pointcloud);
    
}

void PointCloudManager::sendPlanningScene() 
{
    moveit_msgs::ApplyPlanningScene::Request req;
    moveit_msgs::ApplyPlanningScene::Response res;
    
    if (!_callbackDone)
    {
        std::cout << "msg still not generated!" << std::endl;
        return;
    }
    
    octomap_msgs::OctomapWithPose msg;

    // Assign the same header
    msg.header = _map.header;

    // Octree  frame coincident with the header frame
    msg.origin.position.x = 0;
    msg.origin.position.y = 0;
    msg.origin.position.z = 0;
    msg.origin.orientation.x = 0;
    msg.origin.orientation.y = 0;
    msg.origin.orientation.z = 0;
    msg.origin.orientation.w = 1;

    // Assign the map
    msg.octomap.header = _map.header;
    msg.octomap.binary = _map.binary;
    msg.octomap.id = _map.id;
    msg.octomap.resolution = _map.resolution;
    msg.octomap.data = _map.data;
    
    req.scene.world.octomap = msg;
    req.scene.is_diff = true;
    _srv.call(req, res);
}


pcl::PointCloud< pcl::PointXYZ > PointCloudManager::getPointCloud() const
{
    return *_pcl_pointcloud;
}




