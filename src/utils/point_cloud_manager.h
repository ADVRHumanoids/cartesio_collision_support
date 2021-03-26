#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/io/io.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <octomap_msgs/OctomapWithPose.h>
#include <moveit_msgs/ApplyPlanningScene.h>

namespace XBot { namespace Planning {

class PointCloudManager {

public:
    
    PointCloudManager ( ros::NodeHandle& nh);
    
    void generatePointCloud();
    
    void callback(const octomap_msgs::OctomapPtr& msg);
    
    void sendPlanningScene();

    void publicPointCloud();

    pcl::PointCloud< pcl::PointXYZ > getPointCloud() const;


private:
    
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::Publisher _pub, _pspub;
    ros::ServiceClient _srv;
        
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pcl_pointcloud;
    octomap_msgs::Octomap _map;
    
    bool _callbackDone;
};
} }
