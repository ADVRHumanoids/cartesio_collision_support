#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_sphere_node");

    ros::NodeHandle node_handle, nhpr("~");

    ros::ServiceClient srv;

    srv = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("cartesian/collision_avoidance/apply_planning_scene");

    srv.waitForExistence();

    moveit_msgs::ApplyPlanningScene::Request req;
    req.scene.is_diff = true;

    // remove given id
    if(argc == 2)
    {
        moveit_msgs::CollisionObject co;
        co.header.frame_id = "world";
        co.operation = co.REMOVE;
        co.id = argv[1];

        req.scene.world.collision_objects = {co};
    }
    else if(argc == 6)
    {

        shape_msgs::SolidPrimitive solid;
        solid.type = solid.SPHERE;
        solid.dimensions = { std::atof(argv[1]) };

        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = std::atof(argv[2]);
        pose.position.y = std::atof(argv[3]);
        pose.position.z = std::atof(argv[4]);

        moveit_msgs::CollisionObject co;
        co.header.frame_id = "world";
        co.operation = co.ADD;
        co.id = argv[5];
        co.primitives = {solid};
        co.primitive_poses = {pose};

        req.scene.world.collision_objects = {co};

    }
    else if(argc == 8)
    {

        shape_msgs::SolidPrimitive solid;
        solid.type = solid.BOX;
        solid.dimensions = { std::atof(argv[1]), std::atof(argv[2]), std::atof(argv[3]) };

        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = std::atof(argv[4]);
        pose.position.y = std::atof(argv[5]);
        pose.position.z = std::atof(argv[6]);

        moveit_msgs::CollisionObject co;
        co.header.frame_id = "world";
        co.operation = co.ADD;
        co.id = argv[7];
        co.primitives = {solid};
        co.primitive_poses = {pose};

        req.scene.world.collision_objects = {co};

    }

    moveit_msgs::ApplyPlanningScene::Response res;
    srv.call(req, res);

}
