#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

/* Contributed from the cartesio_planning project */

#include <rclcpp/executor.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

#include <xbot2_interface/xbotinterface2.h>


namespace XBot { namespace Cartesian { namespace Collision {

/**
 * @brief The CollisionDetection class provides collision detection capabilities
 * to the ModelInterface class. Changes to the environment are monitored internally
 * from suitable ROS topics between calls to startMonitor() and stopMonitor()
 */
class PlanningSceneWrapper
{

public:

    typedef std::shared_ptr<PlanningSceneWrapper> Ptr;

    /**
     * @brief CollisionDetection constructor. The class keeps a model pointer,
     * which is used for querying the robot state at each call to update()
     * @param model
     */
    PlanningSceneWrapper(ModelInterface::ConstPtr model,
                         urdf::ModelConstSharedPtr collision_urdf = nullptr,
                         srdf::ModelConstSharedPtr collision_srdf = nullptr,
                         rclcpp::Node::SharedPtr node = nullptr);

    /**
     * @brief startMonitor method starts monitoring changes in the environments
     * from topics, in a separate thread.
     */
    void startMonitor();

    /**
     * @brief stopMonitor method stops monitoring changes in the environment
     */
    void stopMonitor();

    /**
     * @brief startGetPlanningSceneServer
     */
    void startGetPlanningSceneServer();

    /**
     * @brief update method updates the internal collision detector model state
     * from the provided pointer to ModelInterface
     */
    void update();

    /**
     * @brief checkCollisions method checks the ModelInterface state at last call to
     * update() for collisions, either between robot links or with the environment.
     * @return true if collisions were found
     */
    bool checkCollisions() const;
    bool checkSelfCollisions() const;

    /**
     * @brief Get the names of the links that are involved in collisions for the current state
     */
    std::vector<std::string> getCollidingLinks() const;

    /**
     * @brief getCollidingChains
     * @return vector of colliding chains (computed from colliding links)
     */
    std::vector<XBot::Chain::ConstPtr> getCollidingChains() const;

    void applyPlanningScene(const moveit_msgs::msg::PlanningScene& scene);

    bool getPlanningScene(moveit_msgs::srv::GetPlanningScene::Request::ConstSharedPtr req,
                          moveit_msgs::srv::GetPlanningScene::Response::SharedPtr res);

    mutable collision_detection::AllowedCollisionMatrix acm;

private:

    ModelInterface::ConstPtr _model;

    planning_scene_monitor::PlanningSceneMonitorPtr _monitor;
    
    rclcpp::Node::SharedPtr _node;
    rclcpp::executors::SingleThreadedExecutor _executor;
    std::unique_ptr<std::thread> _monitor_thread;

    // ros::CallbackQueue _queue;
    // ros::NodeHandle _nh;
    // ros::AsyncSpinner _async_spinner;
    rclcpp::ServiceBase::SharedPtr _get_ps_srv;

    srdf::Model _srdf;

};

} } }

#endif // COLLISION_DETECTION_H
