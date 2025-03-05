#include "planning_scene_wrapper.h"

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/srdf_writer.h>
#include <xbot2_interface/common/utils.h>

namespace
{

/**
 * @brief The MonitorLockguardWrite class provides a RAII-style read-write lock
 * for the planning scene monitor. Constructing an object will acquire the lock,
 * which will automatically be released when the locker goes out of scope.
 * See also std::lock_guard<>.
 */
class MonitorLockguardWrite
{

public:

    MonitorLockguardWrite(planning_scene_monitor::PlanningSceneMonitorPtr monitor)
    {
        _monitor = monitor;
        // _monitor->lockSceneWrite();
    }

    ~MonitorLockguardWrite()
    {
        // _monitor->unlockSceneWrite();
    }

private:

    planning_scene_monitor::PlanningSceneMonitorPtr _monitor;
};


/**
 * @brief The MonitorLockguardRead class provides a RAII-style read-only lock
 * for the planning scene monitor. Constructing an object will acquire the lock,
 * which will automatically be released when the locker goes out of scope.
 * See also std::lock_guard<>.
 */
class MonitorLockguardRead
{

public:

    MonitorLockguardRead(planning_scene_monitor::PlanningSceneMonitorPtr monitor)
    {
        _monitor = monitor;
        // _monitor->lockSceneRead();
    }

    ~MonitorLockguardRead()
    {
        // _monitor->unlockSceneRead();
    }

private:

    planning_scene_monitor::PlanningSceneMonitorPtr _monitor;
};
}

namespace XBot { namespace Cartesian { namespace Collision {

PlanningSceneWrapper::PlanningSceneWrapper(ModelInterface::ConstPtr model,
                                           urdf::ModelConstSharedPtr collision_urdf,
                                           srdf::ModelConstSharedPtr collision_srdf,
                                           rclcpp::Node::SharedPtr node):
    _model(model),
    _node(node->create_sub_node("planning_scene"))
{
    // init urdf/srdf strings from model ifc
    std::string urdf_string = _model->getUrdfString();
    std::string srdf_string = _model->getSrdfString();

    // get urdf/srdf overrides
    if(collision_urdf)
    {
        urdf_string = Utils::urdfToString(*collision_urdf);
    }

    if(collision_srdf)
    {
        srdf_string = Utils::srdfToString(*collision_urdf, *collision_srdf);
    }

    // create robot model loader
    robot_model_loader::RobotModelLoader::Options rml_opt(urdf_string, srdf_string);

    auto rml = std::make_shared<robot_model_loader::RobotModelLoader>(_node, rml_opt);


    // planning scene monitor automatically updates planning scene from topics
    _monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(_node, rml);

    // save srdf
    _srdf.initString(*rml->getURDF(), srdf_string);
}

void PlanningSceneWrapper::startMonitor()
{
    // publish planning scene at 30 Hz (topic is ~/monitored_planning_scene)
    _monitor->setPlanningScenePublishingFrequency(20.); // tbd: hardcoded

    // this subscribes to /planning_scene
    _monitor->startSceneMonitor();

    _monitor->providePlanningSceneService();

    // this is somehow different from the scene monitor.. boh
    //    _monitor->startWorldGeometryMonitor();

    // this starts monitored planning scene publisher
    _monitor->startPublishingPlanningScene(
                planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                "monitored_planning_scene"
                );

    // AllowedCollisionMatrix definition. Entries can be added anywhere in the code simply
    // with acm.setEntry(std::string name1, std::string name2, bool allowed)
    acm = _monitor->getPlanningScene()->getAllowedCollisionMatrix();
}

void PlanningSceneWrapper::stopMonitor()
{
    _monitor->stopSceneMonitor();
    _monitor->stopWorldGeometryMonitor();
    _monitor->stopPublishingPlanningScene();
}

void PlanningSceneWrapper::startGetPlanningSceneServer()
{
    // using namespace std::placeholders;

    // // create new callback group
    // auto cbg = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    // // create executor
    // auto exec = rclcpp::executors::SingleThreadedExecutor::make_shared();

    // _get_ps_srv = _node->create_service<moveit_msgs::srv::GetPlanningScene>(
    //     "get_planning_scene",
    //     std::bind(&PlanningSceneWrapper::getPlanningScene, this, _1, _2),
    //     rclcpp::ServicesQoS(),
    //     cbg);

    // exec->add_callback_group(cbg, _node->get_node_base_interface());

    // // start async spinner
    // using namespace std::chrono_literals;
    // _monitor_thread = std::make_unique<std::thread>(
    //      [this, exec]()
    //      {
    //          for(;;)
    //          {
    //             exec->spin_all(1s);
    //          }
    //      });

    // _monitor_thread->detach();

}

void PlanningSceneWrapper::update()
{
    // acquire lock for thread-safe access to the planning scene
    planning_scene_monitor::LockedPlanningSceneRW ps(_monitor);

    // retrieve robot state data struct
    auto& robot_state = ps->getCurrentStateNonConst();

    // update planning scene from model
    for(const auto& jpair : _model->getUrdf()->joints_)
    {
        auto jname = jpair.first; // joint name
        auto jmodel = jpair.second; // urdf::Joint model
        auto jtype = jmodel->type; // joint type

        if(jtype == urdf::Joint::REVOLUTE  ||
                jtype == urdf::Joint::PRISMATIC ||
                jtype == urdf::Joint::CONTINUOUS)
        {
            double qi = _model->getJoint(jname)->getJointPositionMinimal().value();
            robot_state.setJointPositions(jname, {qi}); // joint value is a simple scalar
        }
        else if(jtype == urdf::Joint::FLOATING) // joint value is actually a pose (3 + 4 values)
        {
            std::string parent_link = jmodel->parent_link_name;
            std::string child_link = jmodel->child_link_name;

            // transform from parent link to child link
            Eigen::Affine3d p_T_c;
            _model->getPose(child_link, parent_link, p_T_c);

            // transform from parent link to joint predecessor frame
            Eigen::Affine3d p_T_j;
            p_T_j.setIdentity();
            p_T_j.translation().x() = jmodel->parent_to_joint_origin_transform.position.x;
            p_T_j.translation().y() = jmodel->parent_to_joint_origin_transform.position.y;
            p_T_j.translation().z() = jmodel->parent_to_joint_origin_transform.position.z;

            Eigen::Quaterniond p_q_j(
                jmodel->parent_to_joint_origin_transform.rotation.w,
                jmodel->parent_to_joint_origin_transform.rotation.x,
                jmodel->parent_to_joint_origin_transform.rotation.y,
                jmodel->parent_to_joint_origin_transform.rotation.z
                               );

            p_T_j.linear() = p_q_j.toRotationMatrix();

            // joint transform
            Eigen::Affine3d Tj = p_T_j.inverse() * p_T_c;

            Eigen::Quaterniond Tj_rotation(Tj.linear());

            std::vector<double> jpos =
            {
                Tj.translation().x(),
                Tj.translation().y(),
                Tj.translation().z(),
                Tj_rotation.x(),
                Tj_rotation.y(),
                Tj_rotation.z(),
                Tj_rotation.w()
            };

            robot_state.setJointPositions(jname, jpos);
            robot_state.update();

        }
        else if(jtype == urdf::Joint::FIXED)
        {
            // do nothing
        }
        else
        {
            throw std::runtime_error("Unsupported joint type");
        }

    }

    _monitor->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_STATE);
}

bool PlanningSceneWrapper::checkCollisions() const
{
    planning_scene_monitor::LockedPlanningSceneRO ps(_monitor);

    collision_detection::CollisionRequest collision_request;

    collision_detection::CollisionResult collision_result;

    ps->checkCollision(collision_request, collision_result);

    return collision_result.collision;
}

bool PlanningSceneWrapper::checkSelfCollisions() const
{
    planning_scene_monitor::LockedPlanningSceneRO ps(_monitor);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    ps->checkSelfCollision(collision_request, collision_result);

    return collision_result.collision;
}

std::vector<std::string> PlanningSceneWrapper::getCollidingLinks() const
{
    planning_scene_monitor::LockedPlanningSceneRO ps(_monitor);

    std::vector<std::string> links;
    ps->getCollidingLinks(links);

    return links;
}

std::vector<XBot::Chain::ConstPtr> PlanningSceneWrapper::getCollidingChains() const
{
    std::vector<std::string> colliding_links = getCollidingLinks();
    std::vector<XBot::Chain::ConstPtr> colliding_chains;
    for (auto i:_srdf.getGroups())
    {
        auto link  = i.links_;
        for (auto j : link)
        {
            if (std::any_of(colliding_links.begin(), colliding_links.end(), [j](std::string k){ return k == j; }))
            {
               colliding_chains.push_back(_model->getChain(i.name_));
               goto cnt;
            }
        }
        cnt:;
    }
    return colliding_chains;
}


void PlanningSceneWrapper::applyPlanningScene(const moveit_msgs::msg::PlanningScene & scene)
{
    planning_scene_monitor::LockedPlanningSceneRW ps(_monitor);

    ps->usePlanningSceneMsg(scene);
}

bool PlanningSceneWrapper::getPlanningScene(moveit_msgs::srv::GetPlanningScene::Request::ConstSharedPtr req,
                                            moveit_msgs::srv::GetPlanningScene::Response::SharedPtr res)
{
    if (req->components.components & moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS)
    {
        _monitor->updateFrameTransforms();
    }

    planning_scene_monitor::LockedPlanningSceneRO ps(_monitor);

    moveit_msgs::msg::PlanningSceneComponents all_components;
    all_components.components = UINT_MAX;  // Return all scene components if nothing is specified.
    ps->getPlanningSceneMsg(res->scene, req->components.components ? req->components : all_components);

    return true;

}


} } }
