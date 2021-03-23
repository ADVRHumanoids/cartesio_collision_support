#include "Collision.h"
#include <boost/make_shared.hpp>
#include <OpenSoT/utils/collision_utils.h>


using namespace XBot::Cartesian;
using namespace XBot::Cartesian::collision;

const int default_max_pairs = 15;

namespace
{
int get_size(YAML::Node node)
{
    if(auto n = node["max_pairs"])
    {
        return n.as<int>();
    }

    if(auto n = node["pairs"])
    {
        return n.size();
    }

    return default_max_pairs;
}
}

CollisionTaskImpl::CollisionTaskImpl(YAML::Node node,
                                     Context::ConstPtr context):
    TaskDescriptionImpl(node, context, "collision_avoidance", get_size(node)),
    _bound_scaling(1.0),
    _min_dist(0.001)  // note: smth > 0 to avoid singular min distance segment
{

    if(auto n = node["pairs"])
    {
        for(auto p : n)
        {
            _pairs.push_back(p.as<std::pair<std::string, std::string>>());
        }
    }

    if(auto n = node["env_collision_links"])
    {
        for(auto p : n)
        {
            _env_links.push_back(p.as<std::string>());
        }
    }

    if(auto n = node["bound_scaling"])
    {
        _bound_scaling = n.as<double>();
    }

    if(auto n = node["distance_threshold"])
    {
        _min_dist = n.as<double>();
    }

    if(auto n = node["collision_urdf_path"])
    {
        // parse path via shell
        auto urdf_path = XBot::Utils::computeAbsolutePathShell(n.as<std::string>());

        // construct model shared ptr
        auto urdf_mdl = new urdf::Model;
        _coll_urdf.reset(urdf_mdl);

        // if could not init, destroy it
        if(!urdf_mdl->initFile(urdf_path))
        {
            Logger::error("could not load collision urdf from file '%s'\n",
                          urdf_path.c_str());

            _coll_urdf.reset();
        }
    }

    if(auto n = node["collision_srdf_path"])
    {
        // parse path via shell
        auto srdf_path = XBot::Utils::computeAbsolutePathShell(n.as<std::string>());

        // construct model shared ptr
        auto srdf_mdl = new srdf::Model;
        _coll_srdf.reset(srdf_mdl);

        // get urdf either from collision or from modelinterface
        const urdf::ModelInterface* urdf = _coll_urdf.get();

        if(!urdf)
        {
            urdf = &(context->model()->getUrdf());
        }

        // if cannot init, destroy
        if(!srdf_mdl->initFile(*urdf, srdf_path))
        {
            Logger::error("could not load collision srdf from file '%s'\n",
                          srdf_path.c_str());

            _coll_srdf.reset();
        }
    }


}

void CollisionTaskImpl::setLinkPairDistances(const std::list<LinkPairDistance>& distance_list)
{
    _distance_list = distance_list;
}

const std::list<LinkPairDistance>& CollisionTaskImpl::getLinkPairDistances()
{
    return _distance_list;
}

bool CollisionTaskImpl::validate()
{
    return _bound_scaling <= 1.0 &&
            _bound_scaling > 0 &&
            _min_dist >= 0.0;
}

double CollisionTaskImpl::getBoundScaling() const
{
    return _bound_scaling;
}

double CollisionTaskImpl::getDistanceThreshold() const
{
    return _min_dist;
}

std::list<std::pair<std::string, std::string> > CollisionTaskImpl::getWhiteList() const
{
    return _pairs;
}

std::list<std::string> CollisionTaskImpl::getEnvironmentWhiteList() const
{
    return _env_links;
}

urdf::ModelConstSharedPtr CollisionTaskImpl::getCollisionUrdf() const
{
    return _coll_urdf;
}

srdf::ModelConstSharedPtr CollisionTaskImpl::getCollisionSrdf() const
{
    return _coll_srdf;
}

void CollisionTaskImpl::registerWorldUpdateCallback(WorldUpdateCallback f)
{
    _world_upd_cb.push_back(f);
}

void CollisionTaskImpl::worldUpdated(const moveit_msgs::PlanningSceneWorld& psw)
{
    for(auto& fn : _world_upd_cb)
    {
        fn(psw);
    }
}




OpenSotCollisionConstraintAdapter::OpenSotCollisionConstraintAdapter(ConstraintDescription::Ptr ci_task,
                                                                     Context::ConstPtr context):
    OpenSotConstraintAdapter(ci_task, context)
{
    _ci_coll = std::dynamic_pointer_cast<CollisionTaskImpl>(ci_task);
    if(!_ci_coll) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'CollisionTask'");
}

OpenSoT::OptvarHelper::VariableVector OpenSotCollisionConstraintAdapter::getRequiredVariables() const
{
    return {};
}

ConstraintPtr OpenSotCollisionConstraintAdapter::constructConstraint()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_coll = boost::make_shared<CollisionConstrSoT>(
                        q,
                        *_model,
                        _ci_coll->getSize(),
                        _ci_coll->getCollisionUrdf(),
                        _ci_coll->getCollisionSrdf()
                        );

    // set parameters
    _opensot_coll->setBoundScaling(_ci_coll->getBoundScaling());
    _opensot_coll->setLinkPairThreshold(_ci_coll->getDistanceThreshold());
    _opensot_coll->setDetectionThreshold(0.05);  // hardcoded!

    // set whitelist if available
    auto whitelist = _ci_coll->getWhiteList();
    if(!whitelist.empty())
    {
        _opensot_coll->setCollisionWhiteList(whitelist);
    }

    // set link-env collisions
    auto env_whitelist = _ci_coll->getEnvironmentWhiteList();
    if(!env_whitelist.empty())
    {
        _opensot_coll->setLinksVsEnvironment(env_whitelist);
    }

    // register world update function
    auto on_world_upd = [this](const moveit_msgs::PlanningSceneWorld& psw)
    {
        _opensot_coll->setWorldCollisions(psw);
    };

    _ci_coll->registerWorldUpdateCallback(on_world_upd);

    return _opensot_coll;
}

void OpenSotCollisionConstraintAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);

    _ci_coll->setLinkPairDistances(_opensot_coll->getLinkPairDistances());
}

void OpenSotCollisionConstraintAdapter::processSolution(const Eigen::VectorXd &solution)
{

}



CollisionRos::CollisionRos(TaskDescription::Ptr task,
                           RosContext::Ptr context):
    TaskRos(task, context)
{
    _ci_coll = std::dynamic_pointer_cast<CollisionTaskImpl>(task);

    if(!_ci_coll) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'CollisionTask'");

    auto nh = ros::NodeHandle(context->nh().getNamespace() + "/" + task->getName());

    _ps = std::make_unique<Planning::PlanningSceneWrapper>(_ci_coll->getModel(),
                                                           _ci_coll->getCollisionUrdf(),
                                                           _ci_coll->getCollisionSrdf(),
                                                           nh);
    _ps->startGetPlanningSceneServer();
    _ps->startMonitor();


    _visualize_distances = nh.param("visulize_distances", true);


    _world_upd_srv = nh.advertiseService("apply_planning_scene",
                                         &CollisionRos::apply_planning_scene_service,
                                         this);

    registerType("Collision");

    _vis_pub = nh.advertise<visualization_msgs::Marker>( "collision_distances", 0 );

}

void CollisionRos::setVisualizeDistances(const bool flag)
{
    _visualize_distances = flag;
}

bool CollisionRos::apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request &req,
                                                moveit_msgs::ApplyPlanningScene::Response &res)
{
    _ps->applyPlanningScene(req.scene);

    _ci_coll->worldUpdated(req.scene.world);

    res.success = true;

    return true;
}

void XBot::Cartesian::collision::CollisionRos::run(ros::Time time)
{
    _ps->update();
    if(_visualize_distances)
    {
        std::list<LinkPairDistance> distance_list = _ci_coll->getLinkPairDistances();

        visualization_msgs::Marker marker;
        marker.header.frame_id = "ci/world";
        marker.header.stamp = ros::Time().now();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.;
        marker.pose.position.y = 0.;
        marker.pose.position.z = 0.;
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.;
        marker.pose.orientation.z = 0.;
        marker.pose.orientation.w = 1.;
        marker.color.r = 0.;
        marker.color.g = 1.;
        marker.color.b = 0.;
        marker.color.a = 1.;
        marker.scale.x = 0.005;
        marker.scale.y = 0.;
        marker.scale.z = 0.;

        for(const auto& data : distance_list)
        {
            auto k2p = [](const KDL::Vector &k)->geometry_msgs::Point{
                geometry_msgs::Point p;
                p.x = k[0]; p.y = k[1]; p.z = k[2];
                return p;
            };



            // closest point on first link
            marker.points.push_back(k2p(data.getClosestPoints().first.p));
            // closest point on second link
            marker.points.push_back(k2p(data.getClosestPoints().second.p));
        }
        _vis_pub.publish(marker);
    }
}


CARTESIO_REGISTER_TASK_PLUGIN(CollisionTaskImpl, CollisionConstraint)
CARTESIO_REGISTER_TASK_PLUGIN(CollisionTaskImpl, CollisionTask)
CARTESIO_REGISTER_ROS_API_PLUGIN(CollisionRos, CollisionConstraint)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotCollisionConstraintAdapter, CollisionConstraint)


