#include "Collision.h"
#include <boost/make_shared.hpp>
#include <xbot2_interface/common/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <cartesio_collision_support/CollisionState.h>

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::collision;

const int default_max_pairs = 50;

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
    _detection_threshold(-1),
    _min_dist(0.001)  // note: smth > 0 to avoid singular min distance segment
{

    if(auto n = node["pairs"])
    {
        for(auto p : n)
        {
            _pairs.insert(p.as<std::pair<std::string, std::string>>());
        }
    }

    if(auto n = node["env_collision_links"])
    {
        for(auto p : n)
        {
            _env_links.insert(p.as<std::string>());
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

    if(auto n = node["detection_threshold"])
    {
        _detection_threshold = n.as<double>();
    }

    if(_detection_threshold > 0 &&
        _detection_threshold < _min_dist)
    {
        throw std::invalid_argument("detection_threshold must be (way) higher than distance_threshold");
    }

    if(auto n = node["collision_urdf_path"])
    {
        // parse path via shell
        auto urdf_path = XBot::Utils::echo(n.as<std::string>());

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
        auto srdf_path = XBot::Utils::echo(n.as<std::string>());

        // construct model shared ptr
        auto srdf_mdl = new srdf::Model;
        _coll_srdf.reset(srdf_mdl);

        // get urdf either from collision or from modelinterface
        const urdf::ModelInterface* urdf = _coll_urdf.get();

        if(!urdf)
        {
            urdf = context->model()->getUrdf().get();
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

double CollisionTaskImpl::getDetectionThreshold() const
{
    return _detection_threshold;
}

std::set<std::pair<std::string, std::string> > CollisionTaskImpl::getWhiteList() const
{
    return _pairs;
}

std::set<std::string> CollisionTaskImpl::getEnvironmentWhiteList() const
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

WitnessPointVector &CollisionTaskImpl::witnessPoints()
{
    return _wp;
}

LinkPairVector &CollisionTaskImpl::linkPairs()
{
    return _cpairs;
}

std::vector<double> &CollisionTaskImpl::distances()
{
    return _dist;
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

    _opensot_coll = SotUtils::make_shared<CollisionConstrSoT>(
                        *_model,
                        _ci_coll->getSize(),
                        _ci_coll->getCollisionUrdf(),
                        _ci_coll->getCollisionSrdf()
                        );

    // set parameters
    _opensot_coll->setBoundScaling(_ci_coll->getBoundScaling());
    _opensot_coll->setLinkPairThreshold(_ci_coll->getDistanceThreshold());
    _opensot_coll->setDetectionThreshold(_ci_coll->getDetectionThreshold());  // hardcoded!

    // set whitelist if available
    auto whitelist = _ci_coll->getWhiteList();
    if(!whitelist.empty())
    {
        std::set<std::pair<std::string, std::string>> whitelist_set(whitelist.begin(), whitelist.end());
        _opensot_coll->setCollisionList(whitelist_set);
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
        for(const auto& co : psw.collision_objects)
        {
            Eigen::Affine3d w_T_co;

            tf::poseMsgToEigen(co.pose, w_T_co);

            if(co.operation == co.ADD)
            {
                for(int i = 0; i < co.primitives.size(); i++)
                {
                    Eigen::Affine3d co_T_p;

                    tf::poseMsgToEigen(co.primitive_poses[i], co_T_p);

                    addPrimitiveShape(co.id + "__" + std::to_string(i),
                                      co.primitives[i],
                                      w_T_co * co_T_p);
                }
            }
        }

        _opensot_coll->collisionModelUpdated();
    };

    _ci_coll->registerWorldUpdateCallback(on_world_upd);

    return _opensot_coll;
}

void OpenSotCollisionConstraintAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);
}

void OpenSotCollisionConstraintAdapter::processSolution(const Eigen::VectorXd &solution)
{
    _opensot_coll->getOrderedWitnessPointVector(_ci_coll->witnessPoints());
    _opensot_coll->getOrderedLinkPairVector(_ci_coll->linkPairs());
    _opensot_coll->getOrderedDistanceVector(_ci_coll->distances());
}

bool OpenSotCollisionConstraintAdapter::addPrimitiveShape(std::string name,
                                                          shape_msgs::SolidPrimitive p,
                                                          Eigen::Affine3d w_T_p)
{
    using Shape = XBot::Collision::Shape;

    Shape::Variant shape;

    if(p.type == p.BOX)
    {
        Shape::Box box;
        box.size << p.dimensions[p.BOX_X], p.dimensions[p.BOX_Y], p.dimensions[p.BOX_Z];
        shape = box;
    }
    else if(p.type == p.SPHERE)
    {
        Shape::Sphere sphere;
        sphere.radius = p.dimensions[p.SPHERE_RADIUS];
        shape = sphere;
    }
    else if(p.type == p.CYLINDER)
    {
        Shape::Cylinder cylinder;
        cylinder.radius = p.dimensions[p.CYLINDER_RADIUS];
        cylinder.length = p.dimensions[p.CYLINDER_HEIGHT];
        shape = cylinder;
    }
    else
    {
        Logger::error("unsupported shape type");
        return false;
    }

    return _opensot_coll->getCollisionModel().addCollisionShape(name, "world", shape, w_T_p);
}

CollisionRos::CollisionRos(TaskDescription::Ptr task,
                           RosContext::Ptr context):
    TaskRos(task, context)
{
    _ci_coll = std::dynamic_pointer_cast<CollisionTaskImpl>(task);

    if(!_ci_coll) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'CollisionTask'");

    auto nh = ros::NodeHandle(context->nh().getNamespace() + "/" + task->getName());

    _ps = std::make_unique<Collision::PlanningSceneWrapper>(_ci_coll->getModel(),
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

    _coll_pub = nh.advertise<cartesio_collision_support::CollisionState>("collision_state", 1);

}

void CollisionRos::setVisualizeDistances(const bool flag)
{
    _visualize_distances = flag;
}

bool CollisionRos::apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request &req,
                                                moveit_msgs::ApplyPlanningScene::Response &res)
{
    // for visualization purposes (e.g. rviz/PlanningScene)
    _ps->applyPlanningScene(req.scene);

    // notify collision avoidance constraint that
    // world geometry has changed
    _ci_coll->worldUpdated(req.scene.world);

    res.success = true;

    return true;
}

void XBot::Cartesian::collision::CollisionRos::run(ros::Time time)
{
    // let base class do its magic
    TaskRos::run(time);

    _ps->update();

    const auto& wpv = _ci_coll->witnessPoints();

    auto k2p = [](const Eigen::Vector3d& eig)
    {
        geometry_msgs::Point p;
        p.x = eig[0]; p.y = eig[1]; p.z = eig[2];
        return p;
    };

    if(_visualize_distances)
    {

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
        marker.color.r = 1.;
        marker.color.g = 0.;
        marker.color.b = 0.;
        marker.color.a = 1.;
        marker.scale.x = 0.005;
        marker.scale.y = 0.;
        marker.scale.z = 0.;

        for(const auto& [p1, p2] : wpv)
        {
            // closest point on first link
            marker.points.push_back(k2p(p1));

            // closest point on second link
            marker.points.push_back(k2p(p2));
        }
        _vis_pub.publish(marker);
    }

    // publish collision state
    cartesio_collision_support::CollisionState msg;

    const auto& lp = _ci_coll->linkPairs();
    const auto& dist = _ci_coll->distances();

    for(int i = 0; i < lp.size(); i++)
    {
        msg.link_1.push_back(lp[i].first);
        msg.link_2.push_back(lp[i].second);
        msg.wp_1.push_back(k2p(wpv[i].first));
        msg.wp_2.push_back(k2p(wpv[i].second));
        msg.distance.push_back(dist[i]);
    }

    _coll_pub.publish(msg);
}


CARTESIO_REGISTER_TASK_PLUGIN(CollisionTaskImpl, CollisionConstraint)
CARTESIO_REGISTER_TASK_PLUGIN(CollisionTaskImpl, CollisionTask)
CARTESIO_REGISTER_ROS_API_PLUGIN(CollisionRos, CollisionConstraint)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotCollisionConstraintAdapter, CollisionConstraint)


