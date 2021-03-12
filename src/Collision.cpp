#include "Collision.h"
#include <boost/make_shared.hpp>

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

urdf::ModelConstSharedPtr CollisionTaskImpl::getCollisionUrdf() const
{
    return _coll_urdf;
}

srdf::ModelConstSharedPtr CollisionTaskImpl::getCollisionSrdf() const
{
    return _coll_srdf;
}

OpenSotCollisionAdapter::OpenSotCollisionAdapter(TaskDescription::Ptr ci_task,
                                                 Context::ConstPtr context):
    OpenSotTaskAdapter(ci_task, context)
{
    _ci_coll = std::dynamic_pointer_cast<CollisionTaskImpl>(ci_task);
    if(!_ci_coll) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'CollisionTask'");
}

OpenSoT::OptvarHelper::VariableVector OpenSotCollisionAdapter::getRequiredVariables() const
{
    return {};
}

TaskPtr OpenSotCollisionAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_coll = boost::make_shared<CollisionTaskSoT>(q,
                                                         *_model,
                                                         _ci_coll->getSize());

    _opensot_coll->setWhiteList(_ci_coll->getWhiteList());

    return _opensot_coll;
}

void OpenSotCollisionAdapter::update(double time, double period)
{
    OpenSotTaskAdapter::update(time, period);
}

void OpenSotCollisionAdapter::processSolution(const Eigen::VectorXd &solution)
{

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

    return _opensot_coll;
}

void OpenSotCollisionConstraintAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);
}

void OpenSotCollisionConstraintAdapter::processSolution(const Eigen::VectorXd &solution)
{

}

CARTESIO_REGISTER_TASK_PLUGIN(CollisionTaskImpl, CollisionConstraint)
CARTESIO_REGISTER_TASK_PLUGIN(CollisionTaskImpl, CollisionTask)
CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotCollisionAdapter, CollisionTask)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotCollisionConstraintAdapter, CollisionConstraint)
