#ifndef COLLISION_H
#define COLLISION_H

#include <cartesian_interface/sdk/problem/Task.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <OpenSoT/tasks/velocity/CollisionRepulsiveField.h>
#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>

#include <urdf/model.h>
#include <srdfdom/model.h>

using CollisionTaskSoT = OpenSoT::tasks::velocity::CollisionRepulsiveField;
using CollisionConstrSoT = OpenSoT::constraints::velocity::SelfCollisionAvoidance;


namespace XBot { namespace Cartesian { namespace collision {

class CollisionTaskImpl : public virtual ConstraintDescription,
                          public TaskDescriptionImpl
{

public:

    CARTESIO_DECLARE_SMART_PTR(CollisionTaskImpl);

    CollisionTaskImpl(YAML::Node node, Context::ConstPtr context);

    bool validate() override;

    double getBoundScaling() const;

    double getDistanceThreshold() const;

    std::list<std::pair<std::string, std::string>> getWhiteList() const;

    urdf::ModelConstSharedPtr getCollisionUrdf() const;
    srdf::ModelConstSharedPtr getCollisionSrdf() const;

private:

    std::list<std::pair<std::string, std::string>> _pairs;
    double _bound_scaling;
    double _min_dist;

    urdf::ModelConstSharedPtr _coll_urdf;
    srdf::ModelConstSharedPtr _coll_srdf;

};

class OpenSotCollisionAdapter :
        public OpenSotTaskAdapter
{

public:

    OpenSotCollisionAdapter(TaskDescription::Ptr ci_task,
                            Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual TaskPtr constructTask() override;

    virtual void update(double time, double period) override;

    virtual void processSolution(const Eigen::VectorXd& solution) override;

protected:

    CollisionTaskSoT::Ptr _opensot_coll;

private:

    CollisionTaskImpl::Ptr _ci_coll;
    Eigen::VectorXd _x;

};

class OpenSotCollisionConstraintAdapter :
        public OpenSotConstraintAdapter
{

public:

    OpenSotCollisionConstraintAdapter(ConstraintDescription::Ptr ci_task,
                                      Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual ConstraintPtr constructConstraint() override;

    virtual void update(double time, double period) override;

    virtual void processSolution(const Eigen::VectorXd& solution) override;

protected:

    CollisionConstrSoT::Ptr _opensot_coll;

private:

    CollisionTaskImpl::Ptr _ci_coll;
    Eigen::VectorXd _x;

};

}}}
#endif // COLLISION_H
