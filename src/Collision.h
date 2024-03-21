#ifndef COLLISION_H
#define COLLISION_H

#include <cartesian_interface/sdk/problem/Task.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>

#include <urdf/model.h>
#include <srdfdom/model.h>

#include "planning_scene/planning_scene_wrapper.h"
#include <moveit_msgs/ApplyPlanningScene.h>

#include <visualization_msgs/Marker.h>

using CollisionConstrSoT = OpenSoT::constraints::velocity::CollisionAvoidance;

namespace XBot { namespace Cartesian { namespace collision {

using WitnessPointVector = XBot::Collision::CollisionModel::WitnessPointVector;
using LinkPairVector = XBot::Collision::CollisionModel::LinkPairVector;

/**
 * @brief The CollisionTaskImpl class implements CartesIO's description
 * of a collision avoidance task or constraint
 */
class CollisionTaskImpl : public virtual ConstraintDescription,
        public TaskDescriptionImpl
{

public:

    CARTESIO_DECLARE_SMART_PTR(CollisionTaskImpl);

    /**
     * @brief The callback type to listen to world update events
     */
    typedef std::function<void(const moveit_msgs::PlanningSceneWorld&)> WorldUpdateCallback;

    /**
     * @brief CollisionTaskImpl constructor
     * This signature is required to register this class with CartesIO
     * (see cpp)
     */
    CollisionTaskImpl(YAML::Node node, Context::ConstPtr context);

    /**
     * @brief validates parameter correctness
     */
    bool validate() override;


    /* Getters for task parameters */

    /**
     * @brief bound scaling getter function; the bound scaling
     * is always smaller than one, its goal is to implement
     * early constraint activation by limiting the constraint
     * approach velocity
     */
    double getBoundScaling() const;

    /**
     * @brief distance threshold getter function; it represents
     * the minimum valid distance between collision objects
     */
    double getDistanceThreshold() const;

    /**
     * @brief get the maximum distance within which a collision
     * is actually taken into account by the constraint
     */
    double getDetectionThreshold() const;

    /**
     * @brief getter for the list of collision pairs that must be taken into
     * account by the constraint
     */
    std::list<std::pair<std::string, std::string>> getWhiteList() const;

    /**
     * @brief getter for the list of links that must be checked for collision
     * against the environment
     */
    std::list<std::string> getEnvironmentWhiteList() const;

    /**
     * @brief collision urdf used to override the default urdf collision
     * information
     */
    urdf::ModelConstSharedPtr getCollisionUrdf() const;

    /**
     * @brief collision srdf used to override the default urdf collision
     * information
     */
    srdf::ModelConstSharedPtr getCollisionSrdf() const;

    /**
     * @brief register a callback to be invoked whenever the world collision
     * model changes
     */
    void registerWorldUpdateCallback(WorldUpdateCallback f);

    /**
     * @brief worldUpdated must be called whenever the world collision
     * model changes; it is mostly for internal use, don't call it
     * unless you are sure!
     */
    void worldUpdated(const moveit_msgs::PlanningSceneWorld& psw);

    /**
     * @brief witnessPoints
     * @return
     */
    WitnessPointVector& witnessPoints();

    /**
     * @brief linkPairs
     * @return
     */
    LinkPairVector& linkPairs();

    /**
     * @brief distances
     * @return
     */
    std::vector<double>& distances();

private:

    std::list<std::pair<std::string, std::string>> _pairs;
    std::list<std::string> _env_links;
    double _bound_scaling;
    double _min_dist;
    double _detection_threshold;

    urdf::ModelConstSharedPtr _coll_urdf;
    srdf::ModelConstSharedPtr _coll_srdf;

    std::list<WorldUpdateCallback> _world_upd_cb;

    WitnessPointVector _wp;
    LinkPairVector _cpairs;
    std::vector<double> _dist;

};

/**
 * @brief The CollisionRos class implements the Ros API for
 * a collision task or constraint.
 * At the moment, it just provides world geometry updating
 * and monitoring tools via moveit's PlanningScene
 */
class CollisionRos : public ServerApi::TaskRos
{

public:

    CollisionRos(TaskDescription::Ptr task,
                 RosContext::Ptr context);

    void run(ros::Time time) override;

    void setVisualizeDistances(const bool flag);

private:

    bool apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request& req,
                                      moveit_msgs::ApplyPlanningScene::Response& res);

    CollisionTaskImpl::Ptr _ci_coll;

    ros::ServiceServer _world_upd_srv;

    std::unique_ptr<Collision::PlanningSceneWrapper> _ps;

    bool _visualize_distances;

    ros::Publisher _vis_pub;

    ros::Publisher _coll_pub;

};

/**
 * @brief The OpenSotCollisionConstraintAdapter class implements
 * the bridge between Cartesio and OpenSot, so that the latter
 * gets the information to construct, configure, and run the
 * collision avoidance constraint
 */
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

    bool addPrimitiveShape(std::string name,
                           shape_msgs::SolidPrimitive p,
                           Eigen::Affine3d w_T_p);

    CollisionTaskImpl::Ptr _ci_coll;
    Eigen::VectorXd _x;

};




}}}
#endif // COLLISION_H
