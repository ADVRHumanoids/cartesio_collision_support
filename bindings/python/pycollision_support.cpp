#include <xbot2_interface/collision.h>
#include <cartesio_collision_support/Collision.h>
#include "../../src/Collision.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

using XBot::Cartesian::collision::CollisionTask;
using XBot::Cartesian::collision::CollisionTaskImpl;
namespace py = pybind11;
using rvp = py::return_value_policy;


PYBIND11_MODULE(pycollision_support, m) {

    py::class_<CollisionTask,
               std::shared_ptr<CollisionTask>>(m, "CollisionTask")
        .def("getCollisionModel", &CollisionTask::getCollisionModel, rvp::reference_internal)
        .def("collisionModelUpdated", &CollisionTask::collisionModelUpdated)
        ;

    py::class_<CollisionTaskImpl, CollisionTask,
               std::shared_ptr<CollisionTaskImpl>>(m, "CollisionTaskImpl")
        ;

}
