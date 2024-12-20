#ifndef CARTESIO_COLLISION_SUPPOTRT_COLLISION_H
#define CARTESIO_COLLISION_SUPPOTRT_COLLISION_H


#include <cartesian_interface/sdk/problem/Task.h>
#include <xbot2_interface/collision.h>

namespace XBot { namespace Cartesian { namespace collision {

class CollisionTask : public virtual TaskDescription
{

public:

    /**
     * @brief returns the internal collision model
     * NOTE: if you modify the collision model, you are required to call collisionModelUpdated()
     * right after
     */
    virtual XBot::Collision::CollisionModel& getCollisionModel() = 0;

    /**
     * @brief collisionModelUpdated
     */
    virtual void collisionModelUpdated() = 0;

};

}}}

#endif // COLLISION_H
