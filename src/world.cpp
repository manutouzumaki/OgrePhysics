#include "world.h"
    
namespace Phy
{
    void World::startFrame()
    {
        for(RigidBodies::iterator b = bodies.begin();
            b != bodies.end();
            b++)
        {
            b->clearAccumulators();
            b->calculateDerivedData();
        }
    }

    void World::integrate(real duration)
    {
        for(RigidBodies::iterator b = bodies.begin();
            b != bodies.end();
            b++)
        {
            b->integrate(duration);
        }
    }

    void World::runPhysics(real duration)
    {
        // First apply the force generators
        registry.updateForces(duration);

        // then integrate the objects
        integrate(duration);
    }
}
