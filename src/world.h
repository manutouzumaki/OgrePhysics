#ifndef PHY_WORLD_H
#define PHY_WORLD_H

#include "body.h"
#include "fgen.h"

#include <vector>

namespace Phy
{
    class World
    {
    public:
        typedef std::vector<RigidBody> RigidBodies;
    protected:
        RigidBodies bodies;
        ForceRegistry registry;
    
    public:
        void startFrame();
        void integrate(real duration);
        void runPhysics(real duration);
    };

}

#endif
