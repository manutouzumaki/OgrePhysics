#ifndef PHY_PARTICLE_H
#define PHY_PARTICLE_H

#include "core.h"

namespace Phy
{

    class Particle
    {
    public:
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        Vector3 forceAccum;
        real damping;
    protected:
        real inverseMass;
    public:

        void integrate(real duration);
        void clearAccumulator();

        void addForce(const Vector3& force);

        void setInverseMass(const real inverseMass);
        real getInverseMass() const;
        void setMass(const real mass);
        real getMass() const;
        bool hasFiniteMass() const;
    };



}

#endif
