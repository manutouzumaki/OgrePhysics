#include "particle.h"

#define Assert(Expression) if(!(Expression)) {*(int *)0 = 0;}

namespace Phy
{
    void Particle::integrate(real duration)
    {
        if(inverseMass <= 0.0f) return;

        Assert(duration > 0.0);

        // Update linear position
        position.addScaledVector(velocity, duration);

        // Work out the acceleration from the force
        Vector3 resultAcc = acceleration;
        resultAcc.addScaledVector(forceAccum, inverseMass);
        // Update linear velocity from the acceleration
        velocity.addScaledVector(resultAcc, duration);

        // Impose Drag
        velocity *= real_pow(damping, duration);

        clearAccumulator();
    }

    void Particle::clearAccumulator()
    {
        forceAccum.clear();
    }

    void Particle::addForce(const Vector3& force)
    {
        forceAccum += force;
    }

    void Particle::setInverseMass(const real inverseMass)
    {
        Particle::inverseMass = inverseMass;
    }

    real Particle::getInverseMass() const
    {
        return inverseMass;
    }

    void Particle::setMass(const real mass)
    {
        Assert(mass != 0);
        Particle::inverseMass = ((real)1.0)/mass;
    }

    real Particle::getMass() const
    {
        if(inverseMass == 0)
        {
            return REAL_MAX;
        }
        else
        {
            return ((real)1.0)/inverseMass;
        }
    }


    bool Particle::hasFiniteMass() const
    {
        return inverseMass >= 0.0f;

    }
}
