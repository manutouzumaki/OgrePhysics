#include "pfgen.h"

namespace Phy
{
    ParticleGravity::ParticleGravity(const Vector3& gravity)
        : gravity(gravity) {}
    

    void ParticleGravity::updateForce(Particle* particle, real duration)
    {
        if(!particle->hasFiniteMass()) return;
        particle->addForce(gravity * particle->getMass());
    }


    ParticleDrag::ParticleDrag(real k1, real k2)
        : k1(k1), k2(k2) {}

    void ParticleDrag::updateForce(Particle* particle, real duration)
    {
        Vector3 force = particle->velocity;
        real dragCoeff = force.magnitude();
        if(dragCoeff > 0) 
        {
            int StopHEre = 0;
        }
        dragCoeff = (k1 * dragCoeff) + (k2 * (dragCoeff * dragCoeff));
        
        force.normalize();
        force *= -dragCoeff;
        particle->addForce(force);
    }

    ParticleSpring::ParticleSpring(Particle* other, real springConstant, real restLength)
        : other(other), springConstant(springConstant), restLength(restLength) {}

    void ParticleSpring::updateForce(Particle* particle, real duration)
    {
        Vector3 force = particle->position;
        force -= other->position;

        real magnitude = force.magnitude();
        if(magnitude <= restLength) return;
        
#if 1
        magnitude = real_abs(magnitude - restLength);
        magnitude *= springConstant;
#else
        magnitude = springConstant * (magnitude - restLength);
#endif

        force.normalize();
        force *= -magnitude;
        particle->addForce(force);
    }

    void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator* fg)
    {
        ParticleForceRegistry::ParticleForceRegistration registration;
        registration.particle = particle;
        registration.fg = fg;
        registrations.push_back(registration);
    }

    void ParticleForceRegistry::remove(Particle* particle, ParticleForceGenerator* fg)
    {
        // TODO: ...
    }

    void ParticleForceRegistry::clear()
    {
        registrations.clear();
    }

    void ParticleForceRegistry::updateForces(real duration)
    {
        Registry::iterator i = registrations.begin();
        for(; i != registrations.end(); i++)
        {
            i->fg->updateForce(i->particle, duration);
        }
    }

}
