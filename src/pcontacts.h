#ifndef PHY_PCONTACTS_H
#define PHY_PCONTACTS_H

#include "particle.h"

namespace Phy
{

    class ParticleContact
    {
    public:
        Particle* particle[2];
        real restitution;
        real penetration;
        Vector3 contactNormal;
        Vector3 particleMovement[2];
        void resolve(real duration);
        real calculateSeparatingVelocity() const;
    private:
        void resolveVelocity(real duration);
        void resolveInterpenetration(real duration);
    };

    class ParticleContactResolver
    {
    protected:
        unsigned iterations;
        unsigned iterationsUsed;
    public:
        ParticleContactResolver(unsigned iterations);
        void setIterations(unsigned iterations);

        // resolves a set of particles contacts for both penetatrion and velocity
        void resolveContacts(ParticleContact* contactArray,
                             unsigned numContacts, real duration);
    };

    class ParticleContactGenerator
    {
    public:
        virtual unsigned addContact(ParticleContact* contact,
                                    unsigned limit) const = 0;
    };

}

#endif
