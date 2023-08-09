#ifndef PHY_PWORLD_H
#define PHY_PWORLD_H

#include <vector>
#include "pfgen.h"
#include "plinks.h"

namespace Phy
{
    class ParticleWorld
    {
    public:
        typedef std::vector<Particle*> Particles;
        typedef std::vector<ParticleContactGenerator*> ContactGenerators;
    protected:
        Particles particles;

        bool calculateIterations;

        ParticleForceRegistry registry;
        ParticleContactResolver resolver;
        ContactGenerators contactGenerators;
        ParticleContact* contacts;
        unsigned maxContacts;

    public:
        ParticleWorld(unsigned maxContacts, unsigned iterations=0);
        ~ParticleWorld();

        unsigned generateContacts();
        void integrate(real duration);
        void runPhysics(real duration);

        void startFrame();

        Particles& getParticles();
        ContactGenerators& getContactGenerators();
        ParticleForceRegistry& getForceRegistry();

    };

    class GroundContacts : public ParticleContactGenerator
    {
        ParticleWorld::Particles* particles;
    public:
        void init(ParticleWorld::Particles* particles);
        virtual unsigned addContact(ParticleContact* contact, unsigned limit) const;
    };
}

#endif
