#ifndef PHY_PLINKS_H
#define PHY_PLINKS_H

#include "pcontacts.h"

namespace Phy
{
    class ParticleLink : public ParticleContactGenerator
    {
    public:
        Particle* particle[2];
    protected:
        real currentLength() const;
    public:
        /**
         * Geneates the contacts to keep this link from being
         * violated. This class can only ever generate a single
         * contact, so the pointer can be a pointer to a single
         * element, the limit parameter is assumed to be at least one
         * (zero isn't valid) and the return value is either 0, if the
         * cable wasn't over-extended, or one if a contact was needed.
         *
         * NB: This method is declared in the same way (as pure
         * virtual) in the parent class, but is replicated here for
         * documentation purposes.
         */
        virtual unsigned addContact(ParticleContact *contact,
                                    unsigned limit) const = 0;
    };

    class ParticleCable : public ParticleLink
    {
    public:
        real maxLength;
        real restitution;
    public:
        virtual unsigned addContact(ParticleContact *contact,
                                    unsigned limit) const;
    };

    class ParticleRod : public ParticleLink
    {
    public:
        real length;
    public:
        virtual unsigned addContact(ParticleContact *contact,
                                    unsigned limit) const;
    };


    class ParticleConstraint : public ParticleContactGenerator
    {
    public:
        Particle* particle;
        Vector3 anchor;
    protected:
        real currentLength() const;
    public:
        virtual unsigned addContact(ParticleContact *contact,
                                    unsigned limit) const = 0;
    };

    class ParticleCableConstraint : public ParticleConstraint
    {
    public:
        real maxLength;
        real restitution;
    public:
        virtual unsigned addContact(ParticleContact *contact,
                                    unsigned limit) const;
    };

    class ParticleRodConstraint : public ParticleConstraint
    {
    public:
        real length;
    public:
        virtual unsigned addContact(ParticleContact *contact,
                                    unsigned limit) const;
    };

};

#endif
