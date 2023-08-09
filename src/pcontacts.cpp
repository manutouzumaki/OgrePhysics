#include "pcontacts.h"

namespace Phy
{
    void ParticleContact::resolve(real duration)
    {
        resolveVelocity(duration);
        resolveInterpenetration(duration);
    }

    real ParticleContact::calculateSeparatingVelocity() const
    {
        Vector3 relativeVelocity = particle[0]->velocity;
        if(particle[1]) relativeVelocity -= particle[1]->velocity;
        return relativeVelocity * contactNormal;
        
    }

    void ParticleContact::resolveVelocity(real duration)
    {
        // Find the velocity in the direciton of the contact.
        real separatingVelocity = calculateSeparatingVelocity();

        // Check if it needs to be resolved
        if(separatingVelocity > 0)
        {
            // The contact is either separating, or stationary
            // no impulse is required
            return;
        }

        // Calculate the new separating velocity
        real newSepVelocity = -separatingVelocity * restitution;

        // Check the velocity build up due to acceleration only
        Vector3 accCausedVelocity = particle[0]->acceleration;
        if(particle[1]) accCausedVelocity -= particle[1]->acceleration;
        real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

        // if we got closing velocity due to acceleration buildup remove it
        if(accCausedSepVelocity < 0)
        {
            newSepVelocity += restitution * accCausedSepVelocity;
            if(newSepVelocity < 0) newSepVelocity = 0;
        }

        real deltaVelocity = newSepVelocity - separatingVelocity;

        // We apply the change in velocity to each object in proportion to their inverse mass
        real totalInverseMass = particle[0]->getInverseMass();
        if(particle[1]) totalInverseMass += particle[1]->getInverseMass();

        // if all particles have infinite mass, then impulses have no effect
        if(totalInverseMass <= 0) return;

        // Calculate the impulse to apply
        real impulse = deltaVelocity / totalInverseMass;

        // Find the amount of impulse per unit of inverse mass
        Vector3 impulsesPerIMass = contactNormal * impulse;

        // Apply impulse
        particle[0]->velocity = particle[0]->velocity + impulsesPerIMass * particle[0]->getInverseMass();
        if(particle[1])
            particle[1]->velocity = particle[1]->velocity + impulsesPerIMass * -particle[1]->getInverseMass();
    }


    void ParticleContact::resolveInterpenetration(real duration)
    {
        if(penetration <= 0) return;

        real totalInverseMass = particle[0]->getInverseMass();
        if(particle[1]) totalInverseMass += particle[1]->getInverseMass();

        // if all particles have infinite mass then do nothing
        if(totalInverseMass <= 0) return;

        // Find the amount of penetration resolution per unit of inverse mass
        Vector3 movePerIMass = contactNormal * (penetration / totalInverseMass);

        // calculate the movement amount
        particleMovement[0] = movePerIMass * particle[0]->getInverseMass();
        if(particle[1])
        {
            particleMovement[1] = movePerIMass * -particle[1]->getInverseMass();
        }
        else
        {
            particleMovement[1].clear();
        }

        // Apply the penetatrion resolution
        particle[0]->position = particle[0]->position + particleMovement[0];
        if(particle[1])
        {
            particle[1]->position = particle[1]->position + particleMovement[1];
        }

    }



    ParticleContactResolver::ParticleContactResolver(unsigned iterations)
        : iterations(iterations), iterationsUsed(0)
    {

    }

    void ParticleContactResolver::setIterations(unsigned iterations)
    {
        ParticleContactResolver::iterations = iterations;
    }

    void ParticleContactResolver::resolveContacts(ParticleContact* contactArray,
                         unsigned numContacts, real duration)
    {
        unsigned i;
        iterationsUsed = 0;
        while(iterationsUsed < iterations)
        {
            // Find the contact with the largest closing velocity
            real max = REAL_MAX;
            unsigned maxIndex = numContacts;
            for(i = 0; i < numContacts; i++)
            {
                real sepVel = contactArray[i].calculateSeparatingVelocity();
                if(sepVel < max &&
                   (sepVel < 0 || contactArray[i].penetration > 0))
                {
                    max = sepVel;
                    maxIndex = i;
                }
            }

            // Do we have anything worth resolving?
            if(maxIndex == numContacts) break;

            // resolve this contact
            contactArray[maxIndex].resolve(duration);

            // Update the interpenetrations for all particles
            Vector3 *move = contactArray[maxIndex].particleMovement;
            for (i = 0; i < numContacts; i++)
            {
                if (contactArray[i].particle[0] == contactArray[maxIndex].particle[0])
                {
                    contactArray[i].penetration -= move[0] * contactArray[i].contactNormal;
                }
                else if (contactArray[i].particle[0] == contactArray[maxIndex].particle[1])
                {
                    contactArray[i].penetration -= move[1] * contactArray[i].contactNormal;
                }
                if (contactArray[i].particle[1])
                {
                    if (contactArray[i].particle[1] == contactArray[maxIndex].particle[0])
                    {
                        contactArray[i].penetration += move[0] * contactArray[i].contactNormal;
                    }
                    else if (contactArray[i].particle[1] == contactArray[maxIndex].particle[1])
                    {
                        contactArray[i].penetration += move[1] * contactArray[i].contactNormal;
                    }
                }
            }

            iterationsUsed++;
        }
    }


}
