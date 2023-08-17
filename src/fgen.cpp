#include "fgen.h"

namespace Phy
{
    void ForceRegistry::add(RigidBody *body, ForceGenerator *fg)
    {
        ForceRegistry::ForceRegistration registration;
        registration.body = body;
        registration.fg = fg;
        registrations.push_back(registration);
    }

    void ForceRegistry::updateForces(real duration)
    {
        Registry::iterator i = registrations.begin();
        for(; i != registrations.end(); i++)
        {
            i->fg->updateForce(i->body, duration);
        }
    }

    Gravity::Gravity(const Vector3 &gravity) : gravity(gravity) {}

    void Gravity::updateForce(RigidBody *body, real duration)
    {
        if(!body->hasFiniteMass()) return;

        body->addForce(gravity * body->getMass());
    }


    Spring::Spring(const Vector3 &localConnectionPt,
                   RigidBody *other,
                   const Vector3 &otherConnectionPt,
                   real springConstant,
                   real restLength)
        : connectionPoint(localConnectionPt),
          otherConnectionPoint(otherConnectionPt),
          other(other),
          springConstant(springConstant),
          restLength(restLength) 
    {
    }

    void Spring::updateForce(RigidBody *body, real duration)
    {
        // Calculate the two ends in world space
        Vector3 lws = body->getPointInWorldSpace(connectionPoint);
        Vector3 ows = other->getPointInWorldSpace(otherConnectionPoint);

        // Calculate the vector of the spring
        Vector3 force = lws - ows;

        // Calculate the magnitude of the force
        real magnitude = force.magnitude();
        magnitude = real_abs(magnitude - restLength);
        magnitude *= springConstant;

        // Calculate the final force and apply it
        force.normalize();
        force *= -magnitude;
        body->addForcePoint(force, lws);
    }


}
