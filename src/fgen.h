#ifndef PHY_FGEN_H
#define PHY_FGEN_H

#include "body.h"

#include <vector>

namespace Phy
{
    class ForceGenerator
    {
    public:
        virtual void updateForce(RigidBody *body, real duration) = 0;
    };

    class Gravity : public ForceGenerator
    {
        Vector3 gravity;
    public:
        Gravity(const Vector3 &gravity);
        virtual void updateForce(RigidBody *body, real duration);
    };

    class Spring : public ForceGenerator
    {
        // the point of conection of the spring, in local coordinates
        Vector3 connectionPoint;

        // the point of conection of the spring to the other object
        // in that object's local coordinates.
        Vector3 otherConnectionPoint;

        // the body at the other end of the spring
        RigidBody *other;

        real springConstant;
        real restLength;
    public:
        Spring(const Vector3 &localConnectionPt,
               RigidBody *other,
               const Vector3 &otherConnectionPt,
               real springConstant,
               real restLength);

        virtual void updateForce(RigidBody *body, real duration);
    };

    class ForceRegistry
    {
    protected:
        
        struct ForceRegistration
        {
            RigidBody *body;
            ForceGenerator *fg;
        };

        typedef std::vector<ForceRegistration> Registry;
        Registry registrations;
    public:
        void add(RigidBody *body, ForceGenerator *fg);
        void updateForces(real duration);
    };
}

#endif
