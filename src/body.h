#ifndef PHY_BODY_H
#define PHY_BODY_H

#include "core.h"

namespace Phy
{

    class RigidBody
    {
    protected:
        real inverseMass;
        real linearDamping;
        // linear position of the rigidbody in world space
        Vector3 position;
        // angular orientation of the rigidbody in world space
        Quaternion orientation;
        // linear velocity of the rigidbody in world space
        Vector3 velocity;
        // angular velocity of the rigidbody in world space
        Vector3 rotation;
        /* Holds a transform matrix for converting body space into
         * world space and vice versa. */
        Matrix4 transformMatrix;

        Matrix3 inverseInertiaTensor;
    public:
        void calculateDerivedData();

        void setInertiaTensor(const Matrix3 &inertiaTensor);
    };

}


#endif
