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
        real angularDamping;
        // linear position of the rigidbody in world space
        Vector3 position;
        // angular orientation of the rigidbody in world space
        Quaternion orientation;
        // linear velocity of the rigidbody in world space
        Vector3 velocity;
        // angular velocity of the rigidbody in world space
        Vector3 rotation;

        Vector3 forceAccum;
        Vector3 torqueAccum;
        Vector3 acceleration;
        Vector3 lastFrameAcceleration;

        Matrix3 inverseInertiaTensor;

        // Derive Data: ##########################################
        /* Holds a transform matrix for converting body space into
         * world space and vice versa. */
        Matrix4 transformMatrix;
        Matrix3 inverseInertiaTensorWorld;
        bool isAwake;
        // #######################################################
    public:

        void calculateDerivedData();

        void integrate(real duration);

        void setPosition(const Vector3 &position);
        void setPosition(const real x, const real y, const real z);
        void getPosition(Vector3 *position) const;
        Vector3 getPosition() const;

        void setVelocity(const Vector3 &velocity);
        void setVelocity(const real x, const real y, const real z);
        void getVelocity(Vector3 *velocity) const;
        Vector3 getVelocity() const;
        void addVelocity(const Vector3 &deltaVelocity);

        void setRotation(const Vector3 &rotation);
        void setRotation(const real x, const real y, const real z);
        void getRotation(Vector3 *rotation) const;
        Vector3 getRotation() const;
        void addRotation(const Vector3 &deltaRotation);

        void setAcceleration(const Vector3 &acceleration);
        void setAcceleration(const real x, const real y, const real z);
        void getAcceleration(Vector3 *acceleration);
        Vector3 getAcceleration() const;

        void setOrientation(const Quaternion &orientation);
        void setOrientation(const real r, const real i, const real j, const real k);
        void getOrientation(Quaternion *orientation) const;
        Quaternion getOrientation() const;
        void getOrientation(real matrix[9]) const;

        void setMass(const real mass);
        real getMass() const;
        void setInverseMass(const real inverseMass);
        real getInverseMass() const;
        bool hasFiniteMass() const;

        void setDamping(const real linearDamping,
                        const real angularDamping);
        void setLinearDamping(const real linearDamping);
        real getLinearDamping() const;
        void setAngularDamping(const real angularDamping);
        real getAngularDamping() const;

        void setInertiaTensor(const Matrix3 &inertiaTensor);

        void clearAccumulators();
        void addForce(const Vector3 &force);
        void addForcePoint(const Vector3 &force, const Vector3 &point);
        void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
        void addTorque(const Vector3 &torque);

        Vector3 getPointInLocalSpace(const Vector3 &point) const;
        Vector3 getPointInWorldSpace(const Vector3 &point) const;

        Matrix4 getTransform() const;
 
    };

}


#endif
