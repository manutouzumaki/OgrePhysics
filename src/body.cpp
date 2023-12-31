#include "body.h"

#define Assert(Expression) if(!(Expression)) {*(int *)0 = 0;}

namespace Phy
{
    static inline void _transformInertiaTensor(Matrix3 &iitWorld,
                                               const Quaternion &q,
                                               const Matrix3 &iitBody,
                                               const Matrix4 &rotmat)
    {
        real t4 = rotmat.data[0]*iitBody.data[0]+
            rotmat.data[1]*iitBody.data[3]+
            rotmat.data[2]*iitBody.data[6];
        real t9 = rotmat.data[0]*iitBody.data[1]+
            rotmat.data[1]*iitBody.data[4]+
            rotmat.data[2]*iitBody.data[7];
        real t14 = rotmat.data[0]*iitBody.data[2]+
            rotmat.data[1]*iitBody.data[5]+
            rotmat.data[2]*iitBody.data[8];
        real t28 = rotmat.data[4]*iitBody.data[0]+
            rotmat.data[5]*iitBody.data[3]+
            rotmat.data[6]*iitBody.data[6];
        real t33 = rotmat.data[4]*iitBody.data[1]+
            rotmat.data[5]*iitBody.data[4]+
            rotmat.data[6]*iitBody.data[7];
        real t38 = rotmat.data[4]*iitBody.data[2]+
            rotmat.data[5]*iitBody.data[5]+
            rotmat.data[6]*iitBody.data[8];
        real t52 = rotmat.data[8]*iitBody.data[0]+
            rotmat.data[9]*iitBody.data[3]+
            rotmat.data[10]*iitBody.data[6];
        real t57 = rotmat.data[8]*iitBody.data[1]+
            rotmat.data[9]*iitBody.data[4]+
            rotmat.data[10]*iitBody.data[7];
        real t62 = rotmat.data[8]*iitBody.data[2]+
            rotmat.data[9]*iitBody.data[5]+
            rotmat.data[10]*iitBody.data[8];

        iitWorld.data[0] = t4*rotmat.data[0]+
            t9*rotmat.data[1]+
            t14*rotmat.data[2];
        iitWorld.data[1] = t4*rotmat.data[4]+
            t9*rotmat.data[5]+
            t14*rotmat.data[6];
        iitWorld.data[2] = t4*rotmat.data[8]+
            t9*rotmat.data[9]+
            t14*rotmat.data[10];
        iitWorld.data[3] = t28*rotmat.data[0]+
            t33*rotmat.data[1]+
            t38*rotmat.data[2];
        iitWorld.data[4] = t28*rotmat.data[4]+
            t33*rotmat.data[5]+
            t38*rotmat.data[6];
        iitWorld.data[5] = t28*rotmat.data[8]+
            t33*rotmat.data[9]+
            t38*rotmat.data[10];
        iitWorld.data[6] = t52*rotmat.data[0]+
            t57*rotmat.data[1]+
            t62*rotmat.data[2];
        iitWorld.data[7] = t52*rotmat.data[4]+
            t57*rotmat.data[5]+
            t62*rotmat.data[6];
        iitWorld.data[8] = t52*rotmat.data[8]+
            t57*rotmat.data[9]+
            t62*rotmat.data[10];
    }


    static inline void _calculateTransformMatrix(Matrix4 &transformMatrix,
                                                 const Vector3 &position,
                                                 const Quaternion &orientation)
    {
        transformMatrix.data[0] = 1-2*orientation.j*orientation.j-
            2*orientation.k*orientation.k;
        transformMatrix.data[1] = 2*orientation.i*orientation.j -
            2*orientation.r*orientation.k;
        transformMatrix.data[2] = 2*orientation.i*orientation.k +
            2*orientation.r*orientation.j;
        transformMatrix.data[3] = position.x;

        transformMatrix.data[4] = 2*orientation.i*orientation.j +
            2*orientation.r*orientation.k;
        transformMatrix.data[5] = 1-2*orientation.i*orientation.i-
            2*orientation.k*orientation.k;
        transformMatrix.data[6] = 2*orientation.j*orientation.k -
            2*orientation.r*orientation.i;
        transformMatrix.data[7] = position.y;

        transformMatrix.data[8] = 2*orientation.i*orientation.k -
            2*orientation.r*orientation.j;
        transformMatrix.data[9] = 2*orientation.j*orientation.k +
            2*orientation.r*orientation.i;
        transformMatrix.data[10] = 1-2*orientation.i*orientation.i-
            2*orientation.j*orientation.j;
        transformMatrix.data[11] = position.z;
    }

    void RigidBody::calculateDerivedData()
    {
        orientation.normalize();
        // Calculate the transform matrix for the body
        _calculateTransformMatrix(transformMatrix, position, orientation);
        // calculate the inertiaTensor in world space (change of basis)
        _transformInertiaTensor(inverseInertiaTensorWorld,
                                orientation,
                                inverseInertiaTensor,
                                transformMatrix);

    }

    void RigidBody::integrate(real duration)
    {
        if(!isAwake) return;

        // Calculate linear acceleration from force inputs.
        lastFrameAcceleration = acceleration;
        lastFrameAcceleration.addScaledVector(forceAccum, inverseMass);

        // Calculate angular acceleration from torque inputs.
        Vector3 angularAcceleration =
            inverseInertiaTensorWorld.transform(torqueAccum);

        // Adjust velocities.
        // Update linear velocity from both acceleration and impulses
        velocity.addScaledVector(lastFrameAcceleration, duration);

        // Update angular velocity from both acceleration and impulses
        rotation.addScaledVector(angularAcceleration, duration);

        // Impose drag
        velocity *= real_pow(linearDamping, duration);
        rotation *= real_pow(angularDamping, duration);

        // Adjust positions.
        // Update linear position
        position.addScaledVector(velocity, duration);
        // Update angular position
        orientation.addScaledVector(rotation, duration);

        calculateDerivedData();
        clearAccumulators();
    }

    void RigidBody::setPosition(const Vector3 &position)
    {
        RigidBody::position = position;
    }

    void RigidBody::setPosition(const real x, const real y, const real z)
    {
        position.x = x;
        position.y = y;
        position.z = z;
    }

    void RigidBody::getPosition(Vector3 *position) const
    {
        *position = RigidBody::position;
    }

    Vector3 RigidBody::getPosition() const
    {
        return position;
    }

    void RigidBody::setVelocity(const Vector3 &velocity)
    {
        RigidBody::velocity = velocity;
    }

    void RigidBody::setVelocity(const real x, const real y, const real z)
    {
        velocity.x = x;
        velocity.y = y;
        velocity.z = z;
    }

    void RigidBody::getVelocity(Vector3 *velocity) const
    {
        *velocity = RigidBody::velocity;
    }

    Vector3 RigidBody::getVelocity() const
    {
        return velocity;
    }

    void RigidBody::addVelocity(const Vector3 &deltaVelocity)
    {
        velocity += deltaVelocity;
    }

    void RigidBody::setRotation(const Vector3 &rotation)
    {
        RigidBody::rotation = rotation;
    }

    void RigidBody::setRotation(const real x, const real y, const real z)
    {
        rotation.x = x;
        rotation.y = y;
        rotation.z = z;
    }
    
    void RigidBody::getRotation(Vector3 *rotation) const
    {
        *rotation = RigidBody::rotation;
    }

    Vector3 RigidBody::getRotation() const
    {
        return rotation;
    }

    void RigidBody::addRotation(const Vector3 &deltaRotation)
    {
        rotation += deltaRotation;
    }

    void RigidBody::setAcceleration(const Vector3 &acceleration)
    {
        RigidBody::acceleration = acceleration;
    }

    void RigidBody::setAcceleration(const real x, const real y, const real z)
    {
        acceleration.x = x;
        acceleration.y = y;
        acceleration.z = z;
    }

    void RigidBody::getAcceleration(Vector3 *acceleration)
    {
        *acceleration = RigidBody::acceleration;
    }

    Vector3 RigidBody::getAcceleration() const
    {
        return acceleration;
    }

    void RigidBody::setOrientation(const Quaternion &orientation)
    {
        RigidBody::orientation = orientation;
        RigidBody::orientation.normalize();
    }

    void RigidBody::setOrientation(const real r, const real i, const real j, const real k)
    {
        orientation.r = r;
        orientation.i = i;
        orientation.j = j;
        orientation.k = k;
        orientation.normalize();
    }

    void RigidBody::getOrientation(Quaternion *orientation) const
    {
        *orientation = RigidBody::orientation;
    }

    Quaternion RigidBody::getOrientation() const
    {
        return orientation;
    }

    void RigidBody::getOrientation(real matrix[9]) const
    {
        matrix[0] = transformMatrix.data[0];
        matrix[1] = transformMatrix.data[1];
        matrix[2] = transformMatrix.data[2];

        matrix[3] = transformMatrix.data[4];
        matrix[4] = transformMatrix.data[5];
        matrix[5] = transformMatrix.data[6];

        matrix[6] = transformMatrix.data[8];
        matrix[7] = transformMatrix.data[9];
        matrix[8] = transformMatrix.data[10];
    }

    void RigidBody::setMass(const real mass)
    {
        Assert(mass != 0);
        RigidBody::inverseMass = ((real)1.0)/mass;
    }

    real RigidBody::getMass() const
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

    void RigidBody::setInverseMass(const real inverseMass)
    {
        RigidBody::inverseMass = inverseMass;
    }

    real RigidBody::getInverseMass() const
    {
        return inverseMass;
    }


    bool RigidBody::hasFiniteMass() const
    {
        return inverseMass >= 0.0f;
    }

    void RigidBody::setDamping(const real linearDamping,
                               const real angularDamping)
    {
        RigidBody::linearDamping = linearDamping;
        RigidBody::angularDamping = angularDamping;
    }

    void RigidBody::setLinearDamping(const real linearDamping)
    {
        RigidBody::linearDamping = linearDamping;
    }

    real RigidBody::getLinearDamping() const
    {
        return linearDamping;
    }

    void RigidBody::setAngularDamping(const real angularDamping)
    {
        RigidBody::angularDamping = angularDamping;
    }

    real RigidBody::getAngularDamping() const
    {
        return angularDamping;
    }

    void RigidBody::setInertiaTensor(const Matrix3 &inertiaTensor)
    {
        inverseInertiaTensor.setInverse(inertiaTensor);
    }

    void RigidBody::clearAccumulators()
    {
        forceAccum.clear();
        torqueAccum.clear();
    }

    void RigidBody::addForce(const Vector3 &force)
    {
        forceAccum += force;
        isAwake = true;
    }

    void RigidBody::addForcePoint(const Vector3 &force, const Vector3 &point)
    {
        // convert to coordinates relative to center of mass.
        Vector3 pt = point;
        pt -= position;

        forceAccum += force;
        torqueAccum += pt % force;

        isAwake = true;
    }

    void RigidBody::addForceAtBodyPoint(const Vector3 &force, const Vector3 &point)
    {
        // convert to coordinates relative to center of mass
        Vector3 pt = getPointInWorldSpace(point);
        addForcePoint(force, pt);

        isAwake = true;
    }

    void RigidBody::addTorque(const Vector3 &torque)
    {
        torqueAccum += torque;
        isAwake = true;
    }

    Vector3 RigidBody::getPointInLocalSpace(const Vector3 &point) const
    {
        return transformMatrix.transformInverse(point);
    }

    Vector3 RigidBody::getPointInWorldSpace(const Vector3 &point) const
    {
        return transformMatrix.transform(point);
    }

    Matrix4 RigidBody::getTransform() const
    {
        return transformMatrix;
    }
}
