#ifndef PHY_COLLIDE_FINE_H
#define PHY_COLLIDE_FINE_H

#include "contacts.h"

namespace Phy
{
    class CollisionPrimitive
    {
    public:

        /* The rigid body that is represented by this primitive */
        RigidBody *body;
        /* The offset of this primitive form the given rigid body */
        Matrix4 offset;

        void calculateInternals();

        Vector3 getAxis(unsigned index) const
        {
            return transform.getAxisVector(index);
        }

        const Matrix4 &getTransform() const
        {
            return transform;
        }


    protected:

        Matrix4 transform;

    };


    /*
     * Represent a rigid body that can be treated as a sphere for collision detection
     */
    class CollisionSphere : public CollisionPrimitive
    {
    public:
        real radius;
    };

    /*
     * The plane is not a primitive: it doesnt represent another rigid body.
     * It is used for contacts with the immovable world geometry
     */
    class CollisionPlane
    {
    public:
        // The plane normal;
        Vector3 direction;
        // the distance of the plane from the origin.
        real offset;
    };

    /*
     * Represents a rigid body that can be treated as an aligned bounding
     * box for collision detection
     */
    class CollisionBox : public CollisionPrimitive
    {
    public:
        // Holds the half-sizes of the box along each of its local axes.
        Vector3 halfSize;
    };

    /*
     * A wrapper class that holds fast intersection tests. These can be used
     * to drive the coarse collision detection system or as an early out in the full
     * collision test below.
     */
    class IntersectionTests
    {
        static bool sphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane);
        static bool sphereAndSphere(const CollisionSphere &one, const CollisionSphere &two);
        static bool boxAndBox(const CollisionBox &one, const CollisionBox &two);
        static bool boxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane);
    };

    /*
     * A halper structure that contains information foe thw detector to use
     * in building its contact data.
     */
    struct CollisionData
    {
        Contact *contactArray;
        Contact *contacts;
        int contactsLeft;
        unsigned contactCount;
        real friction;
        real restitution;
        real tolerance;

        bool hasMoreContacts()
        {
            return contactsLeft > 0;
        }

        void reset(unsigned maxContacts)
        {
            contactsLeft = maxContacts;
            contactCount = 0;
            contacts = contactArray;
        }

        void addContacts(unsigned count)
        {
            contactsLeft -= count;
            contactCount += count;
            contacts += count;
        }
    };

    /*
     * A wrapper class that holds the fine grained collision detection
     * routines.
     *
     * Each of the functions has the same format: it takes the details
     * of two objects, and pointer to a contact array to fill. It
     * returns the number of contacts it wrote into the array.
     */
    class CollisionDetector
    {
    public:
        static unsigned sphereAndHalfSpace(const CollisionSphere &sphere,
                                           const CollisionPlane &plane,
                                           CollisionData *data);
        static unsigned sphereAndTruePlane(const CollisionSphere &sphere,
                                           const CollisionPlane &plane,
                                           CollisionData *data);
        static unsigned sphereAndSphere(const CollisionSphere &one,
                                        const CollisionSphere &two,
                                        CollisionData *data);
        static unsigned boxAndHalfSpace(const CollisionBox &box,
                                        const CollisionPlane &plane,
                                        CollisionData *data);
        static unsigned boxAndBox(const CollisionBox &one,
                                  const CollisionBox &two,
                                  CollisionData *data);
        static unsigned boxAndPoint(const CollisionBox &box,
                                    const Vector3 &point,
                                    CollisionData *data);
        static unsigned boxAndSphere(const CollisionBox &box,
                                     const CollisionSphere &sphere,
                                     CollisionData *data);

    };

}

#endif
