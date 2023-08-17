#ifndef PHY_COLLIDE_COARSE_H
#define PHY_COLLIDE_COARSE_H

#include "body.h"

namespace Phy
{

    struct BoundingSphere
    {
        Vector3 center;
        real radius;
    public:
        // Creates a new bounding sphere at the given center and radius
        BoundingSphere(const Vector3 &center, real radius);
        // Creates a bounding sphere to enclose the two given bounding spheres.
        BoundingSphere(const BoundingSphere &one, const BoundingSphere &two);
        /* Checks wheter the bounding sphere overlaps with the other given
         * bounding sphere.
         */
        bool overlaps(const BoundingSphere *other) const;
    };

    struct PotentialContact
    {
        RigidBody *body[2];
    };

    /* A template class for nodes in a bounding volume hierarchy. This class uses
     * a binary tree to store the bounding volumes.
     */

    template<class BoundingVolumeClass>
    class BVHNode
    {
    public:
        // holds the child node of this node
        BVHNode *children[2];
        BVHNode *parent;

        /* holds a single bounding volume encompassing all the
         * descendants of this node */
        BoundingVolumeClass volume;

        /* Holds the rigid body at this node of the hierarchy.
         * Only leaf nodes can have a rigid body defined */
        RigidBody *body;

        BVHNode(BVHNode *parent, const BoundingVolumeClass &volume,
            RigidBody* body=NULL)
            : parent(parent), volume(volume), body(body)
        {
            children[0] = children[1] = NULL;
        }

        bool isLeaf() const
        {
            return (body != NULL);
        }

        /* Checks the potential contacts from this node downward in
         * the hierarchy, writing to the given array (up to the
         * given limit). Returns the number of potential contacts it found.
         */
        unsigned getPotentialContacts(PotentialContact *contacts, unsigned limit) const;
        
        /* Inserts the given rigid body, with the given bounding volume,
         * into the hierarchy. this may involve the create of further
         * bounding volume nodes.
         */
        void insert(RigidBody *body, const BoundingVolumeClass &volume);

        ~BVHNode();

    protected:

        unsigned getPotentialContactsWith(BVHNode<BoundingVolumeClass> *other,
                                          PotentialContact *contacts, unsigned limit) const;


        bool overlaps(const BVHNode<BoundingVolumeClass> *other) const;


        void recalculateBoundingVolume(bool recurse = true);

    };

    template <class BoundingVolumeClass>
    bool BVHNode<BoundingVolumeClass>::overlaps(const BVHNode<BoundingVolumeClass> *other) const
    {
        return volume->overlaps(other->volume);
    }

    template <class BoundingVolumeClass>
    void BVHNode<BoundingVolumeClass>::insert(RigidBody *newBody, const BoundingVolumeClass &newVolume)
    {
        // If we are a leaf, then the only option is to spawn two
        // new childrens and place the new body in one.
        if(isLeaf())
        {
            // Child one is a copy of us.
            children[0] = new BVHNode<BoundingVolumeClass>(this, volume, body);
            // Child two holds the new body
            children[1] = new BVHNode<BoundingVolumeClass>(this, newVolume, newBody);

            // and now we lose the body (we are no longer a leaf).
            this->body = NULL;
            
            // we need to recalculate our bounding volume
            recalculateBoundingVolume();
        }
        // Otherwise we need to work out which child gets to keep
        // the inserted body. we give it to whoever would grow the
        // least to incorporate it.
        else
        {
            if(children[0]->volume.getGrowth(newVolume) <
               children[1]->volume.getGrowth(newVolume))
            {
                children[0]->insert(newBody, newVolume);
            }
            else
            {
                children[1]->insert(newBody, newVolume);
            }
        }
    }

    template <class BoundingVolumeClass>
    BVHNode<BoundingVolumeClass>::~BVHNode()
    {
        // If we don't have a parent, then we ignore the sibling
        // processing.
        if(parent)
        {
            // Find out sibling.
            BVHNode<BoundingVolumeClass> *sibling;
            if(parent->children[0] == this) sibling = parent->children[1];
            else sibling = parent->children[0];

            // Write its data to our parent.
            parent->volume = sibling->volume;
            parent->body = sibling->body;
            parent->children[0] = sibling->children[0];
            parent->children[1] = sibling->children[1];

            // Delete the sibling (we blank its parent and children to avoid processing/deleting them).
            sibling->parent = NULL;
            sibling->body = NULL;
            sibling->children[0] = NULL;
            sibling->children[1] = NULL;
            delete sibling;

            parent->recalculateBoundingVolume();
        }

        // Delete our children
        if(children[0])
        {
            children[0]->parent = NULL;
            delete children[0];
        }
        if(children[1])
        {
            children[1]->parent = NULL;
            delete children[1];
        }
    }

    template <class BoundingVolumeClass>
    void BVHNode<BoundingVolumeClass>::recalculateBoundingVolume(bool recurse)
    {
        if (isLeaf()) return;

        // Use the bounding volume combining constructor.
        volume = BoundingVolumeClass(
            children[0]->volume,
            children[1]->volume
            );

        // Recurse up the tree
        if (parent) parent->recalculateBoundingVolume(true);
    }

    template <class BoundingVolumeClass>
    unsigned BVHNode<BoundingVolumeClass>::getPotentialContacts(
            PotentialContact *contacts, unsigned limit) const
    {
        // Early out if we dont have the room for contacts, or
        // if we are a leaf node
        if(isLeaf() || limit == 0) return 0;

        // Get the potential contacts of one of our children with the other
        return children[0]->getPotentialContactsWith(children[1], contacts, limit);
    }

    template <class BoundingVolumeClass>
    unsigned BVHNode<BoundingVolumeClass>::getPotentialContactsWith(
                                      BVHNode<BoundingVolumeClass> *other,
                                      PotentialContact *contacts, unsigned limit) const
    {
        // Early out if we dont overlap or if we have no room
        // to report contacts
        if(!overlaps(other) || limit == 0) return 0;

        // if we are both at leaf nodes, the we have a potential contact.
        if(isLeaf() && other->isLeaf())
        {
            contacts->body[0] = body;
            contacts->body[1] = other->body;
            return 1;
        }

        // Determine which node to descend into. If either is a leaf, then we descend the other.
        // If both are branches, then we use the one with the largest size
        if(other->isLeaf() ||
           (!isLeaf() && volume->getSize() >= other->volume->getSize()))
        {
            // Resurce into self
            unsigned count = children[0]->getPotentialContactsWith(other, contacts, limit);

            // Chech that we have enough slots to do the other side too.
            if(limit > count)
            {
                return count + children[1]->getPotentialContactsWith(other, contacts+count, limit-count);
            }
            else
            {
                return count;
            }
        }
        else
        {
            // Resurce into the other node.
            unsigned count = getPotentialContactsWith(other->children[0], contacts, limit);
            if(limit > count)
            {
                return count + getPotentialContactsWith(other->children[1], contacts+count, limit-count);
            }
            else
            {
                return count;
            }
        }
    }


}

#endif
