#ifndef PHY_CONTACTS_H
#define PHY_CONTACTS_H

#include "body.h"

namespace Phy
{

    class Contact
    {
        // Hold the position of the contact in world coordinate
        Vector3 contactPoint;
        // Hold the direction of the contact in world coordinate
        Vector3 contactNormal;
        // Hold the penetration at the contact point
        real penetration;
    };

}

#endif
