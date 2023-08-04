
// This file is part of the OGRE project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at https://www.ogre3d.org/licensing.
// SPDX-License-Identifier: MIT

#include "Ogre.h"
#include "OgreApplicationContext.h"

#include "Particle.h"

Ogre::Vector3 PhyToOgre(Phy::Vector3& v)
{
    return Ogre::Vector3(v.x, v.y, v.z);
}

Phy::Vector3 OgreToPhy(Ogre::Vector3& v)
{
    return Phy::Vector3(v.x, v.y, v.z);
}

class OgreApp : public Ogre::FrameListener, public OgreBites::InputListener
{
public:
    OgreApp(Ogre::SceneNode* node)
        : playerNode(node)
    {
        player.setMass(0.1);
        player.damping = 0.01;
    }
    ~OgreApp() {}
    bool frameStarted(const Ogre::FrameEvent& evt)
    {
        Ogre::Real dt = evt.timeSinceLastFrame;
        Ogre::Real speed = 200.0;

        Phy::Vector3 movementForce;
        if(keyboard['w'] == true)
        {
            movementForce += Phy::Vector3(0, 0, -1);
        }
        if(keyboard['s'] == true)
        {
            movementForce += Phy::Vector3(0, 0, 1);
        }
        if(keyboard['d'] == true)
        {
            movementForce += Phy::Vector3(1, 0, 0);
        }
        if(keyboard['a'] == true)
        {
            movementForce += Phy::Vector3(-1, 0, 0);
        }
        movementForce.normalize();
        movementForce *= speed;

        player.addForce(movementForce);
        if(dt > 0.0) player.integrate(dt);

        playerNode->setPosition(PhyToOgre(player.position));


        return true;
    }

    bool keyPressed(const OgreBites::KeyboardEvent& evt) override
    {
        if(evt.keysym.sym < 345) keyboard[evt.keysym.sym] = true;
        if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
        {
            Ogre::Root::getSingleton().queueEndRendering();
        }
        return true;
    }

    bool keyReleased(const OgreBites::KeyboardEvent& evt)
    {
        if(evt.keysym.sym < 345) keyboard[evt.keysym.sym] = false;
        return true;
    }
private:

    Ogre::SceneNode* playerNode;
    Phy::Particle player;
    bool keyboard[345];
};

int main(int argc, char *argv[])
{
//! [constructor]
    OgreBites::ApplicationContext ctx("OgrePhysics");
    ctx.initApp();
//! [constructor]

//! [setup]
    // get a pointer to the already created root
    Ogre::Root* root = ctx.getRoot();
    Ogre::SceneManager* scnMgr = root->createSceneManager();

    // register our scene with the RTSS
    Ogre::RTShader::ShaderGenerator* shadergen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);

    // without light we would just get a black screen    
    Ogre::Light* spotLight = scnMgr->createLight("SpotLight");
    spotLight->setDiffuseColour(0, 0, 1.0);
    spotLight->setSpecularColour(0, 0, 1.0);
    spotLight->setType(Ogre::Light::LT_SPOTLIGHT);
    //spotLight->setSpotlightRange(Ogre::Degree(35), Ogre::Degree(50));

    Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(spotLight);
    lightNode->setDirection(-1, -1, 0);
    lightNode->setPosition(200, 400, 0);

    Ogre::Light* directionalLight = scnMgr->createLight("DirectionalLight");
    directionalLight->setDiffuseColour(0.4, 0, 0);
    directionalLight->setSpecularColour(0.4, 0, 0);
    directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
    
    Ogre::SceneNode* directionalLightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    directionalLightNode->attachObject(directionalLight);
    directionalLightNode->setDirection(0, -1, 1);

    // also need to tell where we are
    
    // finally something to render
    Ogre::Entity* ninjaEntity = scnMgr->createEntity("ninja.mesh");
    ninjaEntity->setCastShadows(true);
    Ogre::SceneNode* playerNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    playerNode->attachObject(ninjaEntity);

    // create the camera ass child of the player
    Ogre::Camera* cam = scnMgr->createCamera("myCam");
    cam->setNearClipDistance(5); // specific to this sample
    cam->setAutoAspectRatio(true);
    
    //Ogre::SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    Ogre::SceneNode* camNode = playerNode->createChildSceneNode();
    camNode->attachObject(cam);
    camNode->setPosition(0, 500, 800);
    camNode->lookAt(Ogre::Vector3(0, 0, 0), Ogre::Node::TransformSpace::TS_WORLD);

    // and tell it to render into the main window
    Ogre::Viewport* vp = ctx.getRenderWindow()->addViewport(cam);
    vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0));
    
    // create the ground
    Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
    Ogre::MeshManager::getSingleton().createPlane(
            "ground", Ogre::RGN_DEFAULT,
            plane,
            1500, 1500, 20, 20,
            true,
            1, 5, 5,
            Ogre::Vector3::UNIT_Z);

    Ogre::Entity* groundEntity = scnMgr->createEntity("ground");
    scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(groundEntity);
    groundEntity->setCastShadows(false);
    groundEntity->setMaterialName("Examples/Rockwall");


    scnMgr->setAmbientLight(Ogre::ColourValue(0.3, 0.3, 0.3));
    scnMgr->setShadowTechnique(Ogre::ShadowTechnique::SHADOWTYPE_STENCIL_ADDITIVE);


//! [setup]

//! [main]
    // register for input events
    OgreApp app(playerNode);
    
    ctx.addInputListener(&app);
    ctx.getRoot()->addFrameListener(&app);

    ctx.getRoot()->startRendering();
    ctx.closeApp();
//! [main]
    return 0;
}
