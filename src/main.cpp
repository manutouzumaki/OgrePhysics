
// This file is part of the OGRE project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at https://www.ogre3d.org/licensing.
// SPDX-License-Identifier: MIT

#include "Ogre.h"
#include "OgreApplicationContext.h"

#include "pworld.h"

Ogre::Vector3 PhyToOgre(Phy::Vector3& v)
{
    return Ogre::Vector3(v.x, v.y, v.z);
}

Phy::Vector3 OgreToPhy(Ogre::Vector3& v)
{
    return Phy::Vector3(v.x, v.y, v.z);
}

#define ROD_COUNT 6
#define CABLE_COUNT 10
#define SUPPORT_COUNT 12

#define BASE_MASS 1
#define EXTRA_MASS 10

class OgreApp : public Ogre::FrameListener, public OgreBites::InputListener
{
public:

    OgreApp(unsigned int _particleCount, Ogre::SceneNode** _nodes) 
        : particleCount(_particleCount), nodes(_nodes), world(_particleCount*10), cables(0), rods(0), massPos(0, 0, 0.5f)
    {
        particleArray = new Phy::Particle[particleCount];
        for(unsigned i = 0; i < particleCount; i++)
        {
            world.getParticles().push_back(particleArray + i);
        }
        groundContactGenerator.init(&world.getParticles());
        world.getContactGenerators().push_back(&groundContactGenerator);
        
        // TODO: initialize the particles ...
        
        // create the masses and conections
        for(unsigned i = 0; i < 12; i++)
        {
            unsigned x = (i%12)/2;
            particleArray[i].position = Phy::Vector3(
                        Phy::real(i/2)*2.0f-5.0f,
                        4,
                        Phy::real(i%2)*2.0f-1.0f);
            particleArray[i].velocity = Phy::Vector3(0, 0, 0);
            particleArray[i].damping = 0.9;
            particleArray[i].acceleration = Phy::Vector3(0, -9.81, 0);
            particleArray[i].clearAccumulator();

            nodes[i]->setPosition(PhyToOgre(particleArray[i].position));
        }

        nodes[12]->setPosition(0, 0, 0);

        // add the links
        cables = new Phy::ParticleCable[CABLE_COUNT];
        for(unsigned i = 0; i < CABLE_COUNT; i++)
        {
            cables[i].particle[0] = &particleArray[i];
            cables[i].particle[1] = &particleArray[i + 2];
            cables[i].maxLength = 1.9f;
            cables[i].restitution = 0.3f;
            world.getContactGenerators().push_back(&cables[i]);
        }
        
        support = new Phy::ParticleCableConstraint[SUPPORT_COUNT];
        for(unsigned i = 0; i < SUPPORT_COUNT; i++)
        {
            support[i].particle = particleArray+i;
            support[i].anchor = Phy::Vector3(
                    Phy::real(i/2)*2.2f-5.5f,
                    6,
                    Phy::real(i%2)*1.6f-0.8f);
            if(i < 6) support[i].maxLength = Phy::real(i/2)*0.5f + 3.0f;
            else support[i].maxLength = 5.5f - Phy::real(i/2)*0.5f;
            support[i].restitution = 0.5f;
            world.getContactGenerators().push_back(&support[i]);
        }

        rods = new Phy::ParticleRod[ROD_COUNT];
        for(unsigned i = 0; i < ROD_COUNT; i++)
        {
            rods[i].particle[0] = &particleArray[i*2];
            rods[i].particle[1] = &particleArray[i*2+1];
            rods[i].length = 2;
            world.getContactGenerators().push_back(&rods[i]);
        }

        updateAdditionalMass();
    }

    ~OgreApp() 
    {
        delete[] particleArray;
    }

    void handleInput()
    {
        if(keyboard['s'])
        {
            massPos.z += 0.1f;
            if(massPos.z > 1.0f) massPos.z = 1.0f;
        }
        if(keyboard['w'])
        {
            massPos.z -= 0.1f;
            if (massPos.z < 0.0f) massPos.z = 0.0f;
        }
        if(keyboard['a'])
        {
            massPos.x -= 0.1f;
            if (massPos.x < 0.0f) massPos.x = 0.0f;
        }
        if(keyboard['d'])
        {
            massPos.x += 0.1f;
            if (massPos.x > 5.0f) massPos.x = 5.0f;
        }

    }

    bool frameStarted(const Ogre::FrameEvent& evt)
    { 
        handleInput();

        world.startFrame();

        Ogre::Real duration = evt.timeSinceLastFrame;
        if(duration <= 0.0f) return true;

        world.runPhysics(duration);

        updateAdditionalMass();

        for(unsigned i = 0; i < 12; i++)
        {
            nodes[i]->setPosition(PhyToOgre(particleArray[i].position));
        }
        nodes[12]->setPosition(massDisplayPos.x, massDisplayPos.y+0.25f, massDisplayPos.z);

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

    void updateAdditionalMass()
    {
        for (unsigned i = 0; i < 12; i++)
        {
            particleArray[i].setMass(BASE_MASS);
        }

        // Find the coordinates of the mass as an index and proportion
        int x = int(massPos.x);
        Phy::real xp = real_fmod(massPos.x, Phy::real(1.0f));
        if (x < 0)
        {
            x = 0;
            xp = 0;
        }
        if (x >= 5)
        {
            x = 5;
            xp = 0;
        }

        int z = int(massPos.z);
        Phy::real zp = real_fmod(massPos.z, Phy::real(1.0f));
        if (z < 0)
        {
            z = 0;
            zp = 0;
        }
        if (z >= 1)
        {
            z = 1;
            zp = 0;
        }

        // Calculate where to draw the mass
        massDisplayPos.clear();

        // Add the proportion to the correct masses
        particleArray[x*2+z].setMass(BASE_MASS + EXTRA_MASS*(1-xp)*(1-zp));
        massDisplayPos.addScaledVector(
            particleArray[x*2+z].position, (1-xp)*(1-zp)
            );

        if (xp > 0)
        {
            particleArray[x*2+z+2].setMass(BASE_MASS + EXTRA_MASS*xp*(1-zp));
            massDisplayPos.addScaledVector(
                particleArray[x*2+z+2].position, xp*(1-zp)
                );

            if (zp > 0)
            {
                particleArray[x*2+z+3].setMass(BASE_MASS + EXTRA_MASS*xp*zp);
                massDisplayPos.addScaledVector(
                    particleArray[x*2+z+3].position, xp*zp
                    );
            }
        }
        if (zp > 0)
        {
            particleArray[x*2+z+1].setMass(BASE_MASS + EXTRA_MASS*(1-xp)*zp);
            massDisplayPos.addScaledVector(
                particleArray[x*2+z+1].position, (1-xp)*zp
                );
        }
    }

private:
    bool keyboard[345];

    // TODO: Game Physics
    int particleCount;
    Phy::ParticleWorld world;
    Phy::Particle* particleArray;
    Phy::GroundContacts groundContactGenerator;

    // TODO: render nodes
    Ogre::SceneNode** nodes;

    // TODO: Bridge Demo
    Phy::ParticleCableConstraint* support;
    Phy::ParticleCable* cables;
    Phy::ParticleRod* rods;

    Phy::Vector3 massPos;
    Phy::Vector3 massDisplayPos;

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
    Ogre::Entity* pointEntities[13] = {};
    Ogre::SceneNode* pointNodes[13] = {};
    for(int i = 0; i < 13; ++i)
    {
        pointEntities[i] = scnMgr->createEntity("sphere.mesh");
        pointEntities[i]->setCastShadows(true);
        pointNodes[i] = scnMgr->getRootSceneNode()->createChildSceneNode();
        pointNodes[i]->attachObject(pointEntities[i]);
        pointNodes[i]->setScale(0.005, 0.005, 0.005);
    }

    pointNodes[12]->setScale(0.003, 0.003, 0.003);
     
    // create the camera ass child of the player
    Ogre::Camera* cam = scnMgr->createCamera("myCam");
    cam->setNearClipDistance(5); // specific to this sample
    cam->setAutoAspectRatio(true);
    
    //Ogre::SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    Ogre::SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    camNode->attachObject(cam);
    camNode->setPosition(0, 10, 12);
    camNode->lookAt(Ogre::Vector3(0, 0, 0), Ogre::Node::TransformSpace::TS_WORLD);

    // and tell it to render into the main window
    Ogre::Viewport* vp = ctx.getRenderWindow()->addViewport(cam);
    vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0));
    
    // create the ground
    Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
    Ogre::MeshManager::getSingleton().createPlane(
            "ground", Ogre::RGN_DEFAULT,
            plane,
            100, 100, 1, 1,
            true,
            1, 5, 5,
            Ogre::Vector3::UNIT_Z);

    Ogre::Entity* groundEntity = scnMgr->createEntity("ground");
    Ogre::SceneNode* planeNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    planeNode->attachObject(groundEntity);
    planeNode->setPosition(0, -20, 0);
    groundEntity->setCastShadows(false);
    groundEntity->setMaterialName("Examples/Rockwall");


    scnMgr->setAmbientLight(Ogre::ColourValue(0.3, 0.3, 0.3));
    scnMgr->setShadowTechnique(Ogre::ShadowTechnique::SHADOWTYPE_STENCIL_ADDITIVE);


//! [setup]

//! [main]
    // register for input events
    OgreApp app(12, pointNodes);
    
    ctx.addInputListener(&app);
    ctx.getRoot()->addFrameListener(&app);

    ctx.getRoot()->startRendering();

    ctx.closeApp();
//! [main]
    return 0;
}
