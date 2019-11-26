#include "obstacleboxes.h"
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <vector>

obstacleBoxes::obstacleBoxes(std::vector<float> obstacleSize, std::vector<float> obstaclePosition)
{
    mSizeX = obstacleSize[0];
    mSizeY = obstacleSize[1];
    mSizeZ = obstacleSize[2];
    mPositionX = obstaclePosition[0];
    mPositionY = obstaclePosition[1];
    mPositionZ = obstaclePosition[2];
    create_physics_box();
    create_osg_box();
}


void obstacleBoxes::create_physics_box()
{
    btVector3 positionPhysics{mPositionX,mPositionY, mPositionZ};
//    btVector3 positionPhysics{0,0,0};
    mObstacleShape = new btBoxShape(btVector3(mSizeX*.5f,mSizeY*.5f,mSizeZ*.5f));
//    mObstacleShape = new btBoxShape(btVector3(200,200,200));
    mObstacleMotionState= new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),positionPhysics));
    mRigidConstructionInfo= new btRigidBody::btRigidBodyConstructionInfo(0,mObstacleMotionState,mObstacleShape,btVector3(0,0,0));
    mRigidConstructionInfo->m_restitution = 0.9f;
    mRigidBody = new btRigidBody(*mRigidConstructionInfo);
}
void obstacleBoxes::destroy()
{
    delete mRigidBody;
    delete mRigidConstructionInfo;
    delete mObstacleShape;
    delete mObstacleMotionState;
}

void obstacleBoxes::create_osg_box()
{
        osg::Vec3 positionOSG{mPositionX, mPositionY, mPositionZ};
//    osg::Vec3 positionOSG{0, 0, 0};
   mOSGBox  = new osg::Box( positionOSG, mSizeX,mSizeY,mSizeZ );
//     mOSGBox  = new osg::Box( positionOSG, 400,400,400 );
    osg::ShapeDrawable* sd = new osg::ShapeDrawable( mOSGBox );
    sd->setColor(  osg::Vec4(mColor[0], mColor[1], mColor[2],mColor[3]));

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( sd );


    osg::StateSet* stateSet = geode->getOrCreateStateSet();
    osg::Material* material = new osg::Material;

    material->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );

    stateSet->setAttributeAndModes( material, osg::StateAttribute::ON );
    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );

    mTransform = new osg::MatrixTransform;

    mTransform->addChild(geode);
}

obstacleBoxes::~obstacleBoxes()
{
    destroy();
}
