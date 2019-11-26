//-------------------------------------------------------
// Filename: Ground.cpp
//
// Description:  The cpp file for the qt5 bullet bouncy ball example.
//
// Creator:  Professor Corey McBride for MEEN 570 - Brigham Young University
//
// Creation Date: 11/7/18
//
// Owner: Corey McBride
//-------------------------------------------------------
#include "ground.h"
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>

Ground::Ground()
{
  mSize=100;
  create();
}
Ground::Ground(int size, QVector4D& color)
{
    mSize=size;
    mColor=color;

    create();
}

void Ground::create()
{
    btVector3 positionPhysics{100,100,0};
    mGroundShape = new btBoxShape(btVector3(mSize*.5,mSize*.5,mSize*.005));
    mGroundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),positionPhysics));
    mRigidCI= new btRigidBody::btRigidBodyConstructionInfo(0,mGroundMotionState,mGroundShape,btVector3(0,0,0));
    mRigidCI->m_restitution = 0.9;
    mRigidBody = new btRigidBody(*mRigidCI);
    create_mesh();
}
void Ground::create_mesh()
{
    osg::Vec3 positionOSG{100.f, 100.f, 0.f};
    mOSGBox  = new osg::Box( positionOSG, mSize,mSize,mSize*.01 );
    osg::ShapeDrawable* sd = new osg::ShapeDrawable( mOSGBox );
    sd->setColor(  osg::Vec4(mColor[0], mColor[1], mColor[2],mColor[3]));
//    sd->setName( "Ground" );

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

void Ground::destroy()
{
    delete mRigidBody;
    delete mRigidCI;
    delete mGroundShape;
    delete mGroundMotionState;
}

//void Ground::draw(QMatrix4x4 &V_matrix, QMatrix4x4 &P_matrix)
//{
//    // This part of the code updates the rigid body
//    // position in the bullet world. It also gets that
//    // position as an OpenGL format.
//    mRigidBody->getMotionState()->getWorldTransform(trans);
//    trans.getOpenGLMatrix(btMat);

//    M=QMatrix4x4(btMat);
//    M=M.transposed();
//    M.translate(0,0,-mSize*.01);



//}

