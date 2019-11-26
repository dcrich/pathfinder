//-------------------------------------------------------
// Filename: OpenGLMainWidget.cpp
//
// Description:  The cpp file for the qt5 bullet bouncy ball example.
//
// Creator:  Professor Corey McBride for MEEN 570 - Brigham Young University
//
// Creation Date: 11/7/18
//
// Owner: Corey McBride
//-------------------------------------------------------
#include "BouncyBall.h"
#include <osg/Geometry>
#include <iostream>
#include <osg/Material>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>

class BallUpdateCallback: public osg::NodeCallback
{
private:
    btRigidBody *_body;

public:
    BallUpdateCallback(btRigidBody *body) :
        _body(body)
    {
    }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        btScalar m[16];

        btDefaultMotionState* myMotionState = (btDefaultMotionState*) _body->getMotionState();
        myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);

        osg::Matrixf mat(m);

        osg::MatrixTransform *pat = dynamic_cast<osg::MatrixTransform *> (node);
        pat->setMatrix(mat);

        traverse(node, nv);
    }
};



void BouncyBall::create()
{

    // This part of the code sets up all the bullet stuff.
    // It only applies to the ball itself.
    mSphereShape = new btSphereShape(mRadius);

    motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),
                                                       btVector3(mResetPosition[0], mResetPosition[1], mResetPosition[2])));
    inertia = new btVector3(0, 0, 0);
    mSphereShape->calculateLocalInertia(mMass, *inertia);

    rigidCI = new btRigidBody::btRigidBodyConstructionInfo(mMass, motionState, mSphereShape, *inertia);
    rigidCI->m_restitution = 0.9;

    rigidBody = new btRigidBody(*rigidCI);

    mTransform = new osg::MatrixTransform;
    mTransform->setUpdateCallback(new BallUpdateCallback(rigidBody));



    mOSGSphere   = new osg::Sphere( osg::Vec3( 0.f, 0.f, 0.f ), mRadius );
    osg::ShapeDrawable* sd = new osg::ShapeDrawable( mOSGSphere );
    sd->setColor(  osg::Vec4(mColor[0], mColor[1], mColor[2],mColor[3]));
    sd->setName( "Sphere" );

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( sd );

    // Set material for basic lighting and enable depth tests. Else, the sphere
    // will suffer from rendering errors.
    {
        osg::StateSet* stateSet = geode->getOrCreateStateSet();
        osg::Material* material = new osg::Material;

        material->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );

        stateSet->setAttributeAndModes( material, osg::StateAttribute::ON );
        stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
    }

    mTransform->addChild(geode);
}

void BouncyBall::destroy()
{
    delete rigidBody->getMotionState();
    delete rigidBody;
    delete mSphereShape;

    delete inertia;
    delete rigidCI;
}

BouncyBall::BouncyBall()
{
    mColor=QVector4D(0,0,1,1);
    mResetPosition=QVector3D(100,100,0);
    mMass = 10;
    mRadius = 10;
    create();
}

BouncyBall::BouncyBall(QVector3D &initialPosition, QVector4D &color, double mass, double radius)
{
    mResetPosition=initialPosition;
    mColor=color;
    mMass = mass;
    mRadius = radius;

    create();
}

BouncyBall::~BouncyBall()
{
    destroy();
}



