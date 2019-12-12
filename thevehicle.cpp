#include "thevehicle.h"

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



void theVehicle::create()
{
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

    mOSGSphere   = new osg::Sphere( osg::Vec3( 0.f, 0.f, 0.f ), mRadius);
    osg::ShapeDrawable* sd = new osg::ShapeDrawable( mOSGSphere );
    sd->setColor(  osg::Vec4(mColor[0], mColor[1], mColor[2],mColor[3]));
    sd->setName( "Sphere" );

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( sd );

    osg::StateSet* stateSet = geode->getOrCreateStateSet();
    osg::Material* material = new osg::Material;
    material->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );
    stateSet->setAttributeAndModes( material, osg::StateAttribute::ON );
    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );

    mModel = geode;
    mTransform->addChild(geode);
}

void theVehicle::set_velocity(btVector3 velocityIncrease)
{
    btVector3 currentVelocity = rigidBody->getLinearVelocity();
    btVector3 newVelocity = currentVelocity + velocityIncrease;
    rigidBody->setLinearVelocity(newVelocity);
}

void theVehicle::destroy()
{
    delete rigidBody->getMotionState();
    delete rigidBody;
    delete mSphereShape;
    delete inertia;
    delete rigidCI;
}

theVehicle::theVehicle()
{
    mColor=QVector4D(0,0,1,1);
    mResetPosition=QVector3D(100,100,0);
    mMass = 10;
    mRadius = 10;
    create();
}

theVehicle::theVehicle(QVector3D &initialPosition, QVector4D &color, double mass, double radius)
{
    mResetPosition=initialPosition;
    mColor=color;
    mMass = mass;
    mRadius = radius;
    create();
}

theVehicle::~theVehicle()
{
    destroy();
}
