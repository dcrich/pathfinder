#ifndef THEVEHICLE_H
#define THEVEHICLE_H

#include "btBulletDynamicsCommon.h"
#include <QMatrix4x4>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

class theVehicle
{

public:
    theVehicle();
    theVehicle(QVector3D& initalPosition, QVector4D& mColor,
               double mMass, double mRadius);
    ~theVehicle();

    btRigidBody* getRigidBodyPtr() {return rigidBody;}
    osg::ref_ptr<osg::Node> getModel() {return mModel;}
    osg::ref_ptr<osg::Node> getTransform() {return mTransform;}
    void set_velocity(btVector3 velocityIncrease);
private:
    std::vector<GLfloat> mColors;
    btCollisionShape* mSphereShape;
    btMotionState* motionState;
    btRigidBody* rigidBody;
    btRigidBody::btRigidBodyConstructionInfo* rigidCI;


    QMatrix4x4 mMVP;

    QVector3D mResetPosition;
    QVector4D mColor;
    double mMass, mRadius;

    btVector3* inertia;

    btScalar btMat[16];
    QMatrix4x4  M;
    btTransform trans;

    void create();
    void destroy();

    osg::ref_ptr<osg::Node> mModel;
    osg::ref_ptr<osg::MatrixTransform> mTransform;
    osg::ref_ptr<osg::Sphere> mOSGSphere;
};


#endif // THEVEHICLE_H
