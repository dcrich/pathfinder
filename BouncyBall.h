//-------------------------------------------------------
// Filename: BouncyBall.h
//
// Description:  The h file for the qt5 opengl bullet bouncy ball example.
//
// Creator:  Professor Corey McBride for MEEN 570 - Brigham Young University
//
// Creation Date: 11/7/18
//
// Owner: Corey McBride
//-------------------------------------------------------
#ifndef BOUNCYBALL_H
#define BOUNCYBALL_H

#include "btBulletDynamicsCommon.h"
#include <QMatrix4x4>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>



class BouncyBall
{
public:
    BouncyBall();
    BouncyBall(QVector3D& initalPosition, QVector4D& mColor,
               double mMass, double mRadius);
    ~BouncyBall();

    btRigidBody* getRigidBodyPtr() {return rigidBody;}
    osg::Node* getNode() {return mTransform.release();}
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
    void copy(const BouncyBall & other);
    void destroy();

    osg::ref_ptr<osg::MatrixTransform> mTransform;
    osg::ref_ptr<osg::Sphere> mOSGSphere;
};

#endif // BOUNCYBALL_H
