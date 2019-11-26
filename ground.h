//-------------------------------------------------------
// Filename: Ground.h
//
// Description:  The h file for the qt5 bullet bouncy ball example.
//
// Creator:  Professor Corey McBride for MEEN 570 - Brigham Young University
//
// Creation Date: 11/7/18
//
// Owner: Corey McBride
//-------------------------------------------------------

#ifndef GROUND_H
#define GROUND_H
#include <QMatrix4x4>
#include "btBulletDynamicsCommon.h"
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

class Ground
{
public:
    Ground();
    Ground(int size,QVector4D& color);
    btRigidBody* getRigidBodyPtr() {return mRigidBody;}
    osg::Node* getNode() {return mTransform.release();}

    void draw(QMatrix4x4 &V_matrix, QMatrix4x4 &P_matrix);
private:
    double mSize;
    btCollisionShape* mGroundShape;
    btDefaultMotionState* mGroundMotionState;
    btRigidBody* mRigidBody;
    btRigidBody::btRigidBodyConstructionInfo* mRigidCI;
    btScalar btMat[16];
    QMatrix4x4  M;
    btTransform trans;
    QVector4D mColor;

    void create();
    void destroy();
    void create_mesh();

    osg::ref_ptr<osg::MatrixTransform> mTransform;
    osg::ref_ptr<osg::Box> mOSGBox;
};

#endif // GROUND_H
