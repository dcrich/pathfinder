#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H
#include <QMatrix4x4>
#include "btBulletDynamicsCommon.h"
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

class boundingBox
{
public:
    boundingBox(float size,QVector4D& color);
    btRigidBody* getRigidBodyPtr() {return mRigidBody;}
    osg::Node* getNode() {return mTransform.release();}
    void create_sides_xz(btVector3 positionPhysics);
    void create_sides_yz(btVector3 positionPhysics);


private:
    float mSize;
    btCollisionShape* mGroundShape;
    btDefaultMotionState* mGroundMotionState;
    btRigidBody* mRigidBody;
    btRigidBody::btRigidBodyConstructionInfo* mRigidCI;
    btScalar btMat[16];
    QMatrix4x4  M;
    btTransform trans;
    QVector4D mColor;
    osg::Geode* geode;
    osg::Geode* bgeode;
    float baseCenter;
    btVector3 topPosition;
    btVector3 sidePositionYZ1;
    btVector3 sidePositionYZ2;
    btVector3 sidePositionXZ1;
    btVector3 sidePositionXZ2;
    btVector3 bottomPosition;



    void create_sides_xy(btVector3 positionPhysics);
    void destroy();
    void create_mesh();
    void make_boundary_box(float mSize);

    osg::ref_ptr<osg::MatrixTransform> mTransform;
    osg::ref_ptr<osg::Box> mOSGBox;
};

#endif // BOUNDINGBOX_H
