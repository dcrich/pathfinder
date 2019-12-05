#ifndef OBSTACLEBOXES_H
#define OBSTACLEBOXES_H
#include <QMatrix4x4>
#include "btBulletDynamicsCommon.h"
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

class obstacleBoxes
{
public:
    obstacleBoxes(std::vector<float> obstacleSize,std::vector<float> obstaclePosition);
    btRigidBody* getRigidBodyPtr() {return mRigidBody;}
    osg::Node* getNode() {return mTransform.release();}
    ~obstacleBoxes();

private:
    float mSizeX;
    float mSizeY;
    float mSizeZ;
    float mPositionX;
    float mPositionY;
    float mPositionZ;
    btCollisionShape* mObstacleShape;
    btDefaultMotionState* mObstacleMotionState;
    btRigidBody* mRigidBody {nullptr};
    btRigidBody::btRigidBodyConstructionInfo* mRigidConstructionInfo;
    btScalar btMat[16];
    QMatrix4x4  M;
    btTransform trans;
    QVector4D mColor{0.5,0,0,1.0};

    void create_physics_box();
    void destroy();
    void create_osg_box();

    osg::ref_ptr<osg::MatrixTransform> mTransform;
    osg::ref_ptr<osg::Box> mOSGBox;

};

#endif // OBSTACLEBOXES_H
