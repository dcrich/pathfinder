#include "boundingbox.h"
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>


boundingBox::boundingBox(float size, QVector4D& color)
{
    mSize=size;
    baseCenter=mSize*.5f;
    mColor=color;

    bottomPosition = {mSize*.5f,mSize*.5f,0};
    create_sides_xy(bottomPosition);

    create_mesh();
    make_boundary_box(mSize);
}

void boundingBox::create_sides_xy(btVector3 positionPhysics)
{
    //btVector3 positionPhysics{mSize*.5f,mSize*.5f,0};
    mGroundShape = new btBoxShape(btVector3(mSize*.5,mSize*.5,mSize*.005));
    mGroundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),positionPhysics));
    mRigidCI= new btRigidBody::btRigidBodyConstructionInfo(0,mGroundMotionState,mGroundShape,btVector3(0,0,0));
    mRigidCI->m_restitution = 0.9;
    mRigidBody = new btRigidBody(*mRigidCI);
}
void boundingBox::create_mesh()
{
    osg::Vec3 positionOSG{baseCenter, baseCenter, 0.f};
    mOSGBox  = new osg::Box( positionOSG, mSize,mSize,mSize*.01 );
    osg::ShapeDrawable* sd = new osg::ShapeDrawable( mOSGBox );
    sd->setColor(  osg::Vec4(mColor[0], mColor[1], mColor[2],mColor[3]));

    geode = new osg::Geode;
    geode->addDrawable( sd );


    osg::StateSet* stateSet = geode->getOrCreateStateSet();
    osg::Material* material = new osg::Material;

    material->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );

    stateSet->setAttributeAndModes( material, osg::StateAttribute::ON );
    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );

    mTransform = new osg::MatrixTransform;

    mTransform->addChild(geode);

}

void boundingBox::destroy()
{
    delete mRigidBody;
    delete mRigidCI;
    delete mGroundShape;
    delete mGroundMotionState;
}

void boundingBox::make_boundary_box(float mSize)
{
    bgeode = new osg::Geode;
    osg::Vec4 osgVec4color(0.5f, 0.f, 0.f, 1.f);
    osg::Vec3 osgVec3dscaleFactor(1.f, 1.f, 1.f);
    osg::Vec3Array* v = new osg::Vec3Array;
    v->resize( 8 );
    (*v)[0].set( 0.f, 0.f, mSize);
    (*v)[1].set(mSize, 0.f, mSize);
    (*v)[2].set(mSize, mSize, mSize );
    (*v)[3].set(0.f, mSize, mSize);
    (*v)[4].set(0.f, 0.f, 0.f );
    (*v)[5].set(mSize, 0.f, 0.f );
    (*v)[6].set(mSize, mSize, 0.f );
    (*v)[7].set(0.f, mSize, 0.f);

    osg::Geometry* geom = new osg::Geometry;
    geom->setUseDisplayList( false );
    geom->setVertexArray( v );

    osg::Vec4Array* c = new osg::Vec4Array;
    c->push_back( osgVec4color );
    geom->setColorArray( c, osg::Array::BIND_OVERALL );

    GLushort idxLoops1[4] = {0, 1, 2, 3};
    GLushort idxLoops2[4] = {4, 5, 6, 7};
    GLushort idxLoops3[4] = {0, 1, 5, 4};
    GLushort idxLoops4[4] = {3, 2, 6, 7};
    GLushort idxLoops5[4] = {1, 2, 6, 5};
    GLushort idxLoops6[4] = {0, 3, 7, 4};
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops1 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::POLYGON, 4, idxLoops2 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops3 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops4 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops5 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops6 ) );


    bgeode->addDrawable( geom );

    bgeode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    bgeode->getOrCreateStateSet()->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
    mTransform->addChild(bgeode);
    osg::StateSet* mstateSet = bgeode->getOrCreateStateSet();
    osg::Material* mmaterial = new osg::Material;

    mmaterial->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );

    mstateSet->setAttributeAndModes( mmaterial, osg::StateAttribute::ON );
    mstateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
}

void boundingBox::create_sides_xz( btVector3 positionPhysics)
{
    mGroundShape = new btBoxShape(btVector3(mSize*.5f,mSize*.005f,mSize*.5f));
    mGroundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),positionPhysics));
    mRigidCI= new btRigidBody::btRigidBodyConstructionInfo(0,mGroundMotionState,mGroundShape,btVector3(0,0,0));
    mRigidCI->m_restitution = 0.9;
    mRigidBody = new btRigidBody(*mRigidCI);
}

void boundingBox::create_sides_yz( btVector3 positionPhysics)
{
    mGroundShape = new btBoxShape(btVector3(mSize*.005f,mSize*.5f,mSize*.5f));
    mGroundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),positionPhysics));
    mRigidCI= new btRigidBody::btRigidBodyConstructionInfo(0,mGroundMotionState,mGroundShape,btVector3(0,0,0));
    mRigidCI->m_restitution = 0.9;
    mRigidBody = new btRigidBody(*mRigidCI);
}





