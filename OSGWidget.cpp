#include "OSGWidget.h"
#include "randomObstacles.h"
#include "pathfinder.h"

#include <osg/Camera>
#include <osg/DisplaySettings>
#include <osg/Geode>
#include <osg/Material>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>

#include <osgGA/EventQueue>
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>

#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>

#include <vector>
#include <fstream>
#include <random>

#include <QDebug>
#include <QKeyEvent>
#include <QPainter>
#include <QWheelEvent>


OSGWidget::OSGWidget( QWidget* parent,
                      Qt::WindowFlags f )
    : QOpenGLWidget( parent,f )
    , mGraphicsWindow( new osgViewer::GraphicsWindowEmbedded( this->x(),
                                                              this->y(),
                                                              this->width(),
                                                              this->height() ) )
    , mViewer( new osgViewer::CompositeViewer )
    , mBusy{false}
{
    set_up_physics();

    mStarted=false;
    
    mRoot = new osg::Group;
    setup_environment();
    set_camera_view();

    this->setFocusPolicy( Qt::StrongFocus );
    this->setMinimumSize( 400, 400 );
    this->setMouseTracking( false );
    onHome();
    start_timer();
}


void OSGWidget::run_auto_path()
{
    mVehicle->getRigidBodyPtr()->setLinearVelocity(btVector3(0,0,0));
    std::vector<std::vector<bool>> arenaStatusMap = newArenaMap->return_the_map();
    pathFinder newPath(arenaStatusMap,static_cast<size_t>(mSizeGround),xStart,yStart,
                       static_cast<size_t>(xGoalPosition),static_cast<size_t>(yGoalPosition));
    autoPath = newPath.return_path();
    float counterIsPathPossible{1};
    radiusVirtual = 10;
    while(!autoPath.empty())
    {

       autoCoordinates = autoPath.front();
//       reveal_path();
//       mVehicle->set_position(autoCoordinates);
       sphere_travel(counterIsPathPossible);
       autoPath.pop();
       counterIsPathPossible++;
    }
    if (counterIsPathPossible<10)
    {
        camera->setClearColor( osg::Vec4(.541f, 0.18f, 0.192f, 1.f) );
    }
    else
    {
        camera->setClearColor( osg::Vec4(0.949f,0.8157f,0.4196f,1.f));
    }
}


void OSGWidget::reveal_path()
{
    float pathX = autoCoordinates[0];
    float pathY = autoCoordinates[1];
    pathGeode = new osg::Geode;
    osg::Vec3 positionOSG{pathX, pathY, 5.f};
    pathBox = new  osg::Box ( positionOSG, 5.f,5.f,.05f);
    osg::ShapeDrawable* sd = new osg::ShapeDrawable( pathBox );
    sd->setColor(osg::Vec4(0.f,0.6f,0.f,1.f));
    pathGeode->addDrawable( sd );
    osg::StateSet* stateSet = pathGeode->getOrCreateStateSet();
    osg::Material* material = new osg::Material;
    material->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );
    stateSet->setAttributeAndModes( material, osg::StateAttribute::ON );
    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
    mRoot->addChild(pathGeode);
}


void OSGWidget::set_up_physics()
{
    mBroadphaseInterface = new btDbvtBroadphase();
    mDefaultCollisionConfig = new btDefaultCollisionConfiguration();
    mCollisionDispatcher = new btCollisionDispatcher(mDefaultCollisionConfig);
    mSeqImpConstraintSolver = new btSequentialImpulseConstraintSolver;
    mDynamicsWorld = new btDiscreteDynamicsWorld(mCollisionDispatcher, mBroadphaseInterface,
                                                 mSeqImpConstraintSolver, mDefaultCollisionConfig);
    mDynamicsWorld->setGravity(btVector3(0, 0, -1000));
    mStartTick = osg::Timer::instance()->tick();
    mTimeStep = 1/60.0;
}


void OSGWidget::setup_environment()
{
    make_ground();
    vehicleStartPosition=QVector3D(500,7,18);
    vehicleColor =QVector4D(1,1,1,1);
    mVehicle=new theVehicle(vehicleStartPosition, vehicleColor, 100, 10);
    mDynamicsWorld->addRigidBody(mVehicle->getRigidBodyPtr());
    mRoot->addChild(mVehicle->getTransform());
    mVehicle->getRigidBodyPtr()->setLinearVelocity(btVector3(1,5,20));
    mBusy=true;
}


void OSGWidget::run_manual()
{
    mVehicle->getRigidBodyPtr()->setLinearVelocity(btVector3(1,5,100));
    btVector3 currentVelcheck = mVehicle->getRigidBodyPtr()->getLinearVelocity();
}


void OSGWidget::create_obstacles(int numberOfObstacles)
{
    set_goal();
    obstaclesCreated = true;
    for (int i{0}; i<numberOfObstacles; i++)
    {
        randomObstacles newRandomObstacle;
        mObstacleBox = newRandomObstacle.generate_random_obstacle(mSizeGround);
        mDynamicsWorld->addRigidBody(mObstacleBox->getRigidBodyPtr());
        mRoot->addChild(mObstacleBox->getNode());
        newArenaMap->add_to_obstacle_matrix(newRandomObstacle.create_obstacle_area_matrix());
    }
}


void OSGWidget::make_ground()
{
    QVector4D ground_color(0.18f,0.349f,0.2274f,1);

    mGround= new boundingBox(mSizeGround,ground_color);
    mRoot->addChild(mGround->getNode());
    mDynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());

    btVector3 sidePositionXZ1 = {mSizeGround*.5f,-mSizeGround*.005f,mSizeGround*.5f};
    mGround->create_sides_xz(sidePositionXZ1);
    mRoot->addChild(mGround->getNode());
    mDynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());

    btVector3 sidePositionYZ1 = {-mSizeGround*.005f,mSizeGround*.5f,mSizeGround*.5f};
    mGround->create_sides_yz(sidePositionYZ1);
    mRoot->addChild(mGround->getNode());
    mDynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());

    btVector3 sidePositionYZ2 = {mSizeGround*1.005f,mSizeGround*.5f,mSizeGround*.5f};
    mGround->create_sides_yz(sidePositionYZ2);
    mRoot->addChild(mGround->getNode());
    mDynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());

    newArenaMap = new arenaNodeMap(static_cast<size_t>(mSizeGround));
}


void OSGWidget::set_goal()
{
    goalGeode = new osg::Geode;
    osg::Vec3 positionOSG{xGoalPosition, yGoalPosition, 50.f};
    goalBox = new  osg::Box ( positionOSG, mSizeGround,sizeGoal,sizeGoal*5.f);
    osg::ShapeDrawable* sd = new osg::ShapeDrawable( goalBox );
    sd->setColor(osg::Vec4(0.404f,0.651f,0.4667f,1.f));
    goalGeode->addDrawable( sd );
    osg::StateSet* stateSet = goalGeode->getOrCreateStateSet();
    osg::Material* material = new osg::Material;
    material->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );
    stateSet->setAttributeAndModes( material, osg::StateAttribute::ON );
    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
    mRoot->addChild(goalGeode);
}


void OSGWidget::sphere_travel(float counterRadius)
{
    radiusVirtual = radiusVirtual - (1.f/counterRadius);
    float pathX = autoCoordinates[0];
    float pathY = autoCoordinates[1];
    pathGeode = new osg::Geode;
    osg::Vec3 positionOSG{pathX, pathY, 15.f};
    pathSphere = new  osg::Sphere ( positionOSG, radiusVirtual);
    osg::ShapeDrawable* sd = new osg::ShapeDrawable( pathSphere );
    sd->setColor(osg::Vec4(0.949f,0.8157f,0.4196f,1.f));
    pathGeode->addDrawable( sd );
    osg::StateSet* stateSet = pathGeode->getOrCreateStateSet();
    osg::Material* material = new osg::Material;
    material->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );
    stateSet->setAttributeAndModes( material, osg::StateAttribute::ON );
    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
    mRoot->addChild(pathGeode);
}


void OSGWidget::reset_world()
{
    if(mBusy)
    {
        while(mRoot->getNumChildren())
        {
            osg::Node* child=mRoot->getChild(0);
            mRoot->removeChild(child);

        }
        delete mDynamicsWorld;
        mDynamicsWorld=nullptr;
        delete mVehicle;
        mVehicle=nullptr;
        if (obstaclesCreated)
        {
            delete mObstacleBox;
            mObstacleBox = nullptr;
        }
        delete mGround;
        mGround = nullptr;

        goalBox = nullptr;
        pathBox = nullptr;

        set_up_physics();
    }
    setup_environment();
    dynamic_cast<osgGA::NodeTrackerManipulator*>(mManipulator.get())->setTrackNode(mVehicle->getModel());
    camera->setClearColor( osg::Vec4( 0.6f, 0.6f, 0.6f, 1.f ) );
    obstaclesCreated = false;
    winStatus = false;
}


void OSGWidget::timerEvent(QTimerEvent *)
{
    update();
    currentPosition =  mVehicle->getRigidBodyPtr()->getCenterOfMassPosition();
    float currentPositionY = currentPosition.getY();
    float currentPositionZ = currentPosition.getZ();

    if (winStatus == false && currentPositionY >yGoalPosition)
    {
        mVehicle->getRigidBodyPtr()->setLinearVelocity(btVector3(0,0,1000));
        camera->setClearColor( osg::Vec4(0.949f,0.8157f,0.4196f,1.f));
        winStatus = true;
    }
    if (currentPositionZ < 0 && winStatus == false)
    {
        camera->setClearColor( osg::Vec4(0.541f, 0.18f, 0.192f, 1.f));
    }

}


void OSGWidget::start_timer()
{
    mStarted=true;
    // And, start the timer.
    mTimerId=startTimer(static_cast<int>(mTimeStep) * 1000);
}


void OSGWidget::stop_timer()
{
    if(mStarted)
    {
        killTimer(mTimerId);
        mTimerId=-1;
        mStarted=false;
    }
}


void OSGWidget::set_camera_view()
{
    double aspectRatio = static_cast<double>( this->width() ) / static_cast<double>( this->height() );
    auto pixelRatio   = this->devicePixelRatio();

    camera = new osg::Camera;
    camera->setViewport( 0, 0, this->width() * pixelRatio, this->height() * pixelRatio );
    camera->setClearColor( osg::Vec4( 0.6f, 0.6f, 0.6f, 1.f ) );
    camera->setProjectionMatrixAsPerspective( 30., aspectRatio, 1., 1000. );
    camera->setGraphicsContext( mGraphicsWindow );

    view = new osgViewer::View;
    view->setCamera( camera );
    view->setSceneData( mRoot.get() );
    view->addEventHandler( new osgViewer::StatsHandler );

    osgGA::NodeTrackerManipulator *manipulator {new osgGA::NodeTrackerManipulator};
    manipulator = new osgGA::NodeTrackerManipulator;
    manipulator->setHomePosition(osg::Vec3d(500,-1250,1000),osg::Vec3d(500,0,0),osg::Vec3d(0,0,1));
    manipulator->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER);
    manipulator->setTrackNode(mVehicle->getModel());
    mManipulator = manipulator;
    view->setCameraManipulator( mManipulator );
    mViewer->addView( view );
    mViewer->setThreadingModel( osgViewer::CompositeViewer::SingleThreaded );
    mViewer->realize();
}


void OSGWidget::arrow_key_velocity_update(int arrowDirection)
{
    if (arrowDirection == 1)
    {
        velocityIncrease = btVector3(0,30,0);
        mVehicle->set_velocity(velocityIncrease);
    }
    if (arrowDirection == 2)
    {
        velocityIncrease = btVector3(0,-30,0);
        mVehicle->set_velocity(velocityIncrease);
    }
    if (arrowDirection == 3)
    {
        velocityIncrease = btVector3(-30,0,0);
        mVehicle->set_velocity(velocityIncrease);
    }
    if (arrowDirection == 4)
    {
        velocityIncrease = btVector3(30,0,0);
        mVehicle->set_velocity(velocityIncrease);
    }
    if (arrowDirection == 5)
    {
        velocityIncrease = btVector3(0,0,100);
        mVehicle->set_velocity(velocityIncrease);
    }

}


void OSGWidget::keyPressEvent( QKeyEvent* event )
{
    QString keyString   = event->text();
    const char* keyData = keyString.toLocal8Bit().data();

    if( event->key() == Qt::Key_H )
    {
        this->onHome();
        return;
    }
    if(event->key() == Qt::Key_Up)
    {
        int arrowUp{1};
        arrow_key_velocity_update(arrowUp);
    }
    if(event->key() == Qt::Key_Down)
    {
        int arrowDown{2};
        arrow_key_velocity_update(arrowDown);
    }
    if(event->key() == Qt::Key_Left)
    {
        int arrowLeft{3};
        arrow_key_velocity_update(arrowLeft);
    }
    if(event->key() == Qt::Key_Right)
    {
        int arrowRight{4};
        arrow_key_velocity_update(arrowRight);
    }
    if(event->key() == Qt::Key_J)
    {
        int keyJ{5};
        arrow_key_velocity_update(keyJ);
    }

    this->getEventQueue()->keyPress( osgGA::GUIEventAdapter::KeySymbol( *keyData ) );
}


void OSGWidget::keyReleaseEvent( QKeyEvent* event )
{
    QString keyString   = event->text();
    const char* keyData = keyString.toLocal8Bit().data();

    this->getEventQueue()->keyRelease( osgGA::GUIEventAdapter::KeySymbol( *keyData ) );
}


void OSGWidget::paintGL()
{
    if(mStarted)
    {
        osg::Timer_t now_tick = osg::Timer::instance()->tick();
        float dt = osg::Timer::instance()->delta_s(mStartTick, now_tick);
        mStartTick = now_tick;
        /* int numSimSteps = */
        mDynamicsWorld->stepSimulation(dt); //, 10, 0.01);
        mDynamicsWorld->updateAabbs();
    }
    mViewer->frame();
}


void OSGWidget::resizeGL( int width, int height )
{
    this->getEventQueue()->windowResize( this->x(), this->y(), width, height );
    mGraphicsWindow->resized( this->x(), this->y(), width, height );
    this->on_resize( width, height );
}


void OSGWidget::mouseMoveEvent( QMouseEvent* event )
{

    auto pixelRatio = this->devicePixelRatio();

    this->getEventQueue()->mouseMotion( static_cast<float>( event->x() * pixelRatio ),
                                        static_cast<float>( event->y() * pixelRatio ) );
}


void OSGWidget::mousePressEvent( QMouseEvent* event )
{
    // 1 = left mouse button
    // 2 = middle mouse button
    // 3 = right mouse button
    unsigned int button = 0;
    switch( event->button() )
    {
    case Qt::LeftButton:
        button = 1;
        break;

    case Qt::MiddleButton:
        button = 2;
        break;

    case Qt::RightButton:
        button = 3;
        break;

    default:
        break;
    }
    auto pixelRatio = this->devicePixelRatio();
    this->getEventQueue()->mouseButtonPress( static_cast<float>( event->x() * pixelRatio ),
                                             static_cast<float>( event->y() * pixelRatio ),
                                             button );
}


void OSGWidget::mouseReleaseEvent(QMouseEvent* event)
{
    // 1 = left mouse button
    // 2 = middle mouse button
    // 3 = right mouse button
    unsigned int button = 0;
    switch( event->button() )
    {
    case Qt::LeftButton:
        button = 1;
        break;
    case Qt::MiddleButton:
        button = 2;
        break;
    case Qt::RightButton:
        button = 3;
        break;
    default:
        break;
    }
    auto pixelRatio = this->devicePixelRatio();
    this->getEventQueue()->mouseButtonRelease( static_cast<float>( pixelRatio * event->x() ),
                                               static_cast<float>( pixelRatio * event->y() ),
                                               button );
}


void OSGWidget::wheelEvent( QWheelEvent* event )
{
    event->accept();
    int delta = event->delta();
    osgGA::GUIEventAdapter::ScrollingMotion motion = delta > 0 ?   osgGA::GUIEventAdapter::SCROLL_UP
                                                                 : osgGA::GUIEventAdapter::SCROLL_DOWN;
    this->getEventQueue()->mouseScroll( motion );
}


bool OSGWidget::event( QEvent* event )
{
    bool handled = QOpenGLWidget::event( event );
    // This ensures that the OSG widget is always going to be repainted after the
    // user performed some interaction. Doing this in the event handler ensures
    // that we don't forget about some event and prevents duplicate code.
    switch( event->type() )
    {
    case QEvent::KeyPress:
    case QEvent::KeyRelease:
    case QEvent::MouseButtonDblClick:
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    case QEvent::MouseMove:
    case QEvent::Wheel:
        this->update();
        break;
    default:
        break;
    }
    return handled;
}


void OSGWidget::paintEvent( QPaintEvent* /* paintEvent */ )
{
    this->makeCurrent();
    QPainter painter( this );
    painter.setRenderHint( QPainter::Antialiasing );
    this->paintGL();
    painter.end();
    this->doneCurrent();
}


void OSGWidget::onHome()
{
    osgViewer::ViewerBase::Views views;
    mViewer->getViews( views );
    for( std::size_t i = 0; i < views.size(); i++ )
    {
        osgViewer::View* view = views.at(i);
        view->home();
    }
}


void OSGWidget::on_resize( int width, int height )
{
    std::vector<osg::Camera*> cameras;
    mViewer->getCameras( cameras );
    auto pixelRatio = this->devicePixelRatio();
    cameras[0]->setViewport( 0, 0, width * pixelRatio, height * pixelRatio );
}


osgGA::EventQueue* OSGWidget::getEventQueue() const
{
    osgGA::EventQueue* eventQueue = mGraphicsWindow->getEventQueue();
    if( eventQueue )
        return eventQueue;
    else
        throw std::runtime_error( "Unable to obtain valid event queue");
}


OSGWidget::~OSGWidget()
{
}
