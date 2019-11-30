#include "OSGWidget.h"

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

#include <QDebug>
#include <QKeyEvent>
#include <QPainter>
#include <QWheelEvent>

#include <random>



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
    
    mManipulator = new osgGA::TrackballManipulator;
    mManipulator->setAllowThrow( false );
    
    view->setCameraManipulator( mManipulator );
    mManipulator->setHomePosition(osg::Vec3d(3000,1500,1900),osg::Vec3d(0,0,0),osg::Vec3d(0,0,1));
    
    
    
    mViewer->addView( view );
    mViewer->setThreadingModel( osgViewer::CompositeViewer::SingleThreaded );
    mViewer->realize();
    // This ensures that the widget will receive keyboard events. This focus
    // policy is not set by default. The default, Qt::NoFocus, will result in
    // keyboard events that are ignored.
    this->setFocusPolicy( Qt::StrongFocus );
    this->setMinimumSize( 400, 400 );
    
    // Ensures that the widget receives mouse move events even though no
    // mouse button has been pressed. We require this in order to let the
    // graphics window switch viewports properly.
    this->setMouseTracking( false );
    onHome();
    
    start_timer();
    
    
}

void OSGWidget::setup_camera()
{
    
}

OSGWidget::~OSGWidget()
{
}

void OSGWidget::set_up_physics()
{
    // The BulletWidget owns and controls everything to do with
    // the dynamics world. This call allocates the solvers
    // and collision objects, and sets the gravity.
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

void OSGWidget::arrow_key_velocity_update(int arrowDirection)
{
    btVector3 velocityIncrease;
    if (arrowDirection == 1)
    {
        velocityIncrease = btVector3(0,100,0);
    }
    if (arrowDirection == 2)
    {
        velocityIncrease = btVector3(0,-100,0);
    }
    if (arrowDirection == 3)
    {
        velocityIncrease = btVector3(100,0,0);
    }
    if (arrowDirection == 4)
    {
        velocityIncrease = btVector3(-100,0,0);
    }
    mBouncyBall->set_velocity(velocityIncrease);
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

void OSGWidget::setup_environment()
{
    make_ground();
    QVector3D pos;
    QVector4D color;
    pos=QVector3D(10,10,10); // starting position of ball
    color =QVector4D(1,1,1,1);
    mBouncyBall=new BouncyBall(pos, color, 100, 100);
    mDynamicsWorld->addRigidBody(mBouncyBall->getRigidBodyPtr());
    mRoot->addChild(mBouncyBall->getNode());
    
    mBusy=true;
    
    
}
void OSGWidget::make_balls()
{
    QVector3D pos;
    QVector4D color;
    pos=QVector3D(500,500,800); // starting position of ball
    color =QVector4D(1,1,1,1);
    mBouncyBall=new BouncyBall(pos, color, 100, 10);

    mDynamicsWorld->addRigidBody(mBouncyBall->getRigidBodyPtr());
    mRoot->addChild(mBouncyBall->getNode());
}
void OSGWidget::timerEvent(QTimerEvent *)
{
    update();
}

void OSGWidget::reset_world()
{

    if(mBusy)
    {
        //stop_timer();
        while(mRoot->getNumChildren())
        {
            osg::Node* child=mRoot->getChild(0);
            mRoot->removeChild(child);
            
        }
        
        delete mDynamicsWorld;
        mDynamicsWorld=nullptr;
        
        delete mBouncyBall;
        mBouncyBall=nullptr;
        delete mObstacleBox;
        mObstacleBox = nullptr;
        
        delete mGround;
        mGround = nullptr;
        set_up_physics();
    }
    make_ground();
    
}
void OSGWidget::create_obstacles(int numberOfObstacles, int sizeOfObstacles)
{
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<float> dis(0,mSizeGround);
    for (int i{0}; i<numberOfObstacles; i++)
    {
        float x = dis(generator);
        float y = dis(generator);
        float z = 0.f;
        float sizeX = dis(generator)*0.1f*sizeOfObstacles;
        float sizeY = dis(generator)*0.1f*sizeOfObstacles;
        float sizeZ = dis(generator)*0.1f*sizeOfObstacles;
        std::vector<float> obstacleSize{sizeX,sizeY,sizeZ};
        std::vector<float> obstaclePosition{x,y,z};
        mObstacleBox= new obstacleBoxes(obstacleSize, obstaclePosition);
        mDynamicsWorld->addRigidBody(mObstacleBox->getRigidBodyPtr());
        mRoot->addChild(mObstacleBox->getNode());

    }
    //    std::vector<float> obstacleSize{400.f,400.f,400.f};
    //    std::vector<float> obstaclePosition{0.f,0.f,0.f};
    //    mObstacleBox= new obstacleBoxes(obstacleSize, obstaclePosition);
    //    mRoot->addChild(mObstacleBox->getNode());
    //    mDynamicsWorld->addRigidBody(mObstacleBox->getRigidBodyPtr());


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
    
    this->getEventQueue()->keyPress( osgGA::GUIEventAdapter::KeySymbol( *keyData ) );
}

void OSGWidget::keyReleaseEvent( QKeyEvent* event )
{
    QString keyString   = event->text();
    const char* keyData = keyString.toLocal8Bit().data();
    
    this->getEventQueue()->keyRelease( osgGA::GUIEventAdapter::KeySymbol( *keyData ) );
}

void OSGWidget::make_ground()
{
    QVector4D ground_color(.5,0.0,.5,1);

    mGround= new boundingBox(mSizeGround,ground_color);
    mRoot->addChild(mGround->getNode());
    mDynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());

    btVector3 sidePositionXZ1 = {mSizeGround*.5f,0.f,mSizeGround*.5f};
    mGround->create_sides_xz(sidePositionXZ1);
    mRoot->addChild(mGround->getNode());
    mDynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());

    btVector3 sidePositionXZ2 = {mSizeGround*.5f,mSizeGround,mSizeGround*.5f};
    mGround->create_sides_xz(sidePositionXZ2);
    mRoot->addChild(mGround->getNode());
    mDynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());

    btVector3 sidePositionYZ1 = {0.f,mSizeGround*.5f,mSizeGround*.5f};
    mGround->create_sides_yz(sidePositionYZ1);
    mRoot->addChild(mGround->getNode());
    mDynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());

    btVector3 sidePositionYZ2 = {mSizeGround,mSizeGround*.5f,mSizeGround*.5f};
    mGround->create_sides_yz(sidePositionYZ2);
    mRoot->addChild(mGround->getNode());
    mDynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());
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

