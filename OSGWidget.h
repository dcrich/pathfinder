#ifndef OSGWidget_h__
#define OSGWidget_h__

#include <QPoint>
#include <QOpenGLWidget>
#include "btBulletDynamicsCommon.h"
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>

#include <osg/ref_ptr>
#include <osg/Vec4>

#include <osgViewer/GraphicsWindow>
#include <osgViewer/CompositeViewer>
#include "obstacleboxes.h"
#include "boundingbox.h"
#include "arenanodemap.h"
#include "thevehicle.h"
#include <queue>


class OSGWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    OSGWidget( QWidget* parent = 0,
               Qt::WindowFlags f = 0 );
    virtual ~OSGWidget();
    void start_timer();
    void stop_timer();
    void setup_environment();
    void reset_world();
    void run_manual();
    void create_obstacles(int numberOfObstacles);
    void make_ground();
    void arrow_key_velocity_update(int arrowDirection);
    void run_auto_path();
    void set_goal();
    void set_camera_view();
    void reveal_path();

protected:
    virtual void paintEvent( QPaintEvent* paintEvent );
    virtual void paintGL();
    virtual void resizeGL( int width, int height );
    virtual void keyPressEvent( QKeyEvent* event );
    virtual void keyReleaseEvent( QKeyEvent* event );
    virtual void mouseMoveEvent( QMouseEvent* event );
    virtual void mousePressEvent( QMouseEvent* event );
    virtual void mouseReleaseEvent( QMouseEvent* event );
    virtual void wheelEvent( QWheelEvent* event );
    virtual bool event( QEvent* event );
    void timerEvent(QTimerEvent *);


private:
    void set_up_physics();
    void createWorld();
    virtual void onHome();
    virtual void on_resize( int width, int height );
    osgGA::EventQueue* getEventQueue() const;
    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> mGraphicsWindow;
    osg::ref_ptr<osgViewer::CompositeViewer> mViewer;
    osg::ref_ptr<osgGA::CameraManipulator> mManipulator;
    btBroadphaseInterface* mBroadphaseInterface;
    btDefaultCollisionConfiguration* mDefaultCollisionConfig;
    btCollisionDispatcher* mCollisionDispatcher;
    btSequentialImpulseConstraintSolver* mSeqImpConstraintSolver;
    btDiscreteDynamicsWorld* mDynamicsWorld;

    double mTimeStep;
    bool mStarted;
    int mTimerId{0};

    boundingBox* mGround;
    float mSizeGround{1000};
    obstacleBoxes * mObstacleBox;
    size_t xStart{500};
    size_t yStart{1};
    QVector3D vehicleStartPosition;
    QVector4D vehicleColor;
    theVehicle* mVehicle;
    osg::ref_ptr<osg::Group> mRoot;
    osg::Timer_t mStartTick;
    osg::Camera* camera;
    osgViewer::View* view;
    arenaNodeMap * newArenaMap;
    btVector3 currentPosition;
    osg::Geode* goalGeode;
    osg::Box* goalBox;
    float sizeGoal{20};
    float xGoalPosition{500};
    float yGoalPosition{mSizeGround-1};
    std::queue<std::vector<size_t>> autoPath;
    btVector3 velocityIncrease;
    std::vector<size_t> autoCoordinates;
    osg::Geode* pathGeode;
    osg::Box* pathBox;
    bool obstaclesCreated{false};
    bool winStatus{false};
    bool mBusy;
};

#endif
