cmake_minimum_required(VERSION 3.4)
project(aFinalProject)
FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(QT5GUI)
FIND_PACKAGE( Qt5OpenGL )
FIND_PATH(BULLET_ROOT include/bullet/btBulletCollisionCommon.h HINTS ENV BULLET_ROOT)
FIND_PACKAGE(Bullet)
FIND_PACKAGE( OpenSceneGraph REQUIRED COMPONENTS osgDB osgGA osgUtil osgViewer )
INCLUDE_DIRECTORIES( ${OPENSCENEGRAPH_INCLUDE_DIRS} )

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED 1)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

IF (NOT TARGET gtest)
    # Prevent GoogleTest from overriding our compiler/linker options
    # when building with Visual Studio
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    add_subdirectory(googletest)
ENDIF()

IF(BULLET_INCLUDE_DIR)
INCLUDE_DIRECTORIES(${BULLET_INCLUDE_DIR})
ENDIF()

SET(MYSOURCE
    main.cpp
    MainWindow.cpp
    MainWindow.h
    MainWindow.ui
    OSGWidget.h
    OSGWidget.cpp
    )
add_library(physicsWorldStuff STATIC
    obstacleboxes.cpp
    obstacleboxes.h
    boundingbox.cpp
    boundingbox.h
    thevehicle.cpp
    thevehicle.h
    randomObstacles.cpp
    randomObstacles.h
    arenanodemap.cpp
    arenanodemap.h
    pathfinder.cpp
    pathfinder.h
    )
target_link_libraries(physicsWorldStuff
    Qt5::Widgets
    Qt5::Gui
    Qt5::OpenGL
    ${OPENSCENEGRAPH_LIBRARIES}
    ${BULLET_LIBRARIES}
    )

add_executable(theUnitTests
    randomGenUnitTests.cpp
    pathUnitTests.cpp
    )
target_link_libraries(theUnitTests
    Qt5::Widgets
    Qt5::Gui
    Qt5::OpenGL
    ${OPENSCENEGRAPH_LIBRARIES}
    ${BULLET_LIBRARIES}
    physicsWorldStuff
    gtest
    gtest_main
    )

add_executable(${PROJECT_NAME}
    ${MYSOURCE}
    )
target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::Gui
    Qt5::OpenGL
    ${OPENSCENEGRAPH_LIBRARIES}
    ${BULLET_LIBRARIES}
    physicsWorldStuff
    )

