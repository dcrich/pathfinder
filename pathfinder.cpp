#include "pathfinder.h"
#include <vector>


pathFinder::pathFinder(std::vector<std::vector<bool>> &obstacleMap, size_t mapSize, size_t xStart,size_t yStart, size_t xGoal,size_t yGoal)
{
    sizeMap = mapSize;
    mapNodes.resize(mapSize,std::vector<Node>(mapSize));
    startX = xStart;
    startY = yStart;
    goalX = xGoal;
    goalY = yGoal;
    find_the_shortest_path(obstacleMap);
}


void pathFinder::find_the_shortest_path(std::vector<std::vector<bool>> &obstacleMap)
{
    initialize_map(obstacleMap);
    iterate_through_map();

}

void pathFinder::initialize_map(std::vector<std::vector<bool>> &obstacleMap)
{
    theObstacles = obstacleMap;

    for (size_t i{0};i<sizeMap;i++)
    {
        for (size_t j{0};j<sizeMap;j++)
        {
            mapNodes[i][j].x = i;
            mapNodes[i][j].y = j;
            mapNodes[i][j].distanceFromStart= sizeMap*sizeMap;
            mapNodes[i][j].parentX = 0;
            mapNodes[i][j].parentY = 0;
            mapNodes[i][j].needToCheck = true;
        }
    }
    mapNodes[startX][startY].x = startX;
    mapNodes[startX][startY].y = startY;
    mapNodes[startX][startY].parentX = 1000000;
    mapNodes[startX][startY].parentY = 1000000;
    mapNodes[startX][startY].distanceFromStart = 0;
    mapNodes[startX][startY].needToCheck = false;
}


void pathFinder::iterate_through_map()
{
    openList.push(mapNodes[startX][startY]);
    counter = 0;
//    while (!openList.empty() && lookingForGoal && counter < 10000000)
    while (lookingForGoal)
    {
//        check_node_left();
//        check_node_right();
//        check_node_forward();
        checkNodeDirection = 1;
        node_check(openList.front().x-1,openList.front().y);
        checkNodeDirection = 2;
        node_check(openList.front().x+1,openList.front().y);
        checkNodeDirection = 3;
        node_check(openList.front().x,openList.front().y+1);
        check_if_goal();

        if (lookingForGoal == false)
        {
            size_t currentX = openList.front().x;
            size_t currentY = openList.front().y;
            std::vector<size_t> xyPath {currentX,currentY};
            thePath.push(xyPath);
            size_t i{0};
            //            for (size_t i{0}; i<openList.front().distanceFromStart; i++)
            while (mapNodes[currentX][currentY].parentX != startX && mapNodes[currentX][currentY].parentY != startY)
            {
                currentX = mapNodes[currentX][currentY].parentX;
                currentY = mapNodes[currentX][currentY].parentY;
                xyPath = {currentX,currentY};
                thePath.push(xyPath);
                i++;
            }
            xyPath = {startX,startY};
            thePath.push(xyPath);
        }
        else
        {
            closedList.push(openList.front());
            openList.pop();
        }
        counter++;
    }

}


//void pathFinder::check_node_left()
//{
//    checkNodeDirection = 1;
//    if (theObstacles[openList.front().x][openList.front().y] == false)
//    {
//        size_t i = openList.front().x-1;
//        size_t j = openList.front().y;
//        node_check(i,j);
//    }
//    else
//    {

//    }
//}


//void pathFinder::check_node_right()
//{
//    checkNodeDirection = 2;
//    if (theObstacles[openList.front().x][openList.front().y] == false)
//    {
//        size_t i = openList.front().x+1;
//        size_t j = openList.front().y;
//        node_check(i,j);
//    }
//}


//void pathFinder::check_node_forward()
//{
//    checkNodeDirection = 3;
//    if (theObstacles[openList.front().x][openList.front().y] == false)
//    {
//        size_t i = openList.front().x;
//        size_t j = openList.front().y+1;
//        node_check(i,j);
//    }
//}


void pathFinder::node_check(size_t i, size_t j)
{
    if (mapNodes[i][j].needToCheck)
    {
        if(theObstacles[i][j] == false)
        {
            theParentX = openList.front().x;
            theParentY = openList.front().y;
            mapNodes[i][j].distanceFromStart = openList.front().distanceFromStart+1;
            mapNodes[i][j].parentX = theParentX;
            mapNodes[i][j].parentY = theParentY;
            mapNodes[i][j].needToCheck = false;
            if (checkNodeDirection == 1)
            {
                openList.push(mapNodes[openList.front().x-1][openList.front().y]);
            }
            else if (checkNodeDirection == 2)
            {
                openList.push(mapNodes[openList.front().x+1][openList.front().y]);
            }
            else if (checkNodeDirection == 3)
            {
                openList.push(mapNodes[openList.front().x][openList.front().y+1]);
            }
        }
        else
        {
            mapNodes[i][j].needToCheck = false;
            closedList.push(mapNodes[i][j]);
        }
    }
    else if (mapNodes[i][j].distanceFromStart > openList.front().distanceFromStart+1)
    {
        mapNodes[i][j].distanceFromStart = openList.front().distanceFromStart+1;
        mapNodes[i][j].parentX = openList.front().x;
        mapNodes[i][j].parentY = openList.front().y;
    }
}

void pathFinder::check_if_goal()
{
//    if(openList.front().x == goalX && openList.front().y == goalY)
    if(openList.front().y > goalY-20)
    {
        lookingForGoal = false;
    }
}

std::queue<std::vector<size_t>> pathFinder::return_path()
{
    return thePath;
}



pathFinder::~pathFinder()
{

}
