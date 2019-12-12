#include "pathfinder.h"
#include <vector>


pathFinder::pathFinder(std::vector<std::vector<bool>> &obstacleMap, size_t mapSize, size_t xStart,size_t yStart, size_t xGoal,size_t yGoal)
{
    sizeMap = mapSize;
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
    for (size_t i{0};i<sizeMap;i++)
    {
        for (size_t j{0};j<sizeMap;j++)
        {
            mapNodes[i][j].visited = false;
            mapNodes[i][j].distanceFromStart= sizeMap*sizeMap;
            mapNodes[i][j].parentX = 0;
            mapNodes[i][j].parentY = 0;
            if (obstacleMap[i][j] == true)
            {
                mapNodes[i][j].isObstacle = true;
                mapNodes[i][j].needToCheck = false;
            }
            else
            {
                mapNodes[i][j].isObstacle = false;
                mapNodes[i][j].needToCheck = true;
            }
        }
    }
}


void pathFinder::iterate_through_map()
{
    bool doneChecking{false};
    int counterCheck{0};
    size_t currentX = startX;
    size_t  currentY = startY;
    while(doneChecking == false)
    {
        check_node(currentX,currentY);
        check_node_children(currentX,currentY);

        counterCheck++;
        if (counterCheck == 1000000)
        {
            doneChecking = true;
        }
    }
}

bool pathFinder::check_node(size_t &currentX,size_t &currentY)
{
    bool isNodeValid{true};

    if (currentX == startX && startY == goalY)
    {
        isNodeValid = false;
    }

    if (currentX == goalX && currentY == goalY)
    {
        isNodeValid = false;
    }
    return isNodeValid;
}


void pathFinder::check_node_children(size_t &currentX,size_t &currentY)
{
    size_t newX = currentX+1;
    if (mapNodes[newX][currentY].visited == false)
    {
        mapNodes[newX][currentY].parentX = currentX;
        mapNodes[newX][currentY].parentY = currentY;
        mapNodes[newX][currentY].distanceFromStart = mapNodes[currentX][currentY].distanceFromStart + 1;
        mapNodes[newX][currentY].visited = true;
        mapNodes[newX][currentY].needToCheck = false;
    }
    else if (mapNodes[newX][currentY].distanceFromStart > mapNodes[currentX][currentY].distanceFromStart + 1)
    {
        mapNodes[newX][currentY].parentX = currentX;
        mapNodes[newX][currentY].parentY = currentY;
        mapNodes[newX][currentY].distanceFromStart = mapNodes[currentX][currentY].distanceFromStart + 1;
        mapNodes[newX][currentY].visited = true;
        mapNodes[newX][currentY].needToCheck = false;
    }

    size_t newY = currentY+1;
    if (mapNodes[currentX][newY].visited == false)
    {
        mapNodes[currentX][newY].parentX = currentX;
        mapNodes[currentX][newY].parentY = currentY;
        mapNodes[currentX][newY].distanceFromStart = mapNodes[currentX][currentY].distanceFromStart + 1;
        mapNodes[currentX][newY].visited = true;
        mapNodes[currentX][newY].needToCheck = false;
    }
    else if (mapNodes[currentX][newY].distanceFromStart > mapNodes[currentX][currentY].distanceFromStart + 1)
    {
        mapNodes[currentX][newY].parentX = currentX;
        mapNodes[currentX][newY].parentY = currentY;
        mapNodes[currentX][newY].distanceFromStart = mapNodes[currentX][currentY].distanceFromStart + 1;
        mapNodes[currentX][newY].visited = true;
        mapNodes[currentX][newY].needToCheck = false;
    }

    newX = currentX-1;
    if (mapNodes[newX][currentY].visited == false)
    {
        mapNodes[newX][currentY].parentX = currentX;
        mapNodes[newX][currentY].parentY = currentY;
        mapNodes[newX][currentY].distanceFromStart = mapNodes[currentX][currentY].distanceFromStart + 1;
        mapNodes[newX][currentY].visited = true;
        mapNodes[newX][currentY].needToCheck = false;
    }
    else if (mapNodes[newX][currentY].distanceFromStart > mapNodes[currentX][currentY].distanceFromStart + 1)
    {
        mapNodes[newX][currentY].parentX = currentX;
        mapNodes[newX][currentY].parentY = currentY;
        mapNodes[newX][currentY].distanceFromStart = mapNodes[currentX][currentY].distanceFromStart + 1;
        mapNodes[newX][currentY].visited = true;
        mapNodes[newX][currentY].needToCheck = false;
    }
}


pathFinder::~pathFinder()
{

}
