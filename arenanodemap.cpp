#include "arenanodemap.h"

arenaNodeMap::arenaNodeMap(size_t mArenaSize)
{
    mapSize = mArenaSize;
    if (mArenaSize!=1000)
    {
        arenaStatusMap.resize(mArenaSize, std::vector<bool>(mArenaSize,false));
    }
    add_fence_to_map();
}

void arenaNodeMap::find_path(size_t startPoint, size_t endPoint)
{
    startPosition = startPoint;
    endPosition = endPoint;
}


void arenaNodeMap::add_to_obstacle_matrix(std::vector<float> obstacleFootprint)
{
    obstacleFootprint = check_footprint_bounds(obstacleFootprint);
    lowerBoundX = static_cast<size_t>(obstacleFootprint[0]);
    upperBoundX = static_cast<size_t>(obstacleFootprint[2]);
    lowerBoundY = static_cast<size_t>(obstacleFootprint[1]);
    upperBoundY = static_cast<size_t>(obstacleFootprint[3]);
    check_if_bounds_flipped(lowerBoundX,upperBoundX,lowerBoundY,upperBoundY);
    for(size_t i = lowerBoundX; i <= upperBoundX; i++)
    {
        for(size_t j = lowerBoundY; j <= upperBoundY; j++)
        {
            arenaStatusMap[i][j] = true;
        }
    }
}

void arenaNodeMap::add_fence_to_map()
{
    for (size_t i{0};i<mapSize;i++)
    {
        for (size_t j{0};j<mapSize;j++)
        {
            if (i == 0 || i == mapSize-1 || j == 0 || j == mapSize-1)
            {
            arenaStatusMap[i][j] = true;
            }
        }
    }
}

std::vector<std::vector<bool>> arenaNodeMap::return_the_map()
{
    return arenaStatusMap;
}

std::vector<float> check_footprint_bounds(std::vector<float> obstacleFootprint)
{
    size_t j{0};
    for(float i:obstacleFootprint)
    {
        if (i <= 0)
        {
            obstacleFootprint[j] = 0;
        }
        else if (i >= 1000)
        {
            obstacleFootprint[j] = 999;
        }
        j++;
    }
    return obstacleFootprint;
}

void check_if_bounds_flipped(size_t &lowerBoundX,size_t &upperBoundX,size_t &lowerBoundY,size_t &upperBoundY)
{
    if(lowerBoundX>upperBoundX)
    {
        size_t tempVal = lowerBoundX;
        lowerBoundX = upperBoundX;
        upperBoundX = tempVal;
    }
    if(lowerBoundY>upperBoundY)
    {
        size_t tempVal = lowerBoundY;
        lowerBoundY = upperBoundY;
        upperBoundY = tempVal;
    }
}
arenaNodeMap::~arenaNodeMap()
{

}
