#include "arenanodemap.h"

arenaNodeMap::arenaNodeMap(size_t mArenaSize)
{
    mapSize = mArenaSize;
    if (mArenaSize!=1000)
    {
        arenaStatusMap.resize(mArenaSize, std::vector<int>(mArenaSize,0));
    }
}

void arenaNodeMap::add_to_obstacle_matrix(std::vector<float> obstacleFootprint)
{
    if (obstacleFootprint[0] <= 0)
    {
        obstacleFootprint[0] = 0;
    }
    if (obstacleFootprint[1] <= 0)
    {
        obstacleFootprint[1] = 0;
    }
    if (obstacleFootprint[2] > 1000)
    {
        obstacleFootprint[2] = 999;
    }
    if (obstacleFootprint[3] > 1000)
    {
        obstacleFootprint[3] = 999;
    }

    size_t lowerBoundX = static_cast<size_t>(obstacleFootprint[0]);
    size_t upperBoundX = static_cast<size_t>(obstacleFootprint[2]);
    size_t lowerBoundY = static_cast<size_t>(obstacleFootprint[1]);
    size_t upperBoundY = static_cast<size_t>(obstacleFootprint[3]);
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

    for(size_t i = lowerBoundX; i <= upperBoundX; i++)
    {
        for(size_t j = lowerBoundY; j <= upperBoundY; j++)
        {
            arenaStatusMap[i][j] = 1;
        }
    }
}

std::vector<std::vector<int>> arenaNodeMap::return_the_map()
{
    return arenaStatusMap;
}
