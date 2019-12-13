#include <random>
#include "randomObstacles.h"


obstacleBoxes* randomObstacles::generate_random_obstacle(float mSizeGround, int numberOfObstacles)
{
    maxSize = mSizeGround;

    generate_random_size(sizeX, numberOfObstacles);
    generate_random_size(sizeY, numberOfObstacles);
    sizeZ = 20.f;

    generate_random_coordinate(x);
    generate_random_coordinate(y);
    z = sizeZ*.5f;
    check_if_in_boundary(x,sizeX,maxSize);
    check_if_in_boundary(y,sizeY,maxSize);

    obstacleSize = {sizeX,sizeY,sizeZ};
    obstaclePosition = {x,y,z};
    mObstacleBox = new obstacleBoxes(obstacleSize, obstaclePosition);
    return mObstacleBox;
}


void randomObstacles::generate_random_size(float &dimensionOfInterest, int numberOfObstacles)
{
    float minSizeObstacle = .05f*maxSize;
    float maxSizeOfObstacle = .2f*maxSize;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<float> myDistribution(minSizeObstacle,maxSizeOfObstacle);

    dimensionOfInterest = myDistribution(generator);
}


void randomObstacles::generate_random_coordinate(float &coordinateOfInterest)
{
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<float> myDistribution(0,maxSize);

    coordinateOfInterest = myDistribution(generator);
}


std::vector<float> randomObstacles::create_obstacle_area_matrix()
{
    float bufferValue{10};
    float xRangeLower = obstaclePosition[0] - obstacleSize[0]*0.5f - bufferValue;
    float yRangeLower = obstaclePosition[1] - obstacleSize[1]*0.5f - bufferValue;
    float xRangeUpper = obstaclePosition[0] + obstacleSize[0]*0.5f + bufferValue;
    float yRangeUpper = obstaclePosition[1] + obstacleSize[1]*0.5f + bufferValue;

    std::vector<float> obstacleFootprint{xRangeLower, yRangeLower, xRangeUpper, yRangeUpper};
    return obstacleFootprint;
}

void check_if_in_boundary(float &position, float &size, float maxSize)
{
    if (size*.5f + position >=maxSize)
    {
        position = maxSize - size*.5f;
        size = size * .75f;
    }
    if ( position - (.5f * size) <= 0)
    {
        position = position + size * .5f;
        size = size * .75f;
    }
    if (size*.5f + position >=maxSize)
    {
        position = maxSize - size*.5f;
        size = size * .75f;
    }
}
