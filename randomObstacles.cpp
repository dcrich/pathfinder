#include <random>
#include "randomObstacles.h"


obstacleBoxes* randomObstacles::generate_random_obstacle(float mSizeGround, int sizeOfObstacles, float xGoal, float yGoal, float sizeGoal)
{
    goalX = xGoal;
    goalY = yGoal;
    goalSize = sizeGoal;

    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<float> myDistribution(0,mSizeGround);

    sizeX = myDistribution(generator)*0.05f*sizeOfObstacles;

    sizeY = myDistribution(generator)*0.05f*sizeOfObstacles;
    sizeZ = 20.f;

    x = myDistribution(generator);
    y = myDistribution(generator);
    z = sizeZ*.5f;
    check_if_in_boundary(x,sizeX,mSizeGround);
    check_if_in_boundary(y,sizeY,mSizeGround);

    obstacleSize = {sizeX,sizeY,sizeZ};
    obstaclePosition = {x,y,z};
    mObstacleBox = new obstacleBoxes(obstacleSize, obstaclePosition);
    return mObstacleBox;
}

std::vector<float> randomObstacles::create_obstacle_area_matrix()
{
    float bufferValue{5};
    float xRangeLower = obstaclePosition[0] - obstacleSize[0]*0.5f - bufferValue;
    float yRangeLower = obstaclePosition[1] - obstacleSize[1]*0.5f - bufferValue;
    float xRangeUpper = obstaclePosition[0] + obstacleSize[0]*0.5f + bufferValue;
    float yRangeUpper = obstaclePosition[1] + obstacleSize[1]*0.5f + bufferValue;

    std::vector<float> obstacleFootprint{xRangeLower, yRangeLower, xRangeUpper, yRangeUpper};
    return obstacleFootprint;
}

void check_if_in_boundary(float &position, float &size, float mSizeGround)
{
    if (size*.5f + position >=mSizeGround)
    {
        position = mSizeGround - size*.5f;
        size = size * .75f;
    }
    if ( position - (.5f * size) <= 0)
    {
        position = position + size * .5f;
        size = size * .75f;
    }
    if (size*.5f + position >=mSizeGround)
    {
        position = mSizeGround - size*.5f;
        size = size * .75f;
    }
}

void check_if_blocking_goal(float &position, float &size, float xPos, float yPos)
{

}
