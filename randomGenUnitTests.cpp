#include "gtest/gtest.h"
#include "randomObstacles.h"
#include "arenanodemap.h"
#include <vector>

void expect_equal_vectors(std::vector<float> obstacleFootprint, std::vector<float> checkFootprint)
{
    size_t j{0};
    for (float i:obstacleFootprint)
    {
        EXPECT_EQ(i,checkFootprint[j]);
        j++;
    }
}
void expect_equal_vectors(std::vector<size_t> obstacleFootprint, std::vector<size_t> checkFootprint)
{
    size_t j{0};
    for (size_t i:obstacleFootprint)
    {
        EXPECT_EQ(i,checkFootprint[j]);
        j++;
    }
}
void test_the_arena(std::vector<std::vector<bool>> &arenaVector,size_t arenaSize, float xLo, float xHi, float yLo, float yHi)
{

    for(size_t i{0};i<arenaSize;i++)
    {
        for(size_t j{0};j<arenaSize;j++)
        {
            if (i<=static_cast<size_t>(xHi) && i>= static_cast<size_t>(xLo))
            {
                if (j<=static_cast<size_t>(yHi) && j>= static_cast<size_t>(yLo))
                {
                    EXPECT_EQ(arenaVector[i][j],true);
                }
            }
            else if (i == 0 || i == arenaSize-1 || j == 0 || j == arenaSize-1)
            {
                EXPECT_EQ(arenaVector[i][j],true);
            }
            else
            {
                EXPECT_EQ(arenaVector[i][j],false);
            }
        }
    }
}


TEST(makeSureObjectInBoundary, whenRandomObjectsGeneratedWithinBoundary_expectNoChange)
{
    float positionX{50};
    float sizeX{10};
    float environmentDimension{1000};
    float originalPositionX = positionX;
    float originalSizeX = sizeX;

    check_if_in_boundary(positionX,sizeX,environmentDimension);

    EXPECT_EQ(positionX,originalPositionX);
    EXPECT_EQ(sizeX,originalSizeX);
}

TEST(makeSureObjectInBoundary, whenRandomObjectsGeneratedAboveBoundary_expectPositionAndSizeChange)
{
    float positionX{1000};
    float sizeX{10};
    float environmentDimension{1000};
    float checkPositionX = environmentDimension - sizeX*.5f;
    float checkSizeX = sizeX *.75f;

    check_if_in_boundary(positionX,sizeX,environmentDimension);

    EXPECT_EQ(positionX,checkPositionX);
    EXPECT_EQ(sizeX,checkSizeX);
}

TEST(makeSureObjectInBoundary, whenRandomObjectsGeneratedBelowBoundary_expectPositionAndSizeChange)
{
    float positionX{0};
    float sizeX{10};
    float environmentDimension{1000};
    float checkPositionX = positionX + sizeX*.5f;
    float checkSizeX = sizeX *.75f;

    check_if_in_boundary(positionX,sizeX,environmentDimension);

    EXPECT_EQ(positionX,checkPositionX);
    EXPECT_EQ(sizeX,checkSizeX);
}

TEST(makeSureObjectInBoundary, whenRandomObjectsGeneratedBelowBoundaryThenShiftedAbove_expectTwoPositionAndSizeChanges)
{
    float positionX{0};
    float sizeX{1000};
    float environmentDimension{1000};
    float checkPositionX = positionX + sizeX*.5f;
    float checkSizeX = sizeX *.75f;
    checkPositionX = environmentDimension - sizeX*.5f;
    checkSizeX = sizeX *.75f;

    check_if_in_boundary(positionX,sizeX,environmentDimension);

    EXPECT_EQ(positionX,checkPositionX);
    EXPECT_EQ(sizeX,checkSizeX);
}

TEST(createMap, whenWorldGenerated_expectMapOfZero)
{
    size_t arenaSize{1000};
    arenaNodeMap newMap(arenaSize);
    std::vector<std::vector<bool>> arenaVector = newMap.return_the_map();
    for(size_t i{0};i<arenaSize;i++)
    {
        for(size_t j{0};j<arenaSize;j++)
        {
            if (i == 0 || i == arenaSize-1 || j == 0 || j == arenaSize-1)
            {
                EXPECT_EQ(arenaVector[i][j],true);
            }
            else
            {
                EXPECT_EQ(arenaVector[i][j],false);
            }
        }
    }
}

TEST(addObstacleToMap, whenObjectGenerated_checkIfFootprintOutsideMap)
{
    size_t arenaSize{1000};
    arenaNodeMap newMap(arenaSize);
    std::vector<std::vector<bool>> arenaVector = newMap.return_the_map();
    float xLo{-1};
    float yLo{-1};
    float xHi{1000};
    float yHi{1000};
    std::vector<float> obstacleFootprint{xLo,yLo,xHi,yHi};
    obstacleFootprint = check_footprint_bounds(obstacleFootprint);
    std::vector<float> checkFootprint{0,0,999,999};
    expect_equal_vectors(obstacleFootprint,checkFootprint);
}

TEST(addObstacleToMap, whenObjectGenerated_checkIfFootprintBoundsAreFlipped)
{
    size_t arenaSize{1000};
    arenaNodeMap newMap(arenaSize);
    std::vector<std::vector<bool>> arenaVector = newMap.return_the_map();
    size_t xLo{8};
    size_t yLo{7};
    size_t xHi{6};
    size_t yHi{5};
    std::vector<size_t> checkFootprint{xHi,yHi,xLo,yLo};
    check_if_bounds_flipped(xLo,xHi,yLo,yHi);
    std::vector<size_t> obstacleFootprint{xLo,yLo,xHi,yHi};
    expect_equal_vectors(obstacleFootprint,checkFootprint);
}

TEST(addObstacleToMap, whenObjectGenerated_expectTrueForPositionOnMap)
{
    size_t arenaSize{1000};
    arenaNodeMap newMap(arenaSize);
    float xLo{5};
    float yLo{6};
    float xHi{7};
    float yHi{8};
    std::vector<float> obstacleFootprint{xLo,yLo,xHi,yHi};
    newMap.add_to_obstacle_matrix(obstacleFootprint);
    std::vector<std::vector<bool>> arenaVector = newMap.return_the_map();
    test_the_arena(arenaVector,arenaSize, xLo, xHi, yLo, yHi);
}

