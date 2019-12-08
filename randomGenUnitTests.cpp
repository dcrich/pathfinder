#include "gtest/gtest.h"
#include "randomObstacles.h"

TEST(makeSureObjectInBoundary, whenRandomObjectsGeneratedWithinBoundary_expectNoChange)
{
    float positionX{50};
    float sizeX{100};
    float environmentDimension{1000};
    float originalPositionX = positionX;
    float originalSizeX = sizeX;
    check_if_in_boundary(positionX,sizeX,environmentDimension);
    EXPECT_EQ(positionX,originalPositionX);
    EXPECT_EQ(sizeX,originalSizeX);
}
