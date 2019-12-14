#include "gtest/gtest.h"
#include "pathfinder.h"
#include <vector>

void check_node_map_vector(std::vector<std::vector<Node>> nodeMap, size_t startX, size_t startY)
{
    for (size_t i{1};i<nodeMap.size();i++)
    {
        for (size_t j{1};j<nodeMap.size();j++)
        {
            if (i==startX&&j==startY)
            {
                EXPECT_EQ(nodeMap[i][j].x,startX);
                EXPECT_EQ(nodeMap[i][j].y,startY);
                EXPECT_EQ(nodeMap[i][j].distanceFromStart, 0);
                EXPECT_EQ(nodeMap[i][j].parentX, 1000000);
                EXPECT_EQ(nodeMap[i][j].parentY, 1000000);
                EXPECT_EQ(nodeMap[i][j].needToCheck, false);
            }
            else
            {
                EXPECT_EQ(nodeMap[i][j].x,i);
                EXPECT_EQ(nodeMap[i][j].y,j);
                EXPECT_EQ(nodeMap[i][j].distanceFromStart, nodeMap.size()*nodeMap.size());
                EXPECT_EQ(nodeMap[i][j].parentX, 0);
                EXPECT_EQ(nodeMap[i][j].parentY, 0);
                EXPECT_EQ(nodeMap[i][j].needToCheck, true);
            }
        }
    }
}



TEST(checkPathClassCreation, whenMapInitializedWithOriginStart_expectCorrectValues)
{
    size_t mapSize{10};
    size_t xStart{0};
    size_t yStart{0};
    size_t xGoal{5};
    size_t yGoal{9};
    std::vector<std::vector<bool>> obstMap{mapSize,std::vector<bool>(mapSize,false)};
    pathFinder newPath(obstMap,mapSize,xStart,yStart,xGoal,yGoal);
    newPath.initialize_map();
    std::vector<std::vector<Node>> nodeMap = newPath.return_node_map();
    check_node_map_vector(nodeMap, xGoal, yStart);

}

TEST(checkPathClassCreation, whenMapInitializedWithStartNotAtOrigin_expectCorrectY)
{
    size_t mapSize{10};
    size_t xStart{5};
    size_t yStart{1};
    size_t xGoal{5};
    size_t yGoal{9};
    std::vector<std::vector<bool>> obstMap{mapSize,std::vector<bool>(mapSize,false)};
    pathFinder newPath(obstMap,mapSize,xStart,yStart,xGoal,yGoal);
    newPath.initialize_map();
    std::vector<std::vector<Node>> nodeMap = newPath.return_node_map();
    check_node_map_vector(nodeMap, xGoal, yStart);
}


