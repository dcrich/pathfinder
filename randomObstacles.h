#ifndef RANDOMOBSTACLES_H
#define RANDOMOBSTACLES_H

#include <vector>
#include "obstacleboxes.h"

class randomObstacles
{
public:
    obstacleBoxes* generate_random_obstacle(float mSizeGround, int sizeOfObstacles, float xGoal, float yGoal, float sizeGoal);
    std::vector<float> create_obstacle_area_matrix();



private:
    float x;
    float y;
    float z;
    float sizeX;
    float sizeY;
    float sizeZ;
    float goalX;
    float goalY;
    float goalSize;
    std::vector<float> obstacleSize;
    std::vector<float> obstaclePosition;
    obstacleBoxes * mObstacleBox;
};
void check_if_in_boundary(float &position, float &size, float mSizeGround);
void check_if_blocking_goal(float &position, float &size, float xPos, float yPos);


#endif // RANDOMOBSTACLES_H
