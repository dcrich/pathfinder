#ifndef RANDOMOBSTACLES_H
#define RANDOMOBSTACLES_H

#include <vector>
#include "obstacleboxes.h"

class randomObstacles
{
public:
    obstacleBoxes* generate_random_obstacle(float mSizeGround, int sizeOfObstacles);


private:
    float x;
    float y;
    float z;
    float sizeX;
    float sizeY;
    float sizeZ;
    std::vector<float> obstacleSize;
    std::vector<float> obstaclePosition;
    obstacleBoxes * mObstacleBox;
};
void check_if_in_boundary(float &position, float &size, float mSizeGround);


#endif // RANDOMOBSTACLES_H
