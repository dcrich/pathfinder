#ifndef RANDOMOBSTACLES_H
#define RANDOMOBSTACLES_H

#include <vector>
#include "obstacleboxes.h"

class randomObstacles
{
public:
    obstacleBoxes* generate_random_obstacle(float mSizeGround, int numberofObstacles);
    std::vector<float> create_obstacle_area_matrix();
    void generate_random_coordinate(float &coordinateOfInterest);
    void generate_random_size(float &dimensionOfInterest, int numberOfObstacles);


private:
    float maxSize;
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
