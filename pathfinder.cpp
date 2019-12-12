#include "pathfinder.h"
#include <vector>


pathFinder::pathFinder()
{

}


void pathFinder::find_the_shortest_path()
{
    initialize_map();

}

void pathFinder::initialize_map()
{
    for (size_t i{0};i<sizeMap;i++)
    {
        for (size_t j{0};j<sizeMap;j++)
        {
            mapNodes[i][j].x = i;
            mapNodes[i][j].y = j;
            mapNodes[i][j].visited = false;
            mapNodes[i][j].distanceFromStart= sizeMap*sizeMap;
            mapNodes[i][j].parentX = 0;
            mapNodes[i][j].parentY = 0;
        }
    }
}

pathFinder::~pathFinder()
{

}
