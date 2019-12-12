#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <iostream>
#include <vector>

struct Node
{
    size_t parentX;
    size_t parentY;
    size_t distanceFromStart;
    bool visited;
    bool isObstacle;
    bool needToCheck;
};

class pathFinder
{
public:
    pathFinder(std::vector<std::vector<bool>> &obstacleMap, size_t mapSize, size_t xStart, size_t yStart, size_t xGoal, size_t yGoal);
    ~pathFinder();
    void find_the_shortest_path(std::vector<std::vector<bool>> &obstacleMap);
    void initialize_map(std::vector<std::vector<bool>> &obstacleMap);
    void iterate_through_map();
    bool check_node(size_t &currentX,size_t &currentY);
    void check_node_children(size_t &currentX,size_t &currentY);

private:

    size_t sizeMap{1000};
    size_t startX{500};
    size_t startY{1};
    size_t goalX{500};
    size_t goalY{1};
    std::vector<std::vector<Node>> mapNodes;
};

#endif // PATHFINDER_H
