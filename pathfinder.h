#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <iostream>
#include <vector>
#include <queue>

struct Node
{
    size_t x;
    size_t y;
    size_t parentX;
    size_t parentY;
    size_t distanceFromStart;
    bool needToCheck;
};

class pathFinder
{
public:
    pathFinder(std::vector<std::vector<bool>> &obstacleMap, size_t mapSize, size_t xStart, size_t yStart, size_t xGoal, size_t yGoal);
    ~pathFinder();
    void initialize_map();
    void iterate_through_map();
    void node_check(size_t x, size_t y);
    void check_if_goal();
    std::queue<std::vector<size_t>> return_path();
    std::vector<std::vector<Node>> return_node_map();

private:
    size_t sizeMap{1000};
    size_t startX{500};
    size_t startY{1};
    size_t goalX{500};
    size_t goalY{1};
    std::queue <Node> openList;
    std::queue <Node> closedList;
    bool lookingForGoal{true};
    std::queue<std::vector<size_t>> thePath;
    std::vector<std::vector<Node>> mapNodes;
    std::vector<std::vector<bool>> theObstacles;
    int counter{0};
    int checkNodeDirection{1};
    size_t theParentX;
    size_t theParentY;
    bool listCheck{true};
    std::vector<size_t> xyPath;
};

#endif // PATHFINDER_H
