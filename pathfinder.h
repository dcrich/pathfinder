#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <iostream>
#include <vector>



class pathFinder
{
public:
    pathFinder();
    ~pathFinder();
    void find_the_shortest_path();
    void initialize_map();

private:
    struct Node
    {
        size_t x;
        size_t y;
        bool visited;
        size_t distanceFromStart;
        size_t parentX;
        size_t parentY;
    };
    size_t sizeMap{1000};
    std::vector<std::vector<Node>> mapNodes;
};

#endif // PATHFINDER_H
