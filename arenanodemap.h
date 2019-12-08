#ifndef ARENANODEMAP_H
#define ARENANODEMAP_H
#include <vector>

class arenaNodeMap
{
public:
    arenaNodeMap(size_t mArenaSize);
    void add_to_obstacle_matrix(std::vector<float> obstacleFootprint);
    std::vector<std::vector<int>> return_the_map();


private:
    std::vector<std::vector<int>> arenaStatusMap{1000,std::vector<int>(1000,0)};
    size_t mapSize;
};

#endif // ARENANODEMAP_H
