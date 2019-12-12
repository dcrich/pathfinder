#ifndef ARENANODEMAP_H
#define ARENANODEMAP_H
#include <vector>

class arenaNodeMap
{
public:
    arenaNodeMap(size_t mArenaSize);
    ~arenaNodeMap();
    void add_to_obstacle_matrix(std::vector<float> obstacleFootprint);
    void add_fence_to_map();
    std::vector<std::vector<bool>> return_the_map();
    void find_path(size_t startPoint, size_t endPoint);

private:
    std::vector<std::vector<bool>> arenaStatusMap{1000,std::vector<bool>(1000,false)};
    size_t mapSize;
    size_t lowerBoundX;
    size_t upperBoundX;
    size_t lowerBoundY;
    size_t upperBoundY;
    size_t startPosition;
    size_t endPosition;
};
std::vector<float> check_footprint_bounds(std::vector<float> obstacleFootprint);
void check_if_bounds_flipped(size_t &lowerBoundX,size_t &upperBoundX,size_t &lowerBoundY,size_t &upperBoundY);

#endif // ARENANODEMAP_H
