#ifndef MAZE_H
#define MAZE_H

#include <vector>
#include <memory>
#include <ioutil>
#include <unordered_map>
#include <MGDStarNode.h>

class Maze {
 public:
  Maze(float height, float width, float length, float resolution, float occupancy);
  ~Maze();
  changeObstacles(float occupancy = occupancy_);
  
 private:
  std::unordered_map<MGDStarNode, MGDStarNode, MGDStarNodeHash, MGDStarNodeEquals> cells_;
  float height_, width_, length_, resolution_, occupancy_;
  std::unique_ptr<MGDStarNode> start_;
  std::vector<std::shared_ptr<MGDStarNode>> goals_;
};


#endif
