#ifndef MGD_STAR_SEARCH_H
#define MGD_STAR_SEARCH_H

#include <queue>
#include <vector>
#include <utility>
#include <algorithm>
#include <MGDStarNode.h>
#include <PriorityQueue.h>
#include <limits>

class KeyComparator {
  bool operator()(std::pair<float, float>& left, std::pair<float, float>& right);
};

class MGDSearch{
public:
  MGDSearch(std::vector<std::shared_ptr<MGDStarNode>> goals, geometry_msgs::Pose pose, float radius);
  ~MGDSearch();
  void init();
  void calculateKey(std::shared_ptr<MGDStarNode>& node);
  void updateVertex(std::shared_ptr<MGDStarNode>& node);
  void computeShortestPaths();
  void run();
private:
  // member functions
  float heuristic(std::shared_ptr<MGDStarNode>& node);
  void updateRHSValues(std::shared_ptr<MGDStarNode>& node);
  bool withinGoal(std::shared_ptr<MGDStarNode>& node);
  // member variables
  PriorityQueue<std::shared_ptr<MGDStarNode>, std::vector<std::shared_ptr<MGDStarNode>>, KeyComparator, std::pair<float, float>> searchQueue_;
  std::vector<std::shared_ptr<MGDStarNode>> goals_;
  std::shared_ptr<MGDStarNode> start_;
  float km_, successRadius_;
};

#endif
