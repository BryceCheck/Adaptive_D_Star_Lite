#ifndef MGD_STAR_NODE_H
#define MGD_STAR_NODE_H

#include <stdlib.h>
#include <utility>
#include <cstdio>
#include <vector>
#include <limits>
#include <math.h>
#include <ctime>
#include <ros.h>
#include <geometry_msgs/Pose.h>

struct MGDStarNodeHash {
  std::size_t operator()(const MGDStarNode& key) const;
}

struct MGDStarNodeEquals {
  bool operator()(const MGDStarNode& rhs, const MGDStarNode& lhs) const;
}

class MGDStarNode {
public:
  MGDStarNode(geometry_msgs::Pose pose);
  ~MGDStarNode();
  std::vector<std::shared_ptr<MGDStarNode>> getPredecessors();
  std::vector<std::shared_ptr<MGDStarNode>> getSuccessors();
  void removePredecessor(std::shared_ptr<MGDStarNode>& pred);
  void removeSuccessor(std::shared_ptr<MGDStarNode>& succ);
  void addPredecessor(std::shared_ptr<MGDStarNode>& );
  float getG();
  float getRHS();
  float getX();
  float getY();
  float getZ();
  std::pair<float, float> getKey();
  void setKey(std::pair<float, float> newKey);
  void setG(float g);
  void setRHS(float rhs);
  bool operator==(MGDStarNode& other);
  float operator-(MGDStarNode& other);


protected:
  float g_, rhs_;
  geometry_msgs::Pose pose_;
  std::pair <float, float> key_;
  std::vector<std::shared_ptr<MGDStarNode>> predecessors_, successors_;
};

#endif
