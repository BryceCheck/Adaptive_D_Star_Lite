#include <MGDStarNode.h>

std::size_t MGDStarNodeHash::operator()(const MGDStarNode& key) const {
  std::size_t rhs = std::hash(key->getX() * 6.66 + key->getY() * 11.34);
  std::size_t lhs = std::hash(key->getZ() * 80.085);
  return rhs ^ lhs;
}

bool MGDStarNodeEquals::operator()(const MGDStarNode& rhs, const MGDStarNode& lhs) const { return rhs == lhs; }

MGDStarNode::MGDStarNode(geometry_msgs::Pose pose):pose_(pose) {
  float inf = std::numeric_limits<float>::infinity();
  key_ = std::make_pair(inf, inf);
  g_ = inf;
  rhs_ = inf;
}

MGDStarNode::~MGDStarNode(){}

std::vector<std::shared_ptr<MGDStarNode>> MGDStarNode::getPredecessors() {return predecessors_;}
std::vector<std::shared_ptr<MGDStarNode>> MGDStarNode::getSuccessors() {return successors_;}
void MGDStarNode::addPredecessor(std::shared_ptr<MGDStarNode>& node) { predecessors_.push_back(node); }
void MGDStarNode::addSuccessor(std::shared_ptr<MGDStarNode>& node) { successors_.push_back(node); }
void MGDStarNode::setKey(std::pair<float, float> newKey) { key_ = newKey; }
void MGDStarNode::setG(float g) { g_ = g; }
void MGDStarNode::setRHS(float rhs) { rhs_ = rhs; }
std::pair<float, float> getKey() { return key_; }
float getG() { return g_; }
float getRHS() { return rhs_; }
float getX() { return pose_.position.x; }
float getY() { return pose_.position.y; }
float getZ() { return pose_.position.z; }

void MGDStarNode::removePredecessor(std::shared_ptr<MGDStarNode> node) {
  for (auto it = predecessors_.begin(); it != predecessors_.end(); it++) {
  	if (node == **it) {
  	  predecessors_.remove(it);
  	  break;
  	}
  }
}

void MGDStarNode::removeSuccessor(std::shared_ptr<MGDStarNode> node) {
  for (auto it = successors_.begin(); it != successors_.end(); it++) {
  	if (node == **it) {
  	  successors_.remove(it);
  	  break;
  	}
  }
}

bool MGDStarNode::operator==(MGDStarNode& other) {
  return (pose_.position.x == other.pose_.position.x && pose_.position.y == other.pose_.position.y && pose_.position.z = other.pose_.position.z);
}

float MGDStarNode::operator-(MGDStarNode& other) {
  // find the component differences
  float deltaX = pose_.position.x - other.pose_.position.x;
  float deltaY = pose_.position.y - other.pose_.position.y;
  float deltaZ = pose_.position.z - other.pose_.position.z;
  // perform 3D-Pythagorean theorem
  return pow(pow(deltaX, 2.0) + pow(deltaY, 2,0) + pow(deltaZ, 2,0), 0.5);
}
