#include <MGDStarSearch.h>

bool KeyComparator::operator()(std::pair<float, float>& left, std::pair<float, float>& right) {
  if (lKey.first == rKey.first) {
    return lKey.second <= rKey.second; // check whether it should be LT/LTE/GT/GTE
  } else {
    return lKey.first <= rKey.second; // check whether it should be LT/LTE/GT/GTE
  }
}

MGDStarSearch::MGDStarSearch(std::vector<std::shared_ptr<MGDStarNode>> goals, geometry_msgs::Pose pose, float radius) :
 goals_(goals), start_(std::make_shared<MGDStarNode>(pose)), km_(0.0), successRadius_(radius) {}

MGDStarSearch::~MGDStarSearch(){}

void MGDStarSearch::init() {
  // Create a maze here
  // Set all the rhs/g values to infinity
  // set the rhs of all the goals to 0
  // Insert all the goals with (heuristic(start, goal), 0) as the keys
}

void MGDStarSearch::calculateKey(std::shared_ptr<MGDStarNode>& node) {
  k1 = min(node->getG(), node->getRHS() + heuristic(node) + km_);
  k2 = min(node->getG(), node->getRHS());
  return std::make_pair(k1, k2);
}

bool MGDStarNode::withinGoal(std::shared_ptr<MGDStarNode>& node) {
  for (auto& goal : goals_) {
  	if ((*goal - *node) < successRadius_) return true;
  }
  return false;
}

void MGDStarSearch::updateVertex(std::shared_ptr<MGDStarNode>& node) {
  if(node->getG() != node->getRHS() && searchQueue_.exists(node)) searchQueue_.updateKey(node, calculateKey(node));
  else if(node->getG() != node->getRHS() && searchQueue_.exists(node)) searchQueue_.push(node);
  else if(node->getG() == node->getRHS() && searchQueue_.exists(node)) searchQueue_.remove(node);
}

float MGDStarSearch::heuristic(std::shared_ptr<MGDStarNode>& node) {
  return *start_ - *node;
}

void MGDStarSearch::updateRHSValues(std::shared_ptr<MGDStarNode>& s, std::shared_ptr<MGDStarNode>& u, float gOld) {
  std::vector<std::shared_ptr<MGDStarNode>> successors = s->getSuccessors();
  if (s->getRHS() == (*s - *u) + gOld ) {
  	if (!withinGoal(s)) {
  	  float check;
  	  float lowest = (*s - *(successors.begin()) + successors.begin()->getG());
  	  for (auto& sPrime = successors.begin() + 1; sPrime != successors.end(); sPrime++) {
  	  	check = (*s - *sPrime) + sPrime->getG();
  	  	lowest = min(lowest, check);
  	  }
  	  s->setRHS(lowest);
  	}
  	updateVertex(s);
  }
}

bool MGDStarSearch::computeShortestPath() {
  // Set up the keys to compare for the loop invariant
  KeyComparator cmp;
  std::pair<float, float> topKey, startKey;
  topKey = searchQueue_.top()->getKey();
  startKey = calculateKey(start_);
  // Begin the loop
  while(cmp(topKey, startKey)) {
  	// set up the variables for the loop operations
  	std::shared_ptr<MGDStarNode> top;
  	std::pair<float, float> oldKey, newKey;
  	top = searchQueue_.top();
  	oldKey = top->getKey();
  	newKey = calculateKey(top);
  	std::vector<std::shared_ptr<MGDStarNode>> predecessors = top->getPredecessors();
  	if(cmp(oldKey, newKey)) {
  	  searchQueue_.updateKey(top, newKey);
  	} else if (top->getG() > top->getRHS()) { 
  	  top->setG(top->getRHS());
  	  searchQueue_.remove(top);
  	  for (auto& it : predecessors) {
  	  	if (!withinGoal(it)) it->setRHS(min(it->getRHS(), (*it - *top) + top->getG()));
  	  	updateVertex(*it);
  	  }
  	} else {
  	  float inf = std::numeric_limits<float>::infinity();
  	  float gOld = top->getG();
  	  top->setG(inf);
  	  // Check and set top's rhs value
  	  updateRHSValues(top, top, gOld);
  	  // Check and set all of top's predecssors' RHS values
  	  for (auto& s : predecssors) {
  	  	updateRHSValues(s, top, gOld);
  	  }
  	}
  }
}

std::shared_ptr<MGDStarNode> MGDStarSearch::getNextWaypoint(std::shared_ptr<MGDStarNode> curr) {
  std::shared_ptr<MGDStarNode> nextNode = NULL;
  std::pair<float, float> lowestKey;
  KeyComparator cmp;
  for(auto& successor : curr->getSuccessors()) {
    // Start the checking process
    if (nextNode == NULL || cmp(successor->getKey(), lowestKey)) {
      nextNode = successor;
      lowestKey = successor->getKey();
    }
  }
  return nextNode;
}

void MGDStarSearch::run() {
  KeyComparator cmp;
  std::shared_ptr<MGDStarNode> sLast;
  init();
  computeShortestPath();
  while( !goals_.empty() ) {
    start_ = getNextWaypoint(start_);
    // TODO
    // Send out command signal here to robot or simulation
    // Go through the rest of the graph and check for changed edge costs/impassable nodes
    // Update the edge costs according to the optimality in Fig 3 of the paper in the Main step
    // Update each changed, traversable vertex
    // Recompute the shortest path
  }
  
  
}
