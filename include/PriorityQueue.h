#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

template<
  class T,
  class Container = std::vector<T>,
  class Compare,
  class Key>
>

class PriorityQueue {
public:
  PriorityQueue() {}
  ~PriorityQueue() {}

  void push(T t) {
  	container.push_back(t);
  	std::make_heap(container.begin(), container.end(), cmp);
  }

  void updateKey(T t, Key k) {
  	auto it = std::find(container.begin(), container.end(), t);
  	if (it != container.end()) {
  	  it->setKey(k);
  	  std::make_heap(container.begin(), container.end(), cmp);
  	}
  }

  void remove(T t) {
    auto it = std::find(container.begin(), container.end(), t);
  	if (it != container.end()) {
  	  container.erase(it);
  	  std::make_heap(container.begin(), container.end(), cmp);
  	}
  }

  bool exists(T t) {
  	auto it = std::find(container.begin(), container.end(), t);
  	return it != container.end()
  }

  T top() {
  	return container.front();
  }

private:
  Container container;
  Compare cmp;
};

#endif