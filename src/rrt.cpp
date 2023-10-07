// Copyright 2023 watson.wang

#include "rrt.hpp"

std::vector<pos> rrt::planning()
{

	for (int i = 0; i < _cfg.maxIter; ++i) {
		auto randNode = getRandNode();
		auto nearestNode = getNearestNode(randNode);
		auto newNode = steer(nearestNode, randNode, _cfg.expand_dis);
		
		if (newNode != nullptr) {
			if (!isOutsideArea(newNode)) {
				if (!isCollision(newNode)) {
					_nodes.push_back(newNode);

					if (calDist(newNode->_p, _goal) < _cfg.expand_dis) {
						auto finalNode = steer(newNode, _goalNode, _cfg.expand_dis);
						if (!isCollision(finalNode)) {
							_nodes.push_back(finalNode);
							return getResult();
						}
					}
				}
			}
		}
	}

	return {};
}


