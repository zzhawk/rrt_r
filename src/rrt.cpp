// Copyright 2023 watson.wang

#include "rrt.hpp"

nodeSharedPtr rrt::steer(nodeSharedPtr &fnd, nodeSharedPtr &tnd, double lenth)
{

	auto d = calDist(fnd->_p, tnd->_p);

	if (d <= _cfg.pathResolution) {
		tnd->_path.push_back(fnd->_p);
		tnd->_path.push_back(tnd->_p);
	}
	else {

		if (lenth > d) lenth = d;

		auto theta = calAngle(fnd->_p, tnd->_p);
		tnd->_path.push_back(fnd->_p);
		auto n_expand = std::floor(lenth / _cfg.pathResolution);

		pos t = fnd->_p;
		for (int i = 1; i < n_expand; ++i) {
			t.x += _cfg.pathResolution * std::sin(theta);
			t.y += _cfg.pathResolution * std::cos(theta);
			tnd->_path.push_back(t);
		}
		tnd->_p = tnd->_path.back();

	}

	tnd->_parent = fnd;
	return tnd;
}

std::vector<pos> rrt::planning()
{

	for (int i = 0; i < _cfg.max_iter; ++i) {
		auto randNode = getRandNode();
		pos p = randNode->_p;
		auto nearestNode = getNearestNode(randNode);
		auto newNode = steer(nearestNode, randNode, _cfg.expand_dis);

		if (!isOutsideArea(newNode)) {
			if (!isCollision(newNode)) {
				_nodes.push_back(newNode);

				if (calDist(newNode->_p, _goal) < _cfg.expand_dis) {
					auto finalNode = steer(newNode, _goalNode, _cfg.expand_dis);
					if (!isCollision(finalNode)) {
						_nodes.push_back(_goalNode);
						return getResult();
					}
				}
			}
		}


	}

	return {};
}


