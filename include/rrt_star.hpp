// Copyright 2023 watson.wang

#ifndef RRT_STAR_HPP_
#define RRT_STAR_HPP_

#include "rrt_base.hpp"

class rrtStar : public rrtBase
{
public:
	explicit rrtStar(pos start, pos goal, area randArea, area playArea, std::vector<obstacle> obstacles, cfg cfg)
		:rrtBase(start, goal, randArea, playArea, obstacles, cfg) {}

	std::vector<pos> planning();

private:
	nodeSharedPtr chooseParent(nodeSharedPtr& nd, std::vector<int>& nearIdx);

	nodeSharedPtr searchBestGoalNode();
	std::vector<int> findNearNodesIdx(nodeSharedPtr& nd);
	void rewire(nodeSharedPtr& nd, std::vector<int>& nearIdx);
	double calcNewCost(nodeSharedPtr& fnd, nodeSharedPtr& tnd);
	void propagateCostToLeaves(nodeSharedPtr& pd);

	bool search_until_max_iter = true;
	double connect_circle_dist = 5.0;
};

#endif