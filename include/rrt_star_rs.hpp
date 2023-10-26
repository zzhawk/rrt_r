// Copyright 2023 watson.wang
// https://github.com/zzhawk

#ifndef RRT_STAR_RS_HPP_
#define RRT_STAR_RS_HPP_

#include "rrt_star.hpp"

class rrtStarRs : public rrtStar
{
public:
	explicit rrtStarRs(pos start, pos goal, area randArea, area playArea, std::vector<obstacle> obstacles, cfg cfg)
		:rrtStar(start, goal, randArea, playArea, obstacles, cfg) {}

	virtual ~rrtStarRs() = default;

	rrtStarRs(const rrtStarRs&) = default;
	rrtStarRs(rrtStarRs&&) noexcept = default;

	rrtStarRs& operator = (const rrtStarRs&) = default;
	rrtStarRs& operator = (rrtStarRs&&) noexcept = default;

private:
	double _radius = 1.0;
	nodeSharedPtr steer(nodeSharedPtr& fnd, nodeSharedPtr& tnd, double lenth) override;
	double calcNewCost(nodeSharedPtr& fnd, nodeSharedPtr& tnd) override;

	double calcReedsSheppDistance(nodeSharedPtr& p1, nodeSharedPtr& p2, double radius);
	std::vector<pos> calcReedsSheppPath(nodeSharedPtr& p1, nodeSharedPtr& p2, double radius);
};

#endif