// Copyright 2023 watson.wang

#include "rrt_base.hpp"

class rrt : public rrtBase 
{
public:
	explicit rrt(pos start, pos goal, area randArea, area playArea, std::vector<obstacle> obstacles, cfg cfg)
		:rrtBase(start, goal, randArea, playArea, obstacles, cfg){}

	std::vector<pos> planning() override;
	nodeSharedPtr steer(nodeSharedPtr& fnd, nodeSharedPtr& tnd, double lenth) override;

};