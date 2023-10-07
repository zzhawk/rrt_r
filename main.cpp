// Copyright 2023 watson.wang

#include <fstream>
#include "nlohmann/json.hpp"
#include "rrt.hpp"
#include "rrt_star.hpp"

using json = nlohmann::json;

int main() {
	json out;
	std::vector<obstacle> obs;
	obstacle ob;
	std::ofstream env("../env.json");

	ob.p.x = 5.0; ob.p.y = 5.0; ob.radius = 1.0; obs.push_back(ob);
	ob.p.x = 3.0; ob.p.y = 6.0; ob.radius = 2.0; obs.push_back(ob);
	ob.p.x = 3.0; ob.p.y = 8.0; ob.radius = 2.0; obs.push_back(ob);
	ob.p.x = 3.0; ob.p.y = 10.0; ob.radius = 2.0; obs.push_back(ob);
	ob.p.x = 7.0; ob.p.y = 5.0; ob.radius = 2.0; obs.push_back(ob);
	ob.p.x = 9.0; ob.p.y = 5.0; ob.radius = 2.0; obs.push_back(ob);
	ob.p.x = 8.0; ob.p.y = 10.0; ob.radius = 1.0; obs.push_back(ob);

	pos start; start.x = 0.0; start.y = 0.0;
	pos goal; goal.x = 8.0; goal.y = 12.0;
	area rand; rand.max.x = rand.max.y = 15.0; rand.min.x = rand.min.y = -2.0;
	area play; play.max.x = play.max.y = 15.0; play.min.x = play.min.y = 0.0;

	rrt::cfg cfg;
	cfg.pathResolution = 0.1;
	cfg.maxIter = 1000;
	cfg.expand_dis = 3.0;
	cfg.safety = 0.1;

	//std::unique_ptr<rrtBase> r_rrt = std::make_unique<rrt>(start, goal, rand, play, obs, cfg);
	std::unique_ptr<rrtBase> r_rrt = std::make_unique<rrtStar>(start, goal, rand, play, obs, cfg);

	auto path = r_rrt->planning();



	out["play"]["x"]["min"] = play.min.x;
	out["play"]["y"]["min"] = play.min.y;
	out["play"]["x"]["max"] = play.max.x;
	out["play"]["y"]["max"] = play.max.y;
	out["start"]["x"] = start.x;
	out["start"]["y"] = start.y;
	out["goal"]["x"] = goal.x;
	out["goal"]["y"] = goal.y;
	for (int i = 0; i < obs.size(); ++i) {
		out["obstacle"][std::to_string(i)]["x"] = obs[i].p.x;
		out["obstacle"][std::to_string(i)]["y"] = obs[i].p.y;
		out["obstacle"][std::to_string(i)]["radius"] = obs[i].radius;
	}

	auto buff = r_rrt->getDebugNodes();
	for (int i = 0; i < buff.size(); ++i)
	{
		if (buff[i]->_path.size()) {
			out["tries"][std::to_string(i)]["start"]["x"] = buff[i]->_path[0].x;
			out["tries"][std::to_string(i)]["start"]["y"] = buff[i]->_path[0].y;
			out["tries"][std::to_string(i)]["end"]["x"] = buff[i]->_path.back().x;
			out["tries"][std::to_string(i)]["end"]["y"] = buff[i]->_path.back().y;
		}
	}

	for (int i = 0; i < path.size(); ++i)
	{
		out["path"][std::to_string(i)]["start"]["x"] = path[i].x;
		out["path"][std::to_string(i)]["start"]["y"] = path[i].y;
		out["path"][std::to_string(i)]["end"]["x"] = path[i].x;
		out["path"][std::to_string(i)]["end"]["y"] = path[i].y;
	}

	env<<out<<std::endl;
	return 0;
}