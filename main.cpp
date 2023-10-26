// Copyright 2023 watson.wang

#include <fstream>
#include "nlohmann/json.hpp"
#include "rrt.hpp"
#include "rrt_star.hpp"
#include "rrt_star_rs.hpp"

using json = nlohmann::json;

int main() {
	json out;
	std::vector<obstacle> obs;
	obstacle ob;
	std::ofstream env("../env.json");

	obs.push_back({ 5.0, 5.0, 1.0 });
	obs.push_back({ 3.0, 6.0, 2.0 });
	obs.push_back({ 3.0, 8.0, 2.0 });
	obs.push_back({ 3.0, 10.0, 2.0 });
	obs.push_back({ 7.0, 5.0, 2.0 });
	obs.push_back({ 9.0, 5.0, 2.0 });
	obs.push_back({ 8.0, 10.0, 1.0 });

	pos start{ 1.0, 1.0 };
	pos goal{ 8.0, 8.0 };
	area rand; rand.max.x = rand.max.y = 15.0; rand.min.x = rand.min.y = -2.0;
	area play; play.max.x = play.max.y = 15.0; play.min.x = play.min.y = 0.0;

	rrt::cfg cfg;
	cfg.pathResolution = 0.1;
	cfg.maxIter = 500;
	cfg.expand_dis = 3.0;
	cfg.safety = 0.1;

	//std::unique_ptr<rrtBase> r_rrt = std::make_unique<rrt>(start, goal, rand, play, obs, cfg);
	//std::unique_ptr<rrtBase> r_rrt = std::make_unique<rrtStar>(start, goal, rand, play, obs, cfg);
	std::unique_ptr<rrtBase> r_rrt = std::make_unique<rrtStarRs>(start, goal, rand, play, obs, cfg);

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
		nlohmann::json xpth = nlohmann::json::array();
		nlohmann::json ypth = nlohmann::json::array();

		for (auto pth : buff[i]->_path) {
			xpth.push_back(pth.x);
			ypth.push_back(pth.y);
		}

		if (buff[i]->_path.size()) {
			out["tries"][std::to_string(i)]["x"] = xpth;
			out["tries"][std::to_string(i)]["y"] = ypth;
		}
	}

	nlohmann::json oxpth = nlohmann::json::array();
	nlohmann::json oypth = nlohmann::json::array();
	for (int i = 0; i < path.size(); ++i)
	{
		oxpth.push_back(path[i].x);
		oypth.push_back(path[i].y);
	}
	out["path"]["x"] = oxpth;
	out["path"]["y"] = oypth;

	env<<out<<std::endl;
	return 0;
}