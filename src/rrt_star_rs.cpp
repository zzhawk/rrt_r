// Copyright 2023 watson.wang
// https://github.com/zzhawk

#include "rrt_star_rs.hpp"
#include "reeds_shepp.hpp"

using rs = freespace_planning_algorithms::ReedsSheppStateSpace;

nodeSharedPtr rrtStarRs::steer(nodeSharedPtr& fnd, nodeSharedPtr& tnd, double lenth)
{
	auto d = calDist(fnd->_p, tnd->_p);
	auto out = std::make_shared<node>(tnd->_p);

	//if (d <= _cfg.pathResolution) {
	//	return nullptr;
	//}
	//else {

	//	if (lenth > d) lenth = d;

		//calcReedsSheppPath(fnd, tnd)
		auto theta = calAngle(fnd->_p, tnd->_p);
		//out->_path.push_back(fnd->_p);
		//auto n_expand = std::floor(lenth / _cfg.pathResolution);
		
		//pos t = { lenth * std::sin(theta), lenth * std::cos(theta) };
		//out->_p = t;
		out->_p = tnd->_p;
		out->_t = fnd->_t + theta;
		out->_path = calcReedsSheppPath(fnd, out, _radius);

		out->_p = out->_path.back();

	//}

	out->_parent = fnd;
	fnd->_childs.push_back(out);
	return out;
}

double rrtStarRs::calcNewCost(nodeSharedPtr& fnd, nodeSharedPtr& tnd)
{
	return fnd->cost + calcReedsSheppDistance(fnd, tnd, _radius);
}


double rrtStarRs::calcReedsSheppDistance(nodeSharedPtr& p1, nodeSharedPtr& p2, double radius)
{
   const auto rs_space = rs(radius);
   const rs::StateXYT pose0{ p1->_p.x, p1->_p.y, p1->_t };
   const rs::StateXYT pose1{ p1->_p.x, p2->_p.y, p2->_t };
   return rs_space.distance(pose0, pose1);
}

std::vector<pos> rrtStarRs::calcReedsSheppPath(nodeSharedPtr& p1, nodeSharedPtr& p2, double radius)
{
   std::vector<pos> pout;
   const auto rs_space = rs(radius);
	const rs::StateXYT pose0{ p1->_p.x, p1->_p.y, p1->_t };
	const rs::StateXYT pose1{ p2->_p.x, p2->_p.y, p2->_t };
   auto path = rs_space.reedsShepp(pose0, pose1);

   double seg = 0;
   auto p = pose0;
   int iter = static_cast<int>(path.totalLength_ * radius / _cfg.pathResolution) + 1;
   for (int i = 0; i <= iter; ++i) {
      p = rs_space.interpolate(pose0, path, static_cast<double>(i) * _cfg.pathResolution / radius);
      pout.push_back({ p.x, p.y });
   }

   return pout;
}