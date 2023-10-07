// Copyright 2023 watson.wang

#include "rrt_base.hpp"
#include <cmath>
#include <algorithm>
#include <random>

nodeSharedPtr rrtBase::steer(nodeSharedPtr& fnd, nodeSharedPtr& tnd, double lenth)
{

	auto d = calDist(fnd->_p, tnd->_p);
	auto out = std::make_shared<node>(tnd->_p);

	if (d <= _cfg.pathResolution) {
		//out->_path.push_back(fnd->_p);
		//out->_path.push_back(tnd->_p);
		return nullptr;
	}
	else {

		if (lenth > d) lenth = d;

		auto theta = calAngle(fnd->_p, tnd->_p);
		out->_path.push_back(fnd->_p);
		auto n_expand = std::floor(lenth / _cfg.pathResolution);

		pos t = fnd->_p;
		for (int i = 1; i < n_expand; ++i) {
			t.x += _cfg.pathResolution * std::sin(theta);
			t.y += _cfg.pathResolution * std::cos(theta);
			out->_path.push_back(t);
		}
		out->_p = out->_path.back();

	}

	out->_parent = fnd;
	fnd->_childs.push_back(out);
	return out;
}


std::vector<pos> rrtBase::getResult()
{
	std::vector<pos> ans;
	nodeWeakPtr node = _nodes.back();
	ans.push_back(node.lock()->_p);
	while (!node.lock()->_parent.expired()){
		node = node.lock()->_parent;
		ans.push_back(node.lock()->_p);
	}

	return ans;
}

nodeSharedPtr rrtBase::getRandNode()
{
	pos p;
	std::uniform_real_distribution<double> distr_goal(0.0, 100.0);
	if (distr_goal(_engine) > _goalSampleRate) {
		std::uniform_real_distribution<double> distr_x(_randArea.min.x, _randArea.max.x);
		std::uniform_real_distribution<double> distr_y(_randArea.min.y, _randArea.max.y);
		p.x = distr_x(_engine);
		p.y = distr_y(_engine);
		return std::make_shared<node>(p);
	}
	else {
		return std::make_shared<node>(_goal);
	}
}


nodeSharedPtr rrtBase::getNearestNode(nodeSharedPtr& nd) const
{	
	double min = _longest;
	nodeSharedPtr ans = _nodes[0];
	for (int i = 0; i < _nodes.size(); ++i) {
		auto tmp = calDist(_nodes[i]->_p, nd->_p);
		if (tmp < min) {
			min = tmp;
			ans = _nodes[i];
		}
	}

	return ans;
}


bool rrtBase::isOutsideArea(nodeSharedPtr& nd) const
{
	if (nd->_p.x < _playArea.min.x || nd->_p.x > _playArea.max.x ||
		nd->_p.y < _playArea.min.y || nd->_p.y > _playArea.max.y) {
		return true;
	}
	else {
		return false;
	}
}


bool rrtBase::isCollision(nodeSharedPtr& nd) const
{
	for (auto &obs : _obstacles) {
		for (auto &pth : nd->_path) {
			double dx = pth.x - obs.p.x;
			double dy = pth.y - obs.p.y;
			double dist = std::hypot(dx, dy);
			if (dist < (obs.radius + _cfg.safety)) {
				return true;
			}
		}
	}
	return false;
}


double rrtBase::calDist(const pos &fnd, const pos &tnd) const
{
	double dx = tnd.x - fnd.x;
	double dy = tnd.y - fnd.y;
	return std::hypot(dx, dy);
}


double rrtBase::calAngle(const pos& fnd, const pos& tnd) const
{
	double dx = tnd.x - fnd.x;
	double dy = tnd.y - fnd.y;
	return std::atan2(dx, dy);
}


std::vector<nodeSharedPtr> rrtBase::getDebugNodes()
{
	return _nodes;
}