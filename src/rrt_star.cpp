// Copyright 2023 watson.wang
// https://github.com/zzhawk

#include "rrt_star.hpp"
#include <unordered_map>
#include <map>
#include <cmath>

std::vector<pos> rrtStar::planning()
{

	for (int i = 0; i < _cfg.maxIter; ++i) {
		auto randNode = getRandNode();
		pos p = randNode->_p;
		auto nearestNode = getNearestNode(randNode);
		auto newNode = steer(nearestNode, randNode, _cfg.expand_dis);

		if (newNode != nullptr) {
			newNode->cost = calcNewCost(nearestNode, newNode);

			if (!isOutsideArea(newNode)) {
				if (!isCollision(newNode)) {
					auto nearNodesIdx = findNearNodesIdx(newNode);
					auto newNodeNp = chooseParent(newNode, nearNodesIdx);

					if (newNodeNp != nullptr) {
						rewire(newNodeNp, nearNodesIdx);
						_nodes.push_back(newNodeNp);
					}
					else {
						_nodes.push_back(newNode);
					}


					if ((!search_until_max_iter)&& (newNode)){
						auto lastNode = searchBestGoalNode();
						if (lastNode) {
							_nodes.push_back(lastNode);
							return getResult();
						}
					}
				}
			}
		}
	}

	auto lastNode = searchBestGoalNode();
	if (lastNode) {
		_nodes.push_back(lastNode);
		return getResult();
	}
	return {};
}


nodeSharedPtr rrtStar::chooseParent(nodeSharedPtr& nd, std::vector<int>& nearIdx)
{
	if (nearIdx.size() == 0) return nullptr;

	std::unordered_map<double, nodeSharedPtr> rec;
	double min = _longest;
	for (int i = 0; i < nearIdx.size(); ++i) {

		auto t_node = steer(_nodes[nearIdx[i]], nd, _cfg.expand_dis);

		if (t_node != nullptr) {
			if (!isCollision(t_node)) {
				double cost = calcNewCost(_nodes[nearIdx[i]], t_node);
				rec[cost] = _nodes[nearIdx[i]];
				if (cost < min) min = cost;
			}
			else {
				rec[_longest] = _nodes[nearIdx[i]];
			}
		}
	}

	if (min == _longest) return nullptr;

	auto out = steer(rec[min], nd, _cfg.expand_dis);
	if (out != nullptr) out->cost = min;
	return out;
}

nodeSharedPtr rrtStar::searchBestGoalNode()
{
	std::map<double, nodeSharedPtr, std::less<double>> mp;
	for (auto node : _nodes) {
		auto d = calDist(node->_p, _goalNode->_p);
		if (d < _cfg.expand_dis) {
			mp[d] = node;
		}
	}

	for (auto nd : mp) {
		auto final_node = steer(nd.second, _goalNode, _cfg.expand_dis);
		if (final_node != nullptr) {
			if (!isCollision(final_node)) {
				return final_node;
			}
		}
	}

	return nullptr;
}

std::vector<int> rrtStar::findNearNodesIdx(nodeSharedPtr &nd)
{
	std::vector<int> nearIdx;
	size_t num = _nodes.size();
	double r = connect_circle_dist * std::sqrt(std::log((double)num) / (double)num);
	r = (r > _cfg.expand_dis) ? _cfg.expand_dis : r;
	for (int i = 0; i < num; ++i) {
		if (calDist(_nodes[i]->_p, nd->_p) < r) {
			nearIdx.push_back(i);
		}
	}

	return nearIdx;
}

void rrtStar::rewire(nodeSharedPtr& nd, std::vector<int>& nearIdx)
{
	for (int i = 0; i < nearIdx.size(); ++i) {
		auto edgeNode = steer(nd, _nodes[nearIdx[i]], _cfg.expand_dis);
		if (edgeNode != nullptr) {
			edgeNode->cost = calcNewCost(nd, _nodes[nearIdx[i]]);
			bool no_collision = !isCollision(edgeNode);
			bool improved_cost = _nodes[nearIdx[i]]->cost > edgeNode->cost;

			if (no_collision && improved_cost) {
				for (auto child : _nodes[nearIdx[i]]->_childs) {
						child->_parent = edgeNode;
						edgeNode->_childs.push_back(child);
				}
				_nodes[nearIdx[i]] = edgeNode;
				propagateCostToLeaves(_nodes[nearIdx[i]]);
			}
		}
	}
}

double rrtStar::calcNewCost(nodeSharedPtr& fnd, nodeSharedPtr& tnd)
{
	return fnd->cost + calDist(fnd->_p, tnd->_p);
}

void rrtStar::propagateCostToLeaves(nodeSharedPtr& pd)
{
	for (auto child : pd->_childs) {
		child->cost = calcNewCost(pd, child);
		propagateCostToLeaves(child);
	}
}