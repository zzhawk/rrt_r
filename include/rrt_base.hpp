// Copyright 2023 watson.wang

#include <memory>
#include <vector>
#include <random>

struct pos {
	double x;
	double y;
};

struct area {
	pos min;
	pos max;
};

struct obstacle {
	pos p;
	double radius;
};
struct node;

using nodeSharedPtr = std::shared_ptr<node>;
//using NodeConstSharedPtr = std::shared_ptr<const node>;
using nodeWeakPtr = std::weak_ptr<node>;

struct node {
	explicit node(pos p) :_p(p){

	}
	virtual ~node() = default;

	node(const node&) = default;
	node(node&&) noexcept = default;

	node& operator = (const node&) = default;
	node& operator = (node&&) noexcept = default;

	pos _p{};
	std::vector<pos> _path{};
	nodeWeakPtr _parent;
	std::vector<nodeSharedPtr> _childs = std::vector<nodeSharedPtr>();
};



class rrtBase {
public:
	struct cfg {
		double pathResolution = 0.5;
		int max_iter = 500;
		double expand_dis = 3.0;
		double safety = 0.1;
	};

	explicit rrtBase(pos start, pos goal, area randArea, area playArea, std::vector<obstacle> obstacles, cfg cfg) :
		_start(start), _goal(goal), _randArea(randArea), _playArea(playArea), _obstacles(obstacles), _cfg(cfg){
		std::random_device seed;
		std::ranlux48 engine(seed());
		_engine = engine;

		_startNode = std::make_shared<node>(_start);
		_nodes.push_back(_startNode);
		_goalNode = std::make_shared<node>(_goal);

		_longest = calDist(_playArea.min, _playArea.max);
	};
	virtual ~rrtBase() = default;
	
	rrtBase(const rrtBase &) = default;
	rrtBase(rrtBase &&) noexcept = default;

	rrtBase& operator = (const rrtBase&) = default;
	rrtBase& operator = (rrtBase&&) noexcept = default;

	virtual std::vector<pos> planning() = 0;
	virtual nodeSharedPtr steer(nodeSharedPtr& fnd, nodeSharedPtr& tnd, double lenth) = 0;

	virtual nodeSharedPtr getRandNode();

	std::vector<nodeSharedPtr> getDebugNodes();
protected:
	pos _start{};
	pos _goal{};
	area _randArea{};
	area _playArea{};
	std::vector<obstacle> _obstacles;
	cfg _cfg{};
	std::vector<nodeSharedPtr> _nodes;
	nodeSharedPtr _startNode{};
	nodeSharedPtr _goalNode{};
	double _longest{};

	std::vector<pos> getResult();
	nodeSharedPtr getNearestNode(nodeSharedPtr& nd) const;
	bool isOutsideArea(nodeSharedPtr& nd) const;
	bool isCollision(nodeSharedPtr& nd) const;
	double calDist(const pos& fnd, const pos& tnd) const;
	double calAngle(const pos& fnd, const pos& tnd) const;

private:
	std::ranlux48 _engine{};
	double _goalSampleRate = 5.0;
};