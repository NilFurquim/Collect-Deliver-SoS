#ifndef _ASTAR_H_
#define _ASTAR_H_

#include "ros/ros.h"
#include "pioneer_control/Vec2.h"
#include <vector>
#include <queue>
#include <iterator>
#include <algorithm>


class AStar
{
	public:
		AStar() : map(0), mapHeight(0), mapWidth(0) {};
		AStar(int **map, int mapHeight, int mapWidth)
			: map(map), mapHeight(mapHeight), mapWidth(mapWidth) {};
		std::vector<Vec2i> findPath(Vec2i origin, Vec2i goal);
	private:
		int** map;
		int mapHeight, mapWidth;
		bool detectCollision(Vec2i);
		int distance(Vec2i origin, Vec2i goal);
		static const Vec2i dirs[4];

	public:
		class Elem
		{
			public:
				Elem(Vec2i pos, int g, int h)
					: pos(pos), g(g), h(h), parent(0) {};
				Elem(Vec2i pos, int g, int h, Elem *parent)
					: pos(pos), g(g), h(h), parent(parent) {};
				Elem &operator=(const Elem& e)
				{
					pos = e.pos;
					g = e.g;
					h = e.h;
					parent = parent;
					return *this;
				};
				bool operator<(const Elem& e) const
				{ return g+h < e.g+e.h; };
				bool operator>(const Elem& e) const
				{ return g+h > e.g+e.h; };
				bool operator<(const Elem& l, const Elem& r) const
				{ return l.g+l.h > r.g+r.h; };
				bool operator>(const Elem& l, const Elem& r) const
				{ return l.g+l.h > r.g+r.h; };
				Vec2i pos;
				int g;
				int h;
				Elem *parent;
		};
		class ElemLess
		{
			public:
				bool operator()(const Elem& l, const Elem& r) const
				{ return l.g+l.h > r.g+r.h; };
		};

const Vec2i AStar::dirs[4] = {Vec2i(0, 1), Vec2i(1, 0), Vec2i(0, -1), Vec2i(-1, 0)};

#define ABS(var) ((var>0)?var:-var)
int AStar::distance(Vec2i origin, Vec2i goal)
{
	int distance = 0;
	if (map[origin.y][origin.x] == 0 && map[goal.y][goal.x] == 0)
	{
		distance = ABS(origin.x - goal.x) + ABS(origin.y - goal.y);
		distance /= 2;
		if(origin.x == goal.x) {
			if(origin.x == 0 || origin.x == mapWidth-1)
				distance += 2;
		} else if(origin.y == goal.y) {
			if(origin.y == 0 || origin.y == mapHeight-1)
				distance += 2;
		}
		return distance;
	}
	else
	{
		return 0;
	}
}

bool AStar::detectCollision(Vec2i pos)
{
	if(pos.x <= 0 || pos.y <= 0)
		return true;
	if(pos.x >= mapWidth-1 || pos.y >= mapHeight)
		return true;
	if(map[pos.y][pos.x] < 0)
		return true;
	return false;
}

class ElemLess
{
}
std::vector<Vec2i> AStar::findPath(Vec2i origin, Vec2i goal)
{
#if 0
	std::priority_queue<Elem, vector<Elem>, ElemLess> open;
	std::vector<Vec2i> closed;
	Elem current(origin, 0, -distance(origin,goal));
	open.push(current);
	bool isInClosedSet;
	while (!open.empty())
	{
		current = open.top();
		ROS_INFO("CLOSED");
		ROS_INFO("current (%d, %d)\n\tf: %d\n", current.pos.x, current.pos.y, current.h + current.g);
		open.pop();
		if (current.pos == goal)
			break;

		closed.push_back(current.pos);
		for(int i = 0; i < 4; i++)
		{
			Vec2i nextPos = current.pos;
			nextPos += dirs[i];
			if(detectCollision(nextPos))
			{ continue; }
			for(int j = 0; j < closed.size(); j++) {
				if(closed[j] == nextPos)
				{ isInClosedSet = true; break; }
			}
			if(isInClosedSet) continue;
			ROS_INFO("OPEN");
			ROS_INFO("open[%d] (%d, %d)\n\tf: %d\n", i, nextPos.x, nextPos.y, -(current.g + 1)+ -distance(nextPos, goal));
			open.push(Elem(nextPos,-(current.g + 1), -distance(nextPos, goal), &current)); 
		}
	}

	std::vector<Vec2i> response;
	while (current.parent != NULL)
	{
		ROS_INFO("(%d, %d)\n", current.pos.x, current.pos.y);
		current = *current.parent;
	}

	return response;
#endif
}

#endif
