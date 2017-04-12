#ifndef _ASTAR_H_
#define _ASTAR_H_

#include "ros/ros.h"
#include "pioneer_control/Vec2.h"
#include <vector>
#include <queue>
#include <iterator>
#include <algorithm>

class AStar;

class Node
{
	public:
		Node(Vec2i pos)
			: pos(pos), g(0), h(0), parent(0) {};
		Node(Vec2i pos, int g, int h)
			: pos(pos), g(g), h(h), parent(0) {};
		Node(Vec2i pos, int g, int h, Node *parent)
			: pos(pos), g(g), h(h), parent(parent) {};
		Node &operator=(const Node& e)
		{
			pos = e.pos;
			g = e.g;
			h = e.h;
			parent = parent;
			return *this;
		};

		int getScore() { return g + h; }
		Vec2i pos;
		int g, h;
		Node *parent;
		//Vec2i lastDir;
};

class NodeList
{
	typedef std::vector<Node*>::iterator NodeListIt;
	private:
	public:
		std::vector<Node*> list;
		bool empty() {return list.empty(); };

		Node* popMinScore()
		{
			NodeListIt min_it = list.begin();
			for(NodeListIt it = list.begin()+1; it < list.end(); it++)
			{ 
				if((*it)->getScore() < (*min_it)->getScore()) min_it = it;
			}
			if(min_it == list.end()) return NULL;
			Node *min = *min_it;
			list.erase(min_it);
			return min;
		};
		void push(Node* node) { list.push_back(node); };

		//Node* find_generic((bool *)f(Node *))
		//{
		//	for(NodeListIt it = list.begin(); it < list.end(); it++)
		//		if(f(*it)) return *it;
		//	return NULL;
		//};

		Node* findNodeWithPos(Vec2i pos)
		{
			for(NodeListIt it = list.begin(); it < list.end(); it++)
				if((*it)->pos == pos) return *it;
			return NULL;
		}

		void clear()
		{
			for(NodeListIt it = list.begin(); it < list.end(); it++)
				delete *it;
			list.clear();

		};
};

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
		bool detectCollision(Vec2i pos);
		NodeList closed;
		NodeList open;
		int distance(Vec2i origin, Vec2i goal);
		static const Vec2i dirs[4];
};

const Vec2i AStar::dirs[4] = {Vec2i(0, 1), Vec2i(1, 0), Vec2i(0, -1), Vec2i(-1, 0)};

#define ABS(var) ((var>0)?var:-var)
int AStar::distance(Vec2i origin, Vec2i goal)
{
	Vec2i result = origin - goal;
	//printf("distance = %d\n", ABS(result.x) + ABS(result.y));
	return ABS(result.x) + ABS(result.y);
//	if (map[origin.y][origin.x] == 0 && map[goal.y][goal.x] == 0)
//	{
//		distance = ABS(origin.x - goal.x) + ABS(origin.y - goal.y);
//		distance /= 2;
//		if(origin.x == goal.x) {
//			if(origin.x == 0 || origin.x == mapWidth-1)
//				distance += 2;
//		} else if(origin.y == goal.y) {
//			if(origin.y == 0 || origin.y == mapHeight-1)
//				distance += 2;
//		}
//		return distance;
//	}
//	else
//	{
//		return 0;
//	}
}

bool AStar::detectCollision(Vec2i pos)
{
	if(pos.x < 0 || pos.y < 0)
		return true;
	if(pos.x > mapWidth-1 || pos.y > mapHeight-1)
		return true;
	if(map[pos.y][pos.x] < 0)
		return true;
	return false;
}

int mapstring[5][5] = {
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0}
};

void current_m(Vec2i cur) { mapstring[cur.y][cur.x] = 2; }
void open_m(Vec2i cur) { mapstring[cur.y][cur.x] = 1; }
void replace_m(Vec2i cur) { mapstring[cur.y][cur.x] = 3; }

void printmap()
{
	for(int i = 0; i < 5; i++)
	{
		printf("|");
		for(int j = 0; j < 5; j++)
		{
			switch(mapstring[i][j])
			{
				case 0:
					printf(".");
					break;
				case 1:
					printf("o");
					break;
				case 2:
					printf("@");
					break;
				case 3:
					printf("x");
					break;
			}
		}
		printf("|\n");
	}

}
void reset_m()
{ for(int i = 0; i < 5; i++) for(int j = 0; j < 5; j++) mapstring[i][j] = 0; }

std::vector<Vec2i> AStar::findPath(Vec2i origin, Vec2i goal)
{
	std::vector<Vec2i> response;

	Node *current = new Node(origin, 0, distance(origin, goal), NULL);
	open.push(current);
	while (!open.empty())
	{
		current = open.popMinScore();
		//printf("FROM (%d, %d):\n",  current->pos.x, current->pos.y);
		current_m(current->pos);
		if (current->pos == goal)
		{
			printf("RESULT FOUND\n");
			response.clear();
			while (current != NULL)
			{
				response.push_back(current->pos);
				printf("\t(%d, %d)\n", current->pos.x, current->pos.y);
				current = current->parent;
			}
			printf("DONE!\n");

			std::reverse(response.begin(), response.end());

			reset_m();
			open.clear();
			closed.clear();
			return response;
		}

		closed.push(current);
		for(int i = 0; i < 4; i++)
		{
			Vec2i nextPos = current->pos + dirs[i];
			//printf("(%d, %d)", nextPos.x, nextPos.y);
			if(detectCollision(nextPos))
				{ /*printf(" collision\n");*/ continue; }

			if(closed.findNodeWithPos(nextPos)) 
				{ /*printf(" is in closed set\n");*/ continue; }

			//finge que não viu essa atrocidade
			//corrigirei se tiver tempo
			int this_g = current->g + 1;
			int this_h = distance(nextPos, goal);
			Node *node = open.findNodeWithPos(nextPos);
			if(node == NULL)
			{
				open.push(new Node(nextPos, current->g + 1,
							distance(nextPos, goal), current)); 
				//printf("OPEN\n");
				open_m(nextPos);
			} else if(node->getScore() < this_g + this_h)
			{
				node->g = this_g;
				node->h = this_h;
				node->parent = current;
				//printf("REPLACE\n");
			}
			//printf("BETTER IN OPEN SET\n");

		}
		//printmap();
	}
	reset_m();

	open.clear();
	closed.clear();

	printf("\nERROR\n");
	std::vector<Vec2i> error;
	error.clear();
	return error;
}

#endif
