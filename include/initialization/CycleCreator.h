//
// Created by bedela on 01/07/2020.
//

#ifndef HAMILTON_MATCHING_H
#define HAMILTON_MATCHING_H

#include "graphics/Shape.h"
#include "initialization/GraphCreator.h"
#include "initialization/UnionFind.h"

class CycleCreator {
public:
	int _nbConnectedPoints;
	std::vector<std::vector<int>>   _cLinks;
	std::vector<std::vector<int>>   _links;
	std::vector<glm::vec2>          _points;
	UnionFind						_union;

	CycleCreator(const Shape &shape, Graph &graph);

private:
	void    checkInit();
	void    perfectMatching();
	void    switchLink();
	void    addCenters(const Shape &shape, const Graph &graph);
	void    fuseIslands();
	void    removeUnused(const Shape &shape);
	int     getNext(int, int);
	void    createLink(int, int);
	void    removeLink(int, int);
	std::vector<int>    getIdxs(int i);
	std::vector<int>    getPath(int, int);
};

#endif //HAMILTON_MATCHING_H
