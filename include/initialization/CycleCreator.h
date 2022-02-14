//
// Created by bedela on 01/07/2020.
//

#ifndef HAMILTON_MATCHING_H
#define HAMILTON_MATCHING_H

#include "graphics/Shape.h"
#include "initialization/GraphCreator.h"
#include "tools/Union.h"

class CycleCreator {
public:
	int _nbConnectedPoints;
	std::vector<std::vector<int>>   _cLinks;
	std::vector<std::vector<int>>   _links;
	std::vector<glm::vec2>          _points;
	Union                           _union;

	CycleCreator(const Shape &shape, Graph &graph);

private:
	void    removeUnused(const Shape &);
	void    checkInit();
	void    perfectMatching();
	void    switchLink();
	void    initUnion();
	void    fuseIslands();
	int     getNext(int, int);
	void    createLink(int, int);
	void    removeLink(int, int);
	std::vector<int>    getIdxs(int, int);
	std::vector<int>    getPath(int, int);
};

#endif //HAMILTON_MATCHING_H
