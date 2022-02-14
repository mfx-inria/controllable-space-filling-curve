//
// Created by bedela on 18/05/2020.
//

#ifndef HAMILTON_GRAPHCREATOR_H
#define HAMILTON_GRAPHCREATOR_H

#include "graphics/Shape.h"

// Class for graphs
class Graph {
public:
	std::vector<glm::vec2>              _points;
	std::vector<std::vector<int>>       _links;
	std::vector<std::vector<int>>       _cells;
public:
	Graph(){}
	static glm::vec2 getCellCenter(const std::vector<int> &, const std::vector<glm::vec2> &);
	bool static initGraph(const Shape &shape, Graph &graph, int layerIndex);
};

#endif //HAMILTON_GRAPHCREATOR_H
