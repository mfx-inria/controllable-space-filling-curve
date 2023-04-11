//
// Created by bedela on 18/05/2020.
//

#ifndef HAMILTON_GRAPHCREATOR_H
#define HAMILTON_GRAPHCREATOR_H

#include "graphics/Shape.h"

// Class for graphs
class Graph {
public:
	std::vector<glm::dvec2>              _points;
	std::vector<std::vector<int>>       _links;
	std::vector<std::vector<int>>       _cells;

	Graph() = default;

	static glm::dvec2 getCellCenter(const std::vector<int> &cell, const std::vector<glm::dvec2> &points);
	bool static initGraph(const Shape &shape, Graph &graph, int layerIndex);
};

#endif //HAMILTON_GRAPHCREATOR_H
