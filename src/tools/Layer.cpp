//
// Created by bedela on 26/05/2020.
//

#include "tools/Layer.h"
#include "initialization/GraphCreator.h"

void Layer::initLayer(const std::string &fileName, int layerIndex) {
	std::vector<Shape> shapes;
	std::vector<std::vector<Shape>> objZones, colorZones;
	Shape::read(fileName, shapes, objZones, colorZones, layerIndex);
	_operators.clear();
	for(int i = 0; i < (int) shapes.size(); ++i)
		_operators.emplace_back(shapes[i], objZones[i], std::move(colorZones[i]), layerIndex);
}

void Layer::initCycle(int layerIndex) {
	for(int i = 0; i < (int) _operators.size(); ++i) {
		Graph graph;
		if(!Graph::initGraph(_operators[i].getBorder(), graph, layerIndex)) {
			if(i+1 < (int) _operators.size()) _operators[i] = std::move(_operators.back());
			_operators.pop_back();
		} else {
			_operators[i].setGraph(graph);
			_operators[i].calculateScore();
		}
	}
}
