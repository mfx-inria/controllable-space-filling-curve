//
// Created by bedela on 26/05/2020.
//

#include "tools/Layer.h"
#include "managers/GraphCreator.h"

void Layer::initLayer(const std::string &fileName, int layerIndex) {
	std::vector<Shape> bounderies;
	std::vector<std::vector<Shape>> objZones, colorZones;
	Shape::read(fileName, bounderies, objZones, colorZones, layerIndex);
	_operators.clear();
	for(int i = 0; i < (int) bounderies.size(); ++i)
		_operators.emplace_back(objZones[i], std::move(colorZones[i]), bounderies[i], layerIndex);

}

void Layer::initCycle(int layerIndex) {
	for(int i = 0; i < (int) _operators.size(); ++i) {
		Graph graph;
		if(!GraphCreator::graphFromSvg(_operators[i].getBorder(), graph, layerIndex)) {
			if(i+1 < (int) _operators.size()) _operators[i] = std::move(_operators.back());
			_operators.pop_back();
		} else {
			_operators[i].setGraph(graph);
			_operators[i].calculateScore();
		}
	}
}
