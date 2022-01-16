//
// Created by bedela on 26/05/2020.
//

#include "tools/Layer.h"
#include "managers/GraphCreator.h"

void Layer::initLayer(const std::string &fileName, int layerIndex) {
	std::vector<Graph> graphs;
	std::vector<Shape> bounderies;
	std::vector<std::vector<Shape>> zones, strokeZones;
	GraphCreator::graphFromSvg(fileName, bounderies, zones, strokeZones, graphs, layerIndex);

	// match each shape
	_operators.clear();
	for(int i = 0; i < graphs.size(); ++i)
		_operators.emplace_back(zones[i], std::move(strokeZones[i]), bounderies[i], graphs[i], layerIndex);

	// calculte initial score
	_initialScore = 0.f;
	for(LocalOperator &op : _operators) if(op.isSucces()) {
		op.calculateScore();
		_initialScore += op.getScore();
	}
}

float Layer::getScore() const {
	float score = 0.f;
	for(const LocalOperator &op : _operators) if(op.isSucces()) score += op.getScore();
	return score;
}

float Layer::getInitialScore() const
{
	return _initialScore;
}

