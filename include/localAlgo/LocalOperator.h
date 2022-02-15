//
// Created by adrien_bedel on 11/12/19.
//

#ifndef HAMILTON_LOCALOPERATOR_H
#define HAMILTON_LOCALOPERATOR_H

#include "ObjectiveFunctions.h"
#include "initialization/GraphCreator.h"

#include <random>

class LocalOperator : public ObjectiveFunctions {
private:
	const static std::vector<std::vector<std::pair<int, int>>> 	_segments;
	unsigned int												_iter = 0;
	bool														_isEnd = false;
	std::vector<int>											_index;
	std::pair<std::vector<glm::vec3>, std::vector<glm::vec3>>	_final;
	std::vector<Shape>											_colorZones;
	std::mt19937												_gen;

public:
	LocalOperator() = default;
	LocalOperator(Shape &shape, std::vector<Shape> &zones, std::vector<Shape> &&colorZones, int layerIndex);
	void setGraph(Graph &graph);

	void	updateState(bool);
	void	startShuffling(int, int);
	void	optimize();
	const std::pair<std::vector<glm::vec3>, std::vector<glm::vec3>>&	getFinal() const;
	const std::vector<Shape>&											getColorZones() const;

private:
	bool	checkPaperOp(int);
	bool	isLinked(const std::vector<int> &, int);
	bool	switchConditions(float);
	void	computeIndex(int, int);

	std::pair<float, std::vector<int>> 	checkFlip(const std::vector<int> &);
	std::pair<float, std::vector<int>> 	checkTranspose(const std::vector<int> &);
	std::pair<float, std::vector<int>> 	checkCross(const std::vector<int> &);
	std::pair<float, std::vector<int>> 	checkZigZag(const std::vector<int> &);

};

#endif //HAMILTON_LOCALOPERATOR_H
