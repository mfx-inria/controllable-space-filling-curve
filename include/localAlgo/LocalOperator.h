//
// Created by adrien_bedel on 11/12/19.
//

#ifndef HAMILTON_LOCALOPERATOR_H
#define HAMILTON_LOCALOPERATOR_H

#include "ObjectiveFunctions.h"

#include <random>

class LocalOperator : public ObjectiveFunctions {
private:
	const static std::vector<std::vector<std::pair<int, int>>> 	_segments;
	std::default_random_engine									_gen;
	unsigned int												_iter = 0;
	bool														_succes;
	bool														_isEnd = false;
	std::vector<int>											_index;
	std::pair<std::vector<glm::vec3>, std::vector<glm::vec3>>	_final;
	std::vector<Shape>											_strokeZones;

public:
	LocalOperator() = default;
	LocalOperator(std::vector<Shape> &, std::vector<Shape> &&, Shape &, Graph &, int);

	void                            updateState(bool);
	void                            startShuffling(int, int);
	void                            optimize();
	bool                            isSucces() const;
	const std::pair<std::vector<glm::vec3>, std::vector<glm::vec3>>&    getFinal() const;
	const std::vector<Shape>&                                           getStrokeZones() const;

private:
	bool    checkPaperOp(int);
	bool    isLinked(const std::vector<int> &, int);
	bool    switchConditions(float);
	void    computeIndex(int, int);

	// FLIP
	std::pair<float, std::vector<int>>  checkFlip(const std::vector<int> &);
	// TRANSPOSE
	std::pair<float, std::vector<int>>  checkTranspose(const std::vector<int> &);
	// CROSS
	std::pair<float, std::vector<int>>  checkCross(const std::vector<int> &);
	// ZIGZAG
	std::pair<float, std::vector<int>>  checkZigZag(const std::vector<int> &);

};

#endif //HAMILTON_LOCALOPERATOR_H
