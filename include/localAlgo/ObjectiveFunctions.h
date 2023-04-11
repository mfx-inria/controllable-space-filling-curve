//
// Created by adrien on 17/11/2020.
//

#ifndef HAMILTON_OBJECTIVEFUNCTIONS_H
#define HAMILTON_OBJECTIVEFUNCTIONS_H

#include "graphics/Shape.h"

class ObjectiveFunctions {

struct Edge {
	int zone;
	int i, j;
	double len, l2;
	Edge() = default;
	Edge(int z): zone(z) {}
};
struct Segment {
	double              vecW, vecS;
	std::vector<Edge>   iso;
	int                 zon;
	Segment(): vecW(0.), vecS(0.), iso(), zon(0) {}
};

protected:
	Shape                           _shape;
	std::vector<Shape>              _objZones;
	std::vector<glm::dvec2>          _points;
	std::vector<std::vector<int>>   _links;
	std::vector<std::vector<int>>   _cLinks;

	int                                     _layerIndex;
	double                                  _score = 0.;
	// ISOTROPY
	const static int                        _nSamples = 100;
	std::vector<std::vector<double>>        _distribs;
	// VECTOR
	double                                  _segCellRadius;
	double                                  _vecArea;
	double                                  _vecWeight;
	double                                  _vecScore;
	// ZONING
	int                                     _nCrosses;
	
	int                                     _nbUpdates;
	std::vector<std::vector<Segment>>       _segments;

public:
	std::vector<int>                _zone;

public:
	ObjectiveFunctions() = default;
	ObjectiveFunctions(Shape &&shape, std::vector<Shape> &&objZones, int layerIndex);

	double  checkDirection(const std::vector<int> &, const std::vector<std::pair<int, int>> &, const std::vector<std::pair<int, int>> &);
	void    applyShift(const std::vector<int> &, const std::vector<std::pair<int, int>> &, const std::vector<std::pair<int, int>> &);

	double  getScore() const;
	void    calculateScore();
	const std::vector<glm::dvec2>&                                                                                               getPoints() const;
	const std::vector<std::vector<int>> &                                                                                       getLinks() const;
	std::tuple<const std::vector<glm::dvec2> &, const std::vector<std::vector<int>> &, const std::vector<std::vector<int>> &>    getGraph() const;
	std::pair<const std::vector<glm::dvec2> &, const std::vector<std::vector<int>> &>                                            getCycle() const;
	const Shape &               getBorder() const;
	const std::vector<Shape>&   getObjZones() const;
	int                         getColor(int) const;
	double                      getArea() const;

protected:
	void                        computeZone();
	void                        computeData();

private:
	double                      getMeanScore();
	const Segment&              getSegment(int, int);
	void                        addSegment(int, int);
	void                        rmSegment(int, int);
	void                        removeLink(int, int);
	void                        createLink(int, int);

	void    createEdgeVec(const glm::dvec2 &, const glm::dvec2 &, double &, double &);
	Edge    createEdgeIso(int, const glm::dvec2 &);
	double  getWassersteinDistance(int) const;

};

#endif //HAMILTON_OBJECTIVEFUNCTIONS_H
