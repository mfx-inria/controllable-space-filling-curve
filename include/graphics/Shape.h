#ifndef HAMILTON_SHAPE_H
#define HAMILTON_SHAPE_H

#include "Globals.h"

// Shape containing cycles
class Shape {
public:
	double                               _area;
	OBJECTIVE                            _objcetive = NOTHING;
	int                                  _printColor = 0;
	std::vector<glm::dvec2>              _points;
	std::vector<std::vector<glm::dvec2>> _holes;
	std::vector<std::vector<uint>>       _triangles;

public:
	Shape(){}
	Shape(const std::vector<glm::dvec2> &pts) { _points = pts; }

	bool isNear(const glm::dvec2 &p, double eps2=1e-10) const;
	bool isInside(const glm::dvec2 &p) const;
	bool intersect(const glm::dvec2 &a, const glm::dvec2 &b) const;
	void getInter(const glm::dvec2 &a, const glm::dvec2 &b, std::vector<double> &ts) const;
	
	void triangulate();

	static void read(const std::string &fileName,
					std::vector<Shape> &shapes,
					std::vector<std::vector<Shape>> &objZones,
					std::vector<std::vector<Shape>> &colorZones,
					int layerIndex);
};

#endif //HAMILTON_SHAPE_H