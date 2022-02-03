#ifndef HAMILTON_SHAPE_H
#define HAMILTON_SHAPE_H

#include "Globals.h"

// Shape containing cycles
class Shape {
public:
	float                               _area;
	OBJECTIVE                           _objcetive = NOTHING;
	int                                 _printColor = 0;
	std::vector<glm::vec2>              _points;
	std::vector<std::vector<glm::vec2>> _holes;
	std::vector<std::vector<uint>>      _triangles;

public:
	Shape(){}
	Shape(const std::vector<glm::vec2> &pts) { _points = pts; }

	bool isNear(const glm::vec2 &p, float eps2=1e-10f) const;
	bool isInside(const glm::vec2 &p) const;
	void getInter(const glm::vec2 &a, const glm::vec2 &b, std::vector<float> &ts) const;
	
	void triangulate();

	static void read(const std::string &fileName,
					std::vector<Shape> &shapes,
					std::vector<std::vector<Shape>> &objZones,
					std::vector<std::vector<Shape>> &colorZones,
					int layerIndex);
};

#endif //HAMILTON_SHAPE_H