//
// Created by adrien_bedel on 18/11/19.
//

#include "graphics/DirectionField.h"

////////////////////////
//
//      GLOBALS
//
///////////////////////

bool ADD_BORDER(int obj) { return obj == VECTOR_ZONE || obj == ORTHO_ZONE; }
bool IS_ISOTROPY(int obj) { return obj == ISOTROPY || obj == ANISOTROPY; }
bool IS_VECTOR(int obj) { return obj >= VECTOR_FIELD && obj <= ORTHO_ZONE; }
bool IS_ORTHO(int obj) { return obj == ORTHO_VECTOR || obj == ORTHO_ZONE; }

// calculate variable depending on printer
void Globals::initVariables(int layerNb) {
	// calculate frame in transversal area
	_frameInArea = M_PI * _frameRayIn * _frameRayIn;
	// calculate Extrusion value based on the nozzle
	_extrusionHeight = (_outRPrismeHeight * _outRPrismeWidth) / _frameInArea;
	// calculate the length of frame in to push the nozzle volume
	_lengthToClear = _buzeVolume / _frameInArea;
	// Update DirectionFields
	DirectionField::init(layerNb);
}

// area is positive in counter-clockwise
double Globals::polygonArea(const std::vector<glm::dvec2> &points) {
	double area = 0.;
	for (int i = 1; i < (int) points.size(); i++)
		area += (points[i-1].x - points[i].x) * (points[i-1].y + points[i].y);
	return area / 2.;
}

// check if point is inside points
bool Globals::isInPoly(const std::vector<glm::dvec2> &points, const glm::dvec2 &point) {
	bool inside = false;
	for (int i = 1; i < (int) points.size(); i++) {
		double diff = points[i].y - points[i-1].y;
		if(!diff) continue;
		if(points[i].y == point.y) {
			if(points[i].x > point.x && diff < 0.) inside = !inside;
		} else if(points[i-1].y == point.y) {
			if(points[i-1].x > point.x && diff > 0.) inside = !inside;
		} else if((points[i].y > point.y) != (points[i-1].y > point.y)) {
			double d = (points[i].x - point.x) * diff + (points[i].x - points[i-1].x) * (point.y - points[i].y);
			if(diff > 0) { if(d > 0) inside = !inside; }
			else { if(d < 0) inside = !inside; }
		}
	}

	return inside;
}

bool Globals::intersect(const glm::dvec2 &a, const glm::dvec2 &b, const glm::dvec2 &c, const glm::dvec2 &d, double &t) {
	glm::dvec2 v = c - a;
	glm::dvec2 w = d - b;
	t = (b.x - a.x) * w.y - (b.y - a.y) * w.x;
	t /= v.x * w.y - v.y * w.x;
	if(t >= 0. && t <= 1.) {
		double u;
		if(std::abs(w.x) > std::abs(w.y)) u = (a.x + t*v.x - b.x) / w.x;
		else u = (a.y + t*v.y - b.y) / w.y;
		if(u >= 0. && u <= 1.) return true;
	}
	return false;
}

bool Globals::intersect(const glm::dvec2 &a, const glm::dvec2 &b, const glm::dvec2 &c, const glm::dvec2 &d) {
	double t;
	return intersect(a, b, c, d, t);
}
