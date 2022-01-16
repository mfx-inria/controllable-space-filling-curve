//
// Created by adrien_bedel on 18/11/19.
//

#include "graphics/Globals.h"
#include "graphics/Window.h"
#include "graphics/DirectionField.h"

#include <queue>
#include <iostream>

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
	_extrusionHeight = (_millimeter * _outRPrismeHeight * _outRPrismeWidth) / _frameInArea;
	// calculate the length of frame in to push the nozzle volume
	_lengthToClear = _buzeVolume / _frameInArea;
	// Update DirectionFields
	DirectionField::init(layerNb);
}

// area is positive in counter-clockwise
float Globals::polygonArea(const std::vector<glm::vec2> &points) {
	float area = 0.f;
	for (int i = 1; i < points.size(); i++)
		area += (points[i-1].x - points[i].x) * (points[i-1].y + points[i].y);
	return area / 2.f;
}

// check if point is inside points
bool Globals::isInPoly(const std::vector<glm::vec2> &points, const glm::vec2 &point) {
	bool inside = false;
	for (int i = 1; i < points.size(); i++) {
		float diff = points[i].y - points[i-1].y;
		if(!diff) continue;
		if(points[i].y == point.y) {
			if(points[i].x > point.x && diff < 0.) inside = !inside;
		} else if(points[i-1].y == point.y) {
			if(points[i-1].x > point.x && diff > 0.) inside = !inside;
		} else if((points[i].y > point.y) != (points[i-1].y > point.y)) {
			float d = (points[i].x - point.x) * diff + (points[i].x - points[i-1].x) * (point.y - points[i].y);
			if(diff > 0) { if(d > 0) inside = !inside; }
			else { if(d < 0) inside = !inside; }
		}
	}

	return inside;
}

bool Globals::intersect(const glm::vec2 &a, const glm::vec2 &b, const glm::vec2 &c, const glm::vec2 &d) {
	glm::vec2 v = c - a;
	glm::vec2 w = d - b;
	float t = (b.x - a.x) * w.y - (b.y - a.y) * w.x;
	t /= v.x * w.y - v.y * w.x;
	if(t >= 0. && t <= 1.) {
		float u;
		if(std::abs(w.x) > std::abs(w.y)) u = (a.x + t*v.x - b.x) / w.x;
		else u = (a.y + t*v.y - b.y) / w.y;
		if(u >= 0. && u <= 1.) return true;
	}
	return false;
}

bool Globals::intersect(const glm::vec2 &a, const glm::vec2 &b, const glm::vec2 &c, const glm::vec2 &d, float &t) {
	glm::vec2 v = c - a;
	glm::vec2 w = d - b;
	t = (b.x - a.x) * w.y - (b.y - a.y) * w.x;
	t /= v.x * w.y - v.y * w.x;
	if(t >= 0. && t <= 1.) {
		float u;
		if(std::abs(w.x) > std::abs(w.y)) u = (a.x + t*v.x - b.x) / w.x;
		else u = (a.y + t*v.y - b.y) / w.y;
		if(u >= 0. && u <= 1.) return true;
	}
	return false;
}

void Globals::getInter(const glm::vec2 &a, const glm::vec2 &b, const Shape &shape, std::vector<float> &ts) {
	float t;
	for(int i = 1; i < shape._points.size(); ++i)
		if(intersect(a, shape._points[i-1], b, shape._points[i], t))
			ts.push_back(t);
	for(const std::vector<glm::vec2> &hole : shape._holes)
		for(int i = 1; i < hole.size(); ++i)
			if(intersect(a, hole[i-1], b, hole[i], t))
				ts.push_back(t);
}

// get the centroid of a set of points
glm::vec2 Globals::getCenter(const std::vector<glm::vec2> &points) {
	glm::vec2 center(0.f, 0.f);
	float area = 0.;
	for(int i = 0; i < points.size(); ++i) {
		const glm::vec2 &a = points[i], &b = points[(i+1)%points.size()];
		float dx = b.x - a.x;
		glm::vec2 s = a+b;
		area += dx * s.y / 2.;
		center += dx * (b.y*b + s.y*s + a.y*a) / glm::vec2(6.f, 12.f);
	}
	center /= area;
	return center;
}

// ignore Z component
float Globals::getDist(const glm::vec3 &a, const glm::vec3 &b) {
	return (sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)));
}


//////////////////
//
//      SHAPE
//
//////////////////

// is p inside of _points
bool Shape::isInside(const glm::vec2 &p) const {
	if(!Globals::isInPoly(_points, p)) return false;
	for(const std::vector<glm::vec2> &hole : _holes)
		if(Globals::isInPoly(hole, p))
			return false;
	return true;
}


//////////////////
//
//      GRAPH
//
//////////////////

// get voronoi cell's center from its points
glm::vec2 Graph::getCellCenter(const std::vector<int> &cell, const std::vector<glm::vec2> &points) {
	glm::vec2 center(0.f, 0.f);
	float area = 0.;
	for(int i = 0; i < cell.size(); ++i) {
		const glm::vec2 &a = points[cell[i]], &b = points[cell[(i+1)%cell.size()]];
		float dx = b.x - a.x;
		glm::vec2 s = a+b;
		area += dx * s.y / 2.;
		center += dx * (b.y*b + s.y*s + a.y*a) / glm::vec2(6.f, 12.f);
	}
	center /= area;
	return center;
}

