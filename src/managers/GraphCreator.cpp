//
// Created by bedela on 18/05/2020.
//

#include "managers/GraphCreator.h"
#include "LBFGS/cvt.hpp"
#include "tools/Random.h"

// If the point is on the border we pushed it inside the shape and return true
bool push_inside(const std::vector<glm::vec2> &border, glm::vec2 &a) {
	constexpr float EPS = 1e-4f;
	for(int k = 1; k < (int) border.size(); ++k) {
		glm::vec2 v = border[k-1] - border[k];
		float l = glm::length(v);
		v /= l;
		float t = std::max(0.f, std::min(l, glm::dot(v, a-border[k])));
		glm::vec2 p = border[k] + t * v;
		if(glm::distance(a, p) > EPS) continue;
		if(t <= EPS) {
			int j = k+1;
			if(j == (int) border.size()) j = 1;
			glm::vec2 w = border[k] - border[j];
			v += w / glm::length(w);
			v /= glm::length(v);
			p = border[k];
		} else if(t+EPS >= l) {
			int j = k-2;
			if(j < 0) j = border.size() - 2;
			glm::vec2 w = border[j] - border[k-1];
			v += w / glm::length(w);
			v /= glm::length(v);
			p = border[k-1];
		}
		v *= EPS;
		a.x = p.x - v.y;
		a.y = p.y + v.x;
		return true;
	}
	return false;
}

void remove2coPoints(const Shape &border, Graph &graph) {
	// Stack containing points to remove
	std::vector<int> stack;
	for (int i = 0; i < (int) graph._points.size(); ++i)
		if(graph._originalLinks[i].size() < 3 && !border.isNear(graph._points[i]))
			stack.push_back(i);

	// Remove loop (update links)
	while(!stack.empty()) {
		int i = stack.back();
		stack.pop_back();
		std::vector<int> &link = graph._originalLinks[i];
		if(link.size() == 2) {
			for(int o = 0; o < 2; ++o) {
				std::vector<int> &link2 = graph._originalLinks[link[0]];
				std::vector<int>::iterator it = std::find(link2.begin(), link2.end(), i);
				if(std::find(link2.begin(), link2.end(), link[1]) == link2.end()) *it = link[1];
				else {
					*it = link2.back();
					link2.pop_back();
					if(link2.size() < 3 && !border.isNear(graph._points[link[0]])) stack.push_back(link[0]);
				}
				std::swap(link[0], link[1]);
			}
		} else if(link.size() == 1) {
			std::vector<int> &link2 = graph._originalLinks[link[0]];
			std::vector<int>::iterator it = std::find(link2.begin(), link2.end(), i);
			*it = link2.back();
			link2.pop_back();
			if(link2.size() < 3 && !border.isNear(graph._points[link[0]])) stack.push_back(link[0]);
		}
		link.clear();
	}

	// Create new indices for remaining points
	std::vector<int> newIndices(graph._points.size(), -1);
	int newSize = 0;
	for(int i = 0; i < (int) newIndices.size(); ++i)
		if(!graph._originalLinks[i].empty())
			newIndices[i] = newSize++;

	// Update cells
	for(std::vector<int> &cell : graph._cells) {
		int s = 0;
		for(int i : cell) {
			int k = newIndices[i];
			if(k != -1) cell[s++] = k;
		}
		cell.resize(s);
	}

	// Swapping points and links
	for(int i = 0; i < (int) newIndices.size(); ++i) {
		int k = newIndices[i];
		if(k == -1) continue;
		for(int &l : graph._originalLinks[i]) l = newIndices[l];
		std::swap(graph._originalLinks[i], graph._originalLinks[k]);
		std::swap(graph._points[i], graph._points[k]);
	}
	graph._originalLinks.resize(newSize);
	graph._points.resize(newSize);
}

// Create Graph from an SVG file.
bool GraphCreator::graphFromSvg(const Shape &shape, Graph &graph, int layerIndex) {
	UniformReal<float> dis(-Globals::_d / 20.f, Globals::_d / 20.f);
	std::mt19937 gen(Globals::_seed + layerIndex);
	Box<float> box;
	for (const glm::vec2 &point : shape._points) box.update(point);
	std::vector<glm::vec2> centroids;
	for(float x = box.x0; x <= box.x1; x += Globals::_d) {
		int j = 1;
		for (float y = box.y0; y <= box.y1; y += sqrt(0.75) * Globals::_d) {
			glm::vec2 point(x, y);
			if((++j)&1) point.x -= .5f * Globals::_d;
			point.x += dis(gen);
			point.y += dis(gen);
			if(shape.isInside(point)) centroids.push_back(point);
		}
	}
	if(centroids.size() < 3) return false;

	SegmentCVT cvt(&shape, box, layerIndex);
	graph = cvt.optimize(centroids);

	// remove 2co
	remove2coPoints(shape, graph);

	// push points inside
	for(glm::vec2 &p : graph._points) {
		if(push_inside(shape._points, p)) continue;
		for(const std::vector<glm::vec2> &in : shape._holes)
			if(push_inside(in, p)) break;
	}

	return true;
}

