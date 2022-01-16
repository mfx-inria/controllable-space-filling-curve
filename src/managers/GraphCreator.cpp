//
// Created by bedela on 18/05/2020.
//

#include "managers/GraphCreator.h"
#include <fstream>
#define NANOSVG_IMPLEMENTATION		// Expands implementation
#include "tools/nanosvg.h"
#include "LBFGS/cvt.hpp"

////////////////////////////
//                        //
//      SHAPE CREATORS    //
//                        //
////////////////////////////
/*
 *  GraphCreator's objective is to create Shape objects
 *  containing points and links. Then, each Shape will be
 *  drawn separatly
 *
 *  Each Shape only initializes _originalLinks and not _links
 */

// If the point a is on the border we pushed it inside the shape and return true
bool push_inside(const std::vector<glm::vec2> &border, glm::vec2 &a) {
	constexpr float EPS = 1e-4f;
	for(int k = 1; k < border.size(); ++k) {
		glm::vec2 v = border[k-1] - border[k];
		float l = glm::length(v);
		v /= l;
		float t = std::max(0.f, std::min(l, glm::dot(v, a-border[k])));
		glm::vec2 p = border[k] + t * v;
		if(glm::distance(a, p) > EPS) continue;
		if(t <= EPS) {
			int j = k+1;
			if(j == border.size()) j = 1;
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

struct vec2Comp {
	bool operator()(const glm::vec2 &a, const glm::vec2 &b) const {
		if(a.x < b.x) return true;
		return a.x == b.x && a.y < b.y;
	}
};

std::vector<std::vector<Shape>> mergeStroke(const std::vector<std::vector<Shape>> &fusedShapes) {
	std::vector<std::vector<Shape>> strokeZones;
	strokeZones.reserve(fusedShapes.size());
	for(const std::vector<Shape> &zones : fusedShapes) {
		// Reorder to have consecutive stroke colors
		std::vector<int> perm(zones.size());
		for(int i = 0; i < perm.size(); ++i) perm[i] = i;
		std::sort(perm.begin(), perm.end(), [&](int i, int j) { return zones[i]._printColor < zones[j]._printColor; });
		strokeZones.emplace_back();

		int Z0 = 0;

		while(Z0 < perm.size()) {
			int Z1 = Z0+1;
			while(Z1 < perm.size() && zones[perm[Z1]]._printColor == zones[perm[Z0]]._printColor) ++ Z1;

			// Create graph
			std::map<glm::vec2, int, vec2Comp> map;
			Graph graph;
			while(Z0 < Z1) {
				const Shape &zone = zones[perm[Z0++]];
				int prev = -1;
				for(const glm::vec2 &p : zone._points) {
					std::map<glm::vec2, int>::iterator it = map.lower_bound(p);
					int idx;
					if(it != map.end() && it->first == p) idx = it->second;
					else {
						idx = graph._points.size();
						graph._points.emplace_back(p);
						graph._originalLinks.emplace_back();
						map.emplace_hint(it, p, idx);
					}
					if(prev >= 0) graph._originalLinks[prev].push_back(idx);
					prev = idx;
				}
				for(const std::vector<glm::vec2> &hole : zone._holes) {
					prev = -1;
					for(const glm::vec2 &p : hole) {
						std::map<glm::vec2, int>::iterator it = map.lower_bound(p);
						int idx;
						if(it != map.end() && it->first == p) idx = it->second;
						else {
							idx = graph._points.size();
							graph._points.emplace_back(p);
							graph._originalLinks.emplace_back();
							map.emplace_hint(it, p, idx);
						}
						if(prev >= 0) graph._originalLinks[prev].push_back(idx);
						prev = idx;
					}
				}
			}

			// update links
			for(int i = 0; i < graph._points.size(); ++i) {
				int k = 0;
				while(k < graph._originalLinks[i].size()) {
					int j = graph._originalLinks[i][k];
					std::vector<int>::iterator it = std::find(graph._originalLinks[j].begin(), graph._originalLinks[j].end(), i);
					if(it == graph._originalLinks[j].end()) ++k;
					else {
						graph._originalLinks[i][k] = graph._originalLinks[i].back();
						graph._originalLinks[i].pop_back();
						*it = graph._originalLinks[j].back();
						graph._originalLinks[j].pop_back();
					}
				}
			}

			// Create pathes
			std::vector<bool> seen(graph._points.size(), false);
			std::vector<std::vector<glm::vec2>> holes;
			size_t start = strokeZones.back().size();
			for(int i = 0; i < seen.size(); ++i) if(!seen[i] && !graph._originalLinks[i].empty()) {
					std::vector<glm::vec2> path;
					int j = i;
					do {
						seen[j] = true;
						path.push_back(graph._points[j]);
						j = graph._originalLinks[j][0];
					} while(j != i);
					path.push_back(path[0]);
					float area = Globals::polygonArea(path);
					if(area > 0.f) holes.emplace_back(std::move(path));
					else {
						strokeZones.back().emplace_back();
						strokeZones.back().back()._points = std::move(path);
						strokeZones.back().back()._area = -area;
						strokeZones.back().back()._printColor = zones[perm[Z0-1]]._printColor;
					}
				}

			// Add holes
			std::sort(strokeZones.back().begin() + start, strokeZones.back().end(), [](const Shape &a, const Shape &b) { return a._area < b._area; });
			for(std::vector<glm::vec2> &hole : holes)
				for(size_t i = start; i < strokeZones.back().size(); ++i)
					if(Globals::isInPoly(strokeZones.back()[i]._points, hole[0])) {
						strokeZones.back()[i]._holes.emplace_back(std::move(hole));
						break;
					}
		}
		// sort and remove area holes
		std::sort(strokeZones.back().rbegin(), strokeZones.back().rend(), [](const Shape &a, const Shape &b) { return a._area < b._area; });
		for(Shape &zone : strokeZones.back())
			for(const std::vector<glm::vec2> &hole : zone._holes)
				zone._area -= Globals::polygonArea(hole);

	}
	return strokeZones;
}

// Create Graph from an SVG file.
void GraphCreator::graphFromSvg(const std::string &fileName,
								std::vector<Shape> &borders,
								std::vector<std::vector<Shape>> &fusedShapes,
								std::vector<std::vector<Shape>> &strokeZones,
								std::vector<Graph> &graphs,
								int layerIndex)
{
	graphs.clear();
	getShapeFromSVG(fileName, borders);
	fusedShapes = fuseShapes(borders);
	//strokeZones = mergeStroke(fusedShapes);
	strokeZones = fusedShapes;

	// Compute triangulations
	for(std::vector<Shape> &zones : fusedShapes)
		for(Shape &zone : zones)
			zone.triangulate();
	for(std::vector<Shape> &zones : strokeZones)
		for(Shape &zone : zones)
			zone.triangulate();

	Image::initVectorField(fusedShapes, borders, layerIndex);

	std::uniform_real_distribution<> dis(0.0, 1.0);
	if (Globals::_isRandom)
		Globals::_seed = Globals::_rd();
	std::mt19937  gen = std::mt19937(Globals::_seed);
	for (int i = 0; i < borders.size(); ++i) {
		float air = 1.66f; // ideal is 1.3
		// float air = 1.05f; // ideal is 1.3
		Box<double> box;
		for (const auto &point : borders[i]._points) box.update(point);
		std::vector<glm::vec2> toAdd;
		for (float x = box.x0; x <= box.x1; x += air) {
			int j = 1;
			for (float y = box.y0; y <= box.y1; y += sqrt(0.75) * air) {
				glm::vec2 point(x, y);
				if((++j)&1) point.x -= (air / 2.f);
				point.x += static_cast<float>(dis(gen)) * air / 20.f;
				point.y += static_cast<float>(dis(gen)) * air / 20.f;
				if(borders[i].isInside(point)) toAdd.push_back(point);
			}
		}
		if(toAdd.size() < 3) {
			std::swap(borders[i], borders.back());
			std::swap(fusedShapes[i], fusedShapes.back());
			std::swap(strokeZones[i], strokeZones.back());
			borders.pop_back();
			fusedShapes.pop_back();
			strokeZones.pop_back();
			-- i;
			continue;
		}

		SegmentCVT cvt(&borders[i], box, layerIndex);
		graphs.push_back(cvt.optimize(toAdd));

		// remove 2co
		remove2coPoints(borders[i], graphs.back());

		// push points inside
		for(glm::vec2 &p : graphs.back()._points) {
			if(push_inside(borders[i]._points, p)) continue;
			for(const std::vector<glm::vec2> &in : borders[i]._holes)
				if(push_inside(in, p)) break;
		}
	}
}

////////////////////////////
//                        //
//         HELPERS        //
//                        //
////////////////////////////

bool isNear(const std::vector<glm::vec2> &cycle, const glm::vec2 &p, float eps2=1e-10f) {
	for(int i = 1; i < cycle.size(); ++i) {
		const glm::vec2 v = cycle[i-1] - cycle[i];
		const float len2 = glm::dot(v, v);
		const glm::vec2 diff = cycle[i] + v * std::clamp(glm::dot(v, p - cycle[i])/len2, 0.f, 1.f) - p;
		if(glm::dot(diff, diff) < eps2) return true;
	}
	return false;
}

bool isNear(const Shape &shape, const glm::vec2 &p, float eps2=1e-10f) {
	if(isNear(shape._points, p, eps2)) return true;
	for(const std::vector<glm::vec2> &hole : shape._holes)
		if(isNear(hole, p, eps2)) return true;
	return false;
}

std::vector<std::vector<Shape>> GraphCreator::fuseShapes(std::vector<Shape> &borders) {
	// Make black shapes first
	int B = 0;
	for(int i = 0; i < borders.size(); ++i)
		if(borders[i]._objcetive == NOTHING)
			std::swap(borders[B++], borders[i]);
	Box<float> box;
	for(const Shape &s : borders) for(const glm::vec2 &p : s._points) box.update(p);
	const float eps = 1e-5f * box.diag();

	// Fuse black shapes with inner shapes
	std::vector<std::vector<Shape>> fusedShapes(B);
	for(int i = 0; i < B; ++i) {
		int j = B;
		while(j < borders.size()) {
			if(borders[i].isInside(borders[j]._points[0]) || isNear(borders[i], borders[j]._points[0], eps*eps)) {
				fusedShapes[i].emplace_back(std::move(borders[j]));
				if(j+1 != borders.size()) borders[j] = std::move(borders.back());
				borders.pop_back();
			} else ++j;
		}
	}
	fusedShapes.resize(borders.size());
	for(int i = 0; i < (int) borders.size(); ++i)
		if(fusedShapes[i].empty())
			fusedShapes[i].emplace_back(borders[i]);
	for(int i = B; i < (int) borders.size(); ++i) borders[i]._objcetive = NOTHING;

	// sort borders
	std::vector<uint> perm(borders.size());
	for(uint i = 0; i < perm.size(); ++i) perm[i] = i;
	std::sort(perm.begin(), perm.end(), [&](uint i, uint j) { return borders[i]._area > borders[j]._area; });
	for(uint i = 0; i < perm.size(); ++i) {
		uint j = i;
		while(perm[j] != i) {
			uint k = perm[j];
			std::swap(borders[j], borders[k]);
			std::swap(fusedShapes[j], fusedShapes[k]);
			perm[j] = j;
			j = k;
		}
		perm[j] = j;
	}

	// sort zones
	for(std::vector<Shape> &zones : fusedShapes)
		std::sort(zones.begin(), zones.end(), [](const Shape &a, const Shape &b) { return a._area > b._area; });

	// remove holes area
	for(std::vector<Shape> &zones : fusedShapes)
		for(Shape &zone : zones)
			for(const std::vector<glm::vec2> &hole : zone._holes)
				zone._area -= Globals::polygonArea(hole);
	for(Shape &b : borders)
		for(const std::vector<glm::vec2> &hole : b._holes)
			b._area -= Globals::polygonArea(hole);

	return fusedShapes;
}

void GraphCreator::remove2coPoints(Shape &border, Graph &graph) {
	// Stack containing points to remove
	std::vector<int> stack;
	for (int i = 0; i < graph._points.size(); ++i)
		if(graph._originalLinks[i].size() < 3 && !isNear(border, graph._points[i]))
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
					if(link2.size() < 3 && !isNear(border, graph._points[link[0]])) stack.push_back(link[0]);
				}
				std::swap(link[0], link[1]);
			}
		} else if(link.size() == 1) {
			std::vector<int> &link2 = graph._originalLinks[link[0]];
			std::vector<int>::iterator it = std::find(link2.begin(), link2.end(), i);
			*it = link2.back();
			link2.pop_back();
			if(link2.size() < 3 && !isNear(border, graph._points[link[0]])) stack.push_back(link[0]);
		}
		link.clear();
	}

	// Create new indices for remaining points
	std::vector<int> newIndices(graph._points.size(), -1);
	int newSize = 0;
	for(int i = 0; i < newIndices.size(); ++i)
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
	for(int i = 0; i < newIndices.size(); ++i) {
		int k = newIndices[i];
		if(k == -1) continue;
		for(int &l : graph._originalLinks[i]) l = newIndices[l];
		std::swap(graph._originalLinks[i], graph._originalLinks[k]);
		std::swap(graph._points[i], graph._points[k]);
	}
	graph._originalLinks.resize(newSize);
	graph._points.resize(newSize);
}

// Return the index of the nearest objective from color
OBJECTIVE getNearestObjective(unsigned int color) {
	glm::vec3 rgbCol((color & 0xff) / 255.f, ((color >> 8) & 0xff) / 255.f, ((color >> 16) & 0xff) / 255.f);
	float minDiff = std::numeric_limits<float>::max();
	int index;
	for(int i = 0; i < (int) COLORS.size(); i++) {
		float diff = glm::distance(rgbCol, COLORS[i]);
		if(diff < minDiff) {
			minDiff = diff;
			index = i;
		}
	}
	return (OBJECTIVE) index;
}

void GraphCreator::getShapeFromSVG(const std::string &fileName, std::vector<Shape> &shapes) {
	shapes.clear();
	std::vector<Shape> holes;
	struct NSVGimage* image;
	image = nsvgParseFromFile(fileName.c_str(), "mm", 96);
	if(image == nullptr) throw std::runtime_error("can't parse input svg file: " + fileName);
	for(NSVGshape *shape = image->shapes; shape != NULL; shape = shape->next) {
		std::vector<Shape> shapesAdd, holesAdd;
		float maxArea = 0.f;
		for(NSVGpath *path = shape->paths; path != NULL; path = path->next) {
			Shape polygon;
			polygon._points.reserve(path->npts/3+1);
			for (int i = 0; i < path->npts; i += 3) {
				float* p = &path->pts[i*2];
				polygon._points.emplace_back(p[0], p[1]);
			}
			polygon._area = Globals::polygonArea(polygon._points);
			if(std::abs(polygon._area) > std::abs(maxArea)) maxArea = polygon._area;

			if(shape->fill.type) polygon._objcetive = getNearestObjective(shape->fill.color);
			if(shape->stroke.type) polygon._printColor = shape->stroke.color;
			if(polygon._area > 0.f) holesAdd.emplace_back(std::move(polygon));
			else {
				polygon._area = -polygon._area;
				shapesAdd.emplace_back(std::move(polygon));
			}
		}

		if(maxArea > 0.f) { // swap shapes and hole if needed
			std::swap(holesAdd, shapesAdd);
			for(Shape &shape : shapesAdd) std::reverse(shape._points.begin(), shape._points.end());
			for(Shape &hole : holesAdd) std::reverse(hole._points.begin(), hole._points.end());
		}

		// order shapes by area, from small to big
		sort(shapesAdd.begin(), shapesAdd.end(), [&](const Shape &a, const Shape &b) { return a._area < b._area; });

		// put the hole inside the shapes
		for(Shape &hole : holesAdd)
			for(Shape &shapeTmp: shapesAdd)
				if(Globals::isInPoly(shapeTmp._points, hole._points[0])) {
					shapeTmp._holes.emplace_back(std::move(hole._points));
					break;
				}

		shapes.insert(shapes.end(), std::make_move_iterator(shapesAdd.begin()), std::make_move_iterator(shapesAdd.end()));
	}
	sort(shapes.begin(), shapes.end(), [&](const Shape &a, const Shape &b) { return a._area < b._area; });
	nsvgDelete(image);
}