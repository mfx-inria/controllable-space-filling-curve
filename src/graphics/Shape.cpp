#include "graphics/Shape.h"
#include "graphics/DirectionField.h"
#define NANOSVG_IMPLEMENTATION
#include "tools/nanosvg.h"

#include <map>
#include <algorithm>

// Retrun true if p is at a distance < sqrt(eps2) from cycle 
inline bool isNearAux(const std::vector<glm::vec2> &cycle, const glm::vec2 &p, float eps2=1e-10f) {
	for(int i = 1; i < (int) cycle.size(); ++i) {
		const glm::vec2 v = cycle[i-1] - cycle[i];
		if(glm::distance2(cycle[i] + v * std::clamp(glm::dot(v, p - cycle[i])/glm::dot(v, v), 0.f, 1.f), p) < eps2) return true;
	}
	return false;
}

// Retrun true if p is at a distance < sqrt(eps2) from shape border 
bool Shape::isNear(const glm::vec2 &p, float eps2) const {
	if(isNearAux(_points, p, eps2)) return true;
	for(const std::vector<glm::vec2> &hole : _holes)
		if(isNearAux(hole, p, eps2)) return true;
	return false;
}

// is p inside of _points
bool Shape::isInside(const glm::vec2 &p) const {
	if(!Globals::isInPoly(_points, p)) return false;
	for(const std::vector<glm::vec2> &hole : _holes)
		if(Globals::isInPoly(hole, p))
			return false;
	return true;
}

bool Shape::intersect(const glm::vec2 &a, const glm::vec2 &b) const {
	for(int i = 1; i < (int) _points.size(); ++i)
		if(Globals::intersect(a, _points[i-1], b, _points[i]))
			return true;
	for(const std::vector<glm::vec2> &hole : _holes)
		for(int i = 1; i < (int) hole.size(); ++i)
			if(Globals::intersect(a, hole[i-1], b, hole[i]))
				return true;
	return false;
}

void Shape::getInter(const glm::vec2 &a, const glm::vec2 &b, std::vector<float> &ts) const {
	float t;
	for(int i = 1; i < (int) _points.size(); ++i)
		if(Globals::intersect(a, _points[i-1], b, _points[i], t))
			ts.push_back(t);
	for(const std::vector<glm::vec2> &hole : _holes)
		for(int i = 1; i < (int) hole.size(); ++i)
			if(Globals::intersect(a, hole[i-1], b, hole[i], t))
				ts.push_back(t);
}

//=========================//
// READING SHAPES FROM SVG //
//=========================//

// Return the diagonal of the smallest axis-aligned rectangle containing shapes
inline float getDiag(const std::vector<Shape> &shapes) {
	Box<float> box;
	for(const Shape &s : shapes) for(const glm::vec2 &p : s._points) box.update(p);
	return box.diag();
}

void getShapeFromSVG(const std::string &fileName, std::vector<Shape> &shapes);
std::vector<std::vector<Shape>> regroupObjZones(std::vector<Shape> &shapes, float eps);
std::vector<std::vector<Shape>> mergeColorZones(const std::vector<std::vector<Shape>> &shapes, float eps);

void Shape::read(const std::string &fileName,
					std::vector<Shape> &shapes,
					std::vector<std::vector<Shape>> &objZones,
					std::vector<std::vector<Shape>> &colorZones,
					int layerIndex)
{
	getShapeFromSVG(fileName, shapes);
	const float eps = 1e-5f * getDiag(shapes);
	objZones = regroupObjZones(shapes, eps);
	colorZones = mergeColorZones(objZones, eps);

	// Compute triangulations
	for(auto shapes : {&objZones, &colorZones})
		for(std::vector<Shape> &zones : *shapes)
			for(Shape &zone : zones)
				zone.triangulate();

	DirectionField::initVectorField(objZones, shapes, layerIndex);
}

//=====================//
// AUXILIARY FUNCTIONS //
//=====================//

// Return the index of the nearest objective from color
inline OBJECTIVE getNearestObjective(unsigned int color) {
	glm::vec3 rgbCol((color & 0xff) / 255.f, ((color >> 8) & 0xff) / 255.f, ((color >> 16) & 0xff) / 255.f);
	float minDiff = std::numeric_limits<float>::max();
	int index = -1;
	for(int i = 0; i < (int) COLORS.size(); i++) {
		float diff = glm::distance(rgbCol, COLORS[i]);
		if(diff < minDiff) {
			minDiff = diff;
			index = i;
		}
	}
	return (OBJECTIVE) index;
}

// Construct shapes from SVG
void getShapeFromSVG(const std::string &fileName, std::vector<Shape> &shapes) {
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

// Find objective zones and regroup them inside their shapes
std::vector<std::vector<Shape>> regroupObjZones(std::vector<Shape> &shapes, float eps) {
	// Make black shapes first
	int B = 0;
	for(int i = 0; i < (int) shapes.size(); ++i)
		if(shapes[i]._objcetive == NOTHING)
			std::swap(shapes[B++], shapes[i]);

	// Fuse black shapes with inner shapes
	std::vector<std::vector<Shape>> objZones(B);
	for(int i = 0; i < B; ++i) {
		int j = B;
		while(j < (int) shapes.size()) {
			if(shapes[i].isInside(shapes[j]._points[0]) || shapes[i].isNear(shapes[j]._points[0], eps*eps)) {
				objZones[i].emplace_back(std::move(shapes[j]));
				if(j+1 != (int) shapes.size()) shapes[j] = std::move(shapes.back());
				shapes.pop_back();
			} else ++j;
		}
	}
	objZones.resize(shapes.size());
	for(int i = 0; i < (int) shapes.size(); ++i)
		if(objZones[i].empty())
			objZones[i].emplace_back(shapes[i]);
	for(int i = B; i < (int) shapes.size(); ++i) shapes[i]._objcetive = NOTHING;

	// sort borders
	std::vector<uint> perm(shapes.size());
	for(uint i = 0; i < perm.size(); ++i) perm[i] = i;
	std::sort(perm.begin(), perm.end(), [&](uint i, uint j) { return shapes[i]._area > shapes[j]._area; });
	for(uint i = 0; i < perm.size(); ++i) {
		uint j = i;
		while(perm[j] != i) {
			uint k = perm[j];
			std::swap(shapes[j], shapes[k]);
			std::swap(objZones[j], objZones[k]);
			perm[j] = j;
			j = k;
		}
		perm[j] = j;
	}

	// sort zones
	for(std::vector<Shape> &zones : objZones)
		std::sort(zones.begin(), zones.end(), [](const Shape &a, const Shape &b) { return a._area > b._area; });

	// remove holes area
	for(std::vector<Shape> &zones : objZones)
		for(Shape &zone : zones)
			for(const std::vector<glm::vec2> &hole : zone._holes)
				zone._area -= Globals::polygonArea(hole);
	for(Shape &b : shapes)
		for(const std::vector<glm::vec2> &hole : b._holes)
			b._area -= Globals::polygonArea(hole);

	return objZones;
}

// Merge adjacent objectives zones with same print color together
std::vector<std::vector<Shape>> mergeColorZones(const std::vector<std::vector<Shape>> &shapes, float eps) {
	std::vector<std::vector<Shape>> colorZones;
	colorZones.reserve(shapes.size());
	const float eps2 = eps*eps;
	for(const std::vector<Shape> &zones : shapes) {
		// Reorder to have consecutive stroke colors
		std::vector<int> perm(zones.size());
		for(int i = 0; i < (int) perm.size(); ++i) perm[i] = i;
		std::sort(perm.begin(), perm.end(), [&](int i, int j) { return zones[i]._printColor < zones[j]._printColor; });
		colorZones.emplace_back();

		int Z0 = 0;

		while(Z0 < (int) perm.size()) {
			int Z1 = Z0+1;
			while(Z1 < (int) perm.size() && zones[perm[Z1]]._printColor == zones[perm[Z0]]._printColor) ++ Z1;

			// Create graph
			const auto vec2Comp = [](const glm::vec2 &a, const glm::vec2 &b) { return a.x < b.x || (a.x == b.x && a.y < b.y); };
			std::map<glm::vec2, int, decltype(vec2Comp)> map(vec2Comp);
			std::vector<glm::vec2> points;
			std::vector<std::vector<int>> links;
			while(Z0 < Z1) {
				const Shape &zone = zones[perm[Z0++]];
				const auto addCycle = [&](const std::vector<glm::vec2> &cycle) {
					int prev = -1;
					for(const glm::vec2 &p : cycle) {
						std::map<glm::vec2, int>::iterator it = map.lower_bound(p);
						int idx=-1;
						float xmin = p.x-eps, xmax = p.x+eps;
						for(auto it2 = it; it2 != map.end() && it2->first.x <= xmax; ++it2)
							if(glm::distance2(it2->first, p) < eps2) { idx = it->second; break; }
						if(idx == -1) for(auto it2 = it; it2 != map.begin() && (--it2)->first.x >= xmin;)
							if(glm::distance2(it2->first, p) < eps2) { idx = it->second; break; }
						if(idx == -1) {
							idx = points.size();
							points.emplace_back(p);
							links.emplace_back();
							map.emplace_hint(it, p, idx);
						}
						if(prev >= 0) links[prev].push_back(idx);
						prev = idx;
					}
				};
				addCycle(zone._points);
				for(const std::vector<glm::vec2> &hole : zone._holes) addCycle(hole);
			}

			// update links
			for(int i = 0; i < (int) points.size(); ++i) {
				int k = 0;
				while(k < (int) links[i].size()) {
					int j = links[i][k];
					std::vector<int>::iterator it = std::find(links[j].begin(), links[j].end(), i);
					if(it == links[j].end()) ++k;
					else {
						links[i][k] = links[i].back();
						links[i].pop_back();
						*it = links[j].back();
						links[j].pop_back();
					}
				}
			}

			// Create pathes
			std::vector<bool> toSee(points.size(), true);
			std::vector<std::vector<glm::vec2>> holes;
			size_t start = colorZones.back().size();
			for(int i = 0; i < (int) toSee.size(); ++i) if(toSee[i] && !links[i].empty()) {
					std::vector<glm::vec2> path;
					int j = i;
					do {
						toSee[j] = false;
						path.push_back(points[j]);
						if(links[j].size() != 1) THROW_ERROR("Failed to merge color zones");
						j = links[j][0];
					} while(j != i);
					path.push_back(path[0]);
					float area = Globals::polygonArea(path);
					if(area > 0.f) holes.emplace_back(std::move(path));
					else {
						colorZones.back().emplace_back();
						colorZones.back().back()._points = std::move(path);
						colorZones.back().back()._area = -area;
						colorZones.back().back()._printColor = zones[perm[Z0-1]]._printColor;
					}
				}

			// Add holes
			std::sort(colorZones.back().begin() + start, colorZones.back().end(), [](const Shape &a, const Shape &b) { return a._area < b._area; });
			for(std::vector<glm::vec2> &hole : holes)
				for(size_t i = start; i < colorZones.back().size(); ++i)
					if(Globals::isInPoly(colorZones.back()[i]._points, hole[0])) {
						colorZones.back()[i]._holes.emplace_back(std::move(hole));
						break;
					}
		}
		// sort and remove area holes
		std::sort(colorZones.back().rbegin(), colorZones.back().rend(), [](const Shape &a, const Shape &b) { return a._area < b._area; });
		for(Shape &zone : colorZones.back())
			for(const std::vector<glm::vec2> &hole : zone._holes)
				zone._area -= Globals::polygonArea(hole);

	}
	return colorZones;
}