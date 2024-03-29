#include "graphics/DirectionField.h"
#include "graphics/Window.h"

#define GL_GLEXT_PROTOTYPES
#include <GL/freeglut.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <queue>

struct Segment {
	glm::dvec2 _a, _b, _v;
	double _len;

	Segment(const glm::dvec2 &a, const glm::dvec2 &b): _a(a), _b(b), _v(b-a) {
		_len = glm::length(_v);
		_v /= _len;
	}
	
	inline double dist2(const glm::dvec2 &p) const {
		const glm::dvec2 q = p - _a;
		double x = _v.x * q.x + _v.y * q.y;
		const double y = _v.y * q.x - _v.x * q.y;
		if(x < 0) return x*x + y*y;
		if(x > _len) {
			x -= _len;
			return x*x + y*y;
		}
		return y*y;
	}
	
	inline double normal_angle(const glm::dvec2 &p) const {
		const double t = std::clamp(_v.x * (p.x - _a.x) + _v.y * (p.y - _a.y), 0., _len);
		return std::atan2(p.y - _a.y - t * _v.y, p.x - _a.x - t * _v.x);
	}
};

void DirectionField::init(int layerNb) {
	DirectionField::_imgWidth = std::sqrt((_pixelsWanted * Globals::_SVGSize.x) / Globals::_SVGSize.y);
	DirectionField::_imgHeight = _pixelsWanted / _imgWidth;
	_vectorField.resize(layerNb);
	_vectorFieldSum.resize(layerNb);
}

// get vector from the pixel of the vectorfield
// image corresponding to the position of point
glm::dvec2 DirectionField::getVecAtPos(const glm::dvec2 &point, int layerIndex) {
	double x0f = point.x / Globals::_SVGSize.x * _imgWidth - .5;
	int x0 = x0f < 0. ? x0f-1 : x0f;
	double y0f = point.y / Globals::_SVGSize.y * _imgHeight - .5;
	int y0 = y0f < 0. ? y0f-1 : y0f;
	glm::dvec2 V(0., 0.);
	double xc[2] = {x0+1.-x0f, x0f-x0}, yc[2] = {y0+1.-y0f, y0f-y0};
	for(int x : {x0, x0+1}) if(x >= 0 && x < _imgWidth)
			for(int y : {y0, y0+1}) if(y >= 0 && y < _imgHeight)
					V += xc[x-x0] * yc[y-y0] * _vectorField[layerIndex][x + y * _imgWidth];
	return V;
}

glm::dvec2 DirectionField::getVecUnder(const glm::dvec2 &a, const glm::dvec2 &b, int layerIndex) {
	if(std::abs(a.x - b.x) < 1e-7f) return glm::dvec2(0., 0.);
	double x0f = a.x / Globals::_SVGSize.x * _imgWidth;
	double x1f = b.x / Globals::_SVGSize.x * _imgWidth;
	double y0f = a.y / Globals::_SVGSize.y * _imgHeight;
	double y1f = b.y / Globals::_SVGSize.y * _imgHeight;
	bool swapped = false;
	if(x0f > x1f) {
		std::swap(x0f, x1f);
		std::swap(y0f, y1f);
		swapped = true;
	}
	if(x1f <= 0. || x0f >= _imgWidth) return glm::dvec2(0., 0.);
	double rat = (y1f - y0f) / (x1f - x0f);
	if(x0f < 0.) {
		y0f -= rat * x0f;
		x0f = 0.;
	}
	if(x1f > _imgWidth) {
		y0f += (_imgWidth - x1f) * rat;
		x0f = _imgWidth;
	}

	glm::dvec2 V(0., 0.);
	double xx = x0f, yy = y0f;
	int count = 0;
	while(abs(x0f - x1f) > 1e-7) {
		count++;
		if (count == 1000)
			return V;
		double xf = std::floor(x0f+1.);
		double yf = yy + rat * (xf - xx);
		if(rat < 0) {
			double f = std::ceil(y0f) - 1.;
			if(yf < f) {
				yf = f;
				xf = xx + (yf - yy) / rat;
			}
		} else {
			double f = std::floor(y0f) + 1.;
			if(yf > f) {
				yf = f;
				xf = xx + (yf - yy) / rat;
			}
		}
		if(xf > x1f) {
			xf = x1f;
			yf = y1f;
		}
		double my = .5f * (y0f + yf);
		if(my > 0.) {
			int x = .5f * (x0f + xf);
			double dx = xf - x0f;
			if(my < _imgHeight) {
				int y = my;
				V += _vectorFieldSum[layerIndex][x + y * _imgWidth] * dx;
				V -= _vectorField[layerIndex][x + y * _imgWidth] * (y+1.-my) * dx;
			} else V += _vectorFieldSum[layerIndex][x + (_imgHeight-1) * _imgWidth] * dx;
		}
		x0f = xf;
		y0f = yf;
	}
	return swapped ? -V : V;
}

// create vector field image based on image borders
// the vector field is parallel to the borders
void DirectionField::initVectorField(const std::vector<std::vector<Shape>> &zones, const std::vector<Shape> &borders, int layerIndex) {
	std::vector<Segment> segments;
	std::vector<int> nearest;

	// 1. creates segments
	const auto addShapeSegments = [&segments](const Shape &shape)->void {
		for(int i = 1; i < (int) shape._points.size(); ++i)
			segments.emplace_back(shape._points[i-1], shape._points[i]);
		for(const std::vector<glm::dvec2> &hole : shape._holes)
			for(int i = 1; i < (int) hole.size(); ++i)
				segments.emplace_back(hole[i-1], hole[i]);
	};
	bool anyVector = false;
	for(const std::vector<Shape> &ss : zones)
		for(const Shape &shape : ss) if(IS_VECTOR(shape._objcetive)) {
				anyVector = true;
				if(ADD_BORDER(shape._objcetive))
					addShapeSegments(shape);
			}
	if(!anyVector) { // if there is no alignment objective the vector field is null
		_vectorField[layerIndex].assign(_imgWidth * _imgHeight, glm::dvec2(0., 0.));
		_vectorFieldSum[layerIndex].assign(_vectorField[layerIndex].size(), glm::dvec2(0., 0.));
		return;
	}
	for(const Shape &shape : borders) addShapeSegments(shape);
	nearest.assign(_imgWidth * _imgHeight, -1);

	//////////////
	// 2. Dijkstra
	//////////////

	// The queue
	typedef std::pair<double, int> QEl;
	std::priority_queue<QEl, std::vector<QEl>, std::greater<QEl>> Q;
	// vector of distances
	std::vector<double> D(_imgWidth * _imgHeight, std::numeric_limits<double>::max());
	const int imS[2] { _imgWidth, _imgHeight };
	const double rat[2] { Globals::_SVGSize.x / _imgWidth, Globals::_SVGSize.y / _imgHeight };
	const double irat[2] { _imgWidth / Globals::_SVGSize.x, _imgHeight / Globals::_SVGSize.y };

	// init Q
	for(int i = 0; i < (int) segments.size(); ++i) {
		const Segment &seg = segments[i];
		const int h = std::abs(seg._v.x) < std::abs(seg._v.y);
		const double slope = seg._v[h^1] / seg._v[h];
		int x0 = seg._a[h] * irat[h], x1 = seg._b[h] * irat[h];
		if(x0 > x1) std::swap(x0, x1);
		x0 = std::max(0, x0-1);
		x1 = std::min(imS[h]-1, x1+1);
		for(int x = x0; x <= x1; ++x) {
			glm::dvec2 p;
			p[h] = (x + 0.5f) * rat[h];
			const double y = seg._a[h^1] + (p[h] - seg._a[h]) * slope;
			const int y0 = std::max(0, int(y * irat[h^1] - 1.));
			const int y1 = std::min(imS[h^1]-1, y0 + 2);
			for(int y = y0; y <= y1; ++y) {
				p[h^1] = (y + 0.5f) * rat[h^1];
				const double d = seg.dist2(p);
				const int pix = h ? y + x * _imgWidth : x + y * _imgWidth;
				if(d < D[pix]) {
					D[pix] = d;
					nearest[pix] = i;
					Q.emplace(d, pix);
				}
			}
		}
	}

	// propagation
	while(!Q.empty()) {
		auto [d, pix] = Q.top(); Q.pop();
		if(d > D[pix]) continue;
		const int seg_ind = nearest[pix];
		const int x = pix % _imgWidth;
		const int y = pix / _imgWidth;
		const int x0 = std::max(0, x-1);
		const int x1 = std::min(_imgWidth, x+2);
		const int y0 = std::max(0, y-1);
		const int y1 = std::min(_imgHeight, y+2);
		for(int a = x0; a < x1; ++a) {
			double xf = (a + 0.5f) * rat[0];
			for(int b = y0; b < y1; ++b) {
				int pix2 = a + b * _imgWidth;
				if(nearest[pix2] == seg_ind) continue;
				const glm::dvec2 p(xf, (b + 0.5f) * rat[1]);
				const double d2 = segments[seg_ind].dist2(p);
				if(d2 < D[pix2]) {
					D[pix2] = d2;
					nearest[pix2] = seg_ind;
					Q.emplace(d2, pix2);
				}
			}
		}
	}

	/////////
	// Rastzerize zones to init vector field
	/////////

	_vectorField[layerIndex].assign(nearest.size(), glm::dvec2(0., 0.));

	const auto raster = [&](std::array<glm::dvec2, 3> &&v, const int c=-1) {
		std::sort(v.begin(), v.end(), [&](const glm::dvec2 &a, const glm::dvec2 &b) { return a.y < b.y; });
		for(glm::dvec2 &a : v) for(int d : {0, 1}) a[d] *= irat[d];
		int y = v[0].y;
		double yf = y + .5f;
		if(yf < v[0].y) ++y, ++yf;
		const auto yloop = [&](double my, double &x0, double &x1, double &slope1, double &slope2) {
			for(; yf < my; ++y, ++yf, x0 += slope1, x1 += slope2) {
				const int x = x0;
				double xf = x + .5f;
				int pix = x + y * _imgWidth;
				if(xf < x0) ++pix, ++xf;
				for(; xf < x1; ++pix, ++xf) {
					if(nearest[pix] == -1) THROW_ERROR("Nearest not filled!");
					if(c == -1) {
						_vectorField[layerIndex][pix] = glm::dvec2(0., 0.);
						continue;
					}
					const glm::dvec2 p(xf * rat[0], yf * rat[1]);
					double angle = segments[nearest[pix]].normal_angle(p);
					if(c) angle += M_PI_2;
					_vectorField[layerIndex][pix].x = std::cos(angle);
					_vectorField[layerIndex][pix].y = std::sin(angle);
				}
			}
		};
		if(yf < v[1].y) {
			double slope1 = (v[1].x - v[0].x) / (v[1].y - v[0].y);
			double slope2 = (v[2].x - v[0].x) / (v[2].y - v[0].y);
			if(slope1 > slope2) std::swap(slope1, slope2);
			const double dy = yf - v[0].y;
			double x0 = v[0].x + dy * slope1;
			double x1 = v[0].x + dy * slope2;
			yloop(v[1].y, x0, x1, slope1, slope2);
		}
		if(yf < v[2].y) {
			double slope1 = (v[2].x - v[1].x) / (v[2].y - v[1].y);
			double slope2 = (v[2].x - v[0].x) / (v[2].y - v[0].y);
			double x0 = v[1].x + (yf - v[1].y) * slope1;
			double x1 = v[0].x + (yf - v[0].y) * slope2;
			if(x0 > x1) {
				std::swap(slope1, slope2);
				std::swap(x0, x1);
			}
			yloop(v[2].y, x0, x1, slope1, slope2);
		}
	};

	for(const std::vector<Shape> &zones : zones) {
		for(const Shape &zone : zones) if(IS_VECTOR(zone._objcetive)) {
			const int c = !IS_ORTHO(zone._objcetive);
			for(uint i = 0; i < zone._triangles[0].size(); i += 3)
				raster({zone._points[zone._triangles[0][i]], zone._points[zone._triangles[0][i+1]], zone._points[zone._triangles[0][i+2]]}, c);
			for(uint i = 0; i < zone._holes.size(); ++i)
				for(uint j = 0; j < zone._triangles[i+1].size(); j += 3)
					raster({zone._holes[i][zone._triangles[i+1][j]], zone._holes[i][zone._triangles[i+1][j+1]], zone._holes[i][zone._triangles[i+1][j+2]]});
		}
	}

	// vector field sum
	_vectorFieldSum[layerIndex] = _vectorField[layerIndex];
	for(int i = _imgWidth; i < (int) nearest.size(); ++i)
		_vectorFieldSum[layerIndex][i] += _vectorFieldSum[layerIndex][i-_imgWidth];
	for(int i = 1; i < (int) nearest.size(); ++i) if(i % _imgWidth != 0)
		_vectorFieldSum[layerIndex][i] += _vectorFieldSum[layerIndex][i-1];

	//////////
	// Smoothing
	//////////

	for(int i = 0; i < (int) nearest.size(); ++i) {
		int x = i % _imgWidth, y = i / _imgWidth;
		if(_vectorField[layerIndex][i] == glm::dvec2(0., 0.)) continue;
		const int K = 40;
		glm::dvec2 s(0., 0.);
		for(int k = K; k >= 5; k >>= 1) {
			const int x0 = std::max(0, x-k);
			const int x1 = std::min(_imgWidth-1, x+k);
			const int y0 = std::max(0, y-k);
			const int y1 = std::min(_imgHeight-1, y+k);
			s += _vectorFieldSum[layerIndex][x1 + y1*_imgWidth];
			if(x0 > 0) s -= _vectorFieldSum[layerIndex][x0-1 + y1*_imgWidth];
			if(y0 > 0) s -= _vectorFieldSum[layerIndex][x1 + (y0-1)*_imgWidth];
			if(x0 > 0 && y0 > 0) s += _vectorFieldSum[layerIndex][x0-1 + (y0-1)*_imgWidth];
		}
		double angle = 2. * std::atan2(s.y, s.x);
		_vectorField[layerIndex][i].x = std::cos(angle);
		_vectorField[layerIndex][i].y = std::sin(angle);
	}

	// vector field sum
	_vectorFieldSum[layerIndex] = _vectorField[layerIndex];
	for(int i = _imgWidth; i < (int) nearest.size(); ++i)
		_vectorFieldSum[layerIndex][i] += _vectorFieldSum[layerIndex][i-_imgWidth];
}
