//
// Created by adrien_bedel on 18/11/19.
//

#include "graphics/Globals.h"

#define GL_GLEXT_PROTOTYPES
#include <GL/glut.h>

#include "graphics/Window.h"

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
	// Update Image sizes
	Image::_imgWidth = std::sqrt((Image::_pixelsWanted * Globals::_SVGSize.x) / Globals::_SVGSize.y);
	Image::_imgHeight = Image::_pixelsWanted / Image::_imgWidth;
	Image::resize(layerNb);
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
		if(points[i-1].y == points[i].y) continue;
		if(points[i].y == point.y) {
			if(points[i].x > point.x && points[i].y < points[i-1].y) inside = !inside;
		} else if(points[i-1].y == point.y) {
			if(points[i-1].x > point.x && points[i].y > points[i-1].y) inside = !inside;
		} else if((points[i].y > point.y) != (points[i-1].y > point.y)) {
			float x = points[i].x + (points[i-1].x - points[i].x) * (point.y - points[i].y) / (points[i-1].y - points[i].y);
			if(x > point.x) inside = !inside;
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
float Globals::getDist(const glm::vec3 &a, const glm::vec3 &b)
{
	return (sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)));
}

//////////////////
//
//      IMAGE
//
//////////////////

void Image::resize(int layerNb) {
	_vectorField.resize(layerNb);
	_vectorFieldSum.resize(layerNb);
	_tmpImages.assign(layerNb, nullptr);
	_tmpZones.resize(layerNb);
}

// get vector from the pixel of the vectorfield
// image corresponding to the position of point
glm::vec2 Image::getVecAtPos(const glm::vec2 &point, int layerIndex) {
	float x0f = point.x / Globals::_SVGSize.x * _imgWidth - .5f;
	int x0 = x0f < 0.f ? x0f-1 : x0f;
	float y0f = point.y / Globals::_SVGSize.y * _imgHeight - .5f;
	int y0 = y0f < 0.f ? y0f-1 : y0f;
	glm::vec2 V(0.f, 0.f);
	float xc[2] = {x0+1.f-x0f, x0f-x0}, yc[2] = {y0+1.f-y0f, y0f-y0};
	for(int x : {x0, x0+1}) if(x >= 0 && x < _imgWidth)
			for(int y : {y0, y0+1}) if(y >= 0 && y < _imgHeight)
					V += xc[x-x0] * yc[y-y0] * _vectorField[layerIndex][x + y * _imgWidth];
	return V;
}

glm::vec2 Image::getVecUnder(const glm::vec2 &a, const glm::vec2 &b, int layerIndex) {
	if(std::abs(a.x - b.x) < 1e-7f) return glm::vec2(0.f, 0.f);
	float x0f = a.x / Globals::_SVGSize.x * _imgWidth;
	float x1f = b.x / Globals::_SVGSize.x * _imgWidth;
	float y0f = a.y / Globals::_SVGSize.y * _imgHeight;
	float y1f = b.y / Globals::_SVGSize.y * _imgHeight;
	bool swapped = false;
	if(x0f > x1f) {
		std::swap(x0f, x1f);
		std::swap(y0f, y1f);
		swapped = true;
	}
	if(x1f <= 0.f || x0f >= _imgWidth) return glm::vec2(0., 0.);
	float rat = (y1f - y0f) / (x1f - x0f);
	if(x0f < 0.f) {
		y0f -= rat * x0f;
		x0f = 0.f;
	}
	if(x1f > _imgWidth) {
		y0f += (_imgWidth - x1f) * rat;
		x0f = _imgWidth;
	}

	glm::vec2 V(0.f, 0.f);
	float xx = x0f, yy = y0f;
	int count = 0;
	while(abs(x0f - x1f) > 1e-7) {
		count++;
		if (count == 1000)
			return V;
		float xf = std::floor(x0f+1.f);
		float yf = yy + rat * (xf - xx);
		if(rat < 0) {
			float f = std::ceil(y0f) - 1.f;
			if(yf < f) {
				yf = f;
				xf = xx + (yf - yy) / rat;
			}
		} else {
			float f = std::floor(y0f) + 1.f;
			if(yf > f) {
				yf = f;
				xf = xx + (yf - yy) / rat;
			}
		}
		if(xf > x1f) {
			xf = x1f;
			yf = y1f;
		}
		float my = .5f * (y0f + yf);
		if(my > 0.f) {
			int x = .5f * (x0f + xf);
			float dx = xf - x0f;
			if(my < _imgHeight) {
				int y = my;
				V += _vectorFieldSum[layerIndex][x + y * _imgWidth] * dx;
				V -= _vectorField[layerIndex][x + y * _imgWidth] * (y+1.f-my) * dx;
			} else V += _vectorFieldSum[layerIndex][x + (_imgHeight-1) * _imgWidth] * dx;
		}
		x0f = xf;
		y0f = yf;
	}
	return swapped ? -V : V;
}

// create vector field image based on image borders
// the vector field is parallel to the borders
void Image::initVectorField(const std::vector<std::vector<Shape>> &zones, const std::vector<Shape> &borders, int layerIndex) {
	std::vector<Segment> segments;
	std::vector<int> nearest;

	// 1. creates segments
	const auto addShapeSegments = [&segments](const Shape &shape)->void {
		for(int i = 1; i < shape._points.size(); ++i)
			segments.emplace_back(shape._points[i-1], shape._points[i]);
		for(const std::vector<glm::vec2> &hole : shape._holes)
			for(int i = 1; i < hole.size(); ++i)
				segments.emplace_back(hole[i-1], hole[i]);
	};
	bool anyVector = false;
	for(const std::vector<Shape> &ss : zones)
		for(const Shape &shape : ss) if(IS_VECTOR(shape._fillColor)) {
				anyVector = true;
				if(ADD_BORDER(shape._fillColor))
					addShapeSegments(shape);
			}
	if(!anyVector) {
		_vectorField[layerIndex].assign(_imgWidth * _imgHeight, glm::vec2(0.f, 0.f));
		_vectorFieldSum[layerIndex].assign(_vectorField[layerIndex].size(), glm::vec2(0.f, 0.f));
		return;
	}
	for(const Shape &shape : borders) addShapeSegments(shape);
	nearest.assign(_imgWidth * _imgHeight, -1);

	//////////////
	// 2. Dijkstra
	//////////////

	// The queue
	std::priority_queue<std::pair<float, int>> Q;
	// vector of distances
	std::vector<float> D(_imgWidth * _imgHeight, std::numeric_limits<float>::max());
	// diagonal of pixel
	float diag = glm::length(Globals::_SVGSize / glm::vec2(_imgWidth, _imgHeight));

	// init Q
	for(int i = 0; i < segments.size(); ++i) {
		const Segment &seg = segments[i];
		if(std::abs(seg._v.x) > std::abs(seg._v.y)) {
			int x0 = seg._a.x / Globals::_SVGSize.x * _imgWidth;
			int x1 = seg._b.x / Globals::_SVGSize.x * _imgWidth;
			if(x0 > x1) std::swap(x0, x1);
			x0 = std::max(0, x0-1);
			x1 = std::min(_imgWidth-1, x1+1);
			for(int x = x0; x <= x1; ++x) {
				float xf = (x + 0.5f) / _imgWidth * Globals::_SVGSize.x;
				float y = seg._a.y + (xf - seg._a.x) * seg._v.y / seg._v.x;
				int y0 = y / Globals::_SVGSize.y * _imgHeight - 1;
				int y1 = std::min(_imgHeight-1, y0 + 2);
				y0 = std::max(0, y0);
				for(int y = y0; y <= y1; ++y) {
					glm::vec2 p(xf, (y + 0.5f) / _imgHeight * Globals::_SVGSize.y);
					float d = seg.dist2(p);
					int pix = x + y * _imgWidth;
					if(d < D[pix]) {
						D[pix] = d;
						nearest[pix] = i;
						Q.emplace(-d, pix);
					}
				}
			}
		} else {
			int y0 = seg._a.y / Globals::_SVGSize.y * _imgHeight;
			int y1 = seg._b.y / Globals::_SVGSize.y * _imgHeight;
			if(y0 > y1) std::swap(y0, y1);
			y0 = std::max(0, y0-1);
			y1 = std::min(_imgHeight-1, y1+1);
			for(int y = y0; y <= y1; ++y) {
				float yf = (y + 0.5f) / _imgHeight * Globals::_SVGSize.y;
				float x = seg._a.x + (yf - seg._a.y) * seg._v.x / seg._v.y;
				int x0 = x / Globals::_SVGSize.x * _imgWidth - 1.f;
				int x1 = std::min(_imgWidth-1, x0 + 2);
				x0 = std::max(0, x0);
				for(int x = x0; x <= x1; ++x) {
					glm::vec2 p((x + 0.5f) / _imgWidth * Globals::_SVGSize.x, yf);
					float d = seg.dist2(p);
					int pix = x + y * _imgWidth;
					if(d < D[pix]) {
						D[pix] = d;
						nearest[pix] = i;
						Q.emplace(-d, pix);
					}
				}
			}
		}
	}

	// propagation
	while(!Q.empty()) {
		auto [d, pix] = Q.top();
		Q.pop();
		if(-d > D[pix]) continue;
		int seg_ind = nearest[pix];
		int x = pix % _imgWidth;
		int y = pix / _imgWidth;
		int x0 = x == 0 ? 0 : x-1;
		int x1 = std::min(_imgWidth, x+2);
		int y0 = y == 0 ? 0 : y-1;
		int y1 = std::min(_imgHeight, y+2);
		for(int a = x0; a < x1; ++a) {
			float xf = (a + 0.5f) / _imgWidth * Globals::_SVGSize.x;
			for(int b = y0; b < y1; ++b) {
				int pix2 = a + b * _imgWidth;
				if(nearest[pix2] == seg_ind) continue;
				glm::vec2 p(xf, (b + 0.5f) / _imgHeight * Globals::_SVGSize.y);
				float d2 = segments[seg_ind].dist2(p);
				if(d2 < D[pix2]) {
					D[pix2] = d2;
					nearest[pix2] = seg_ind;
					Q.emplace(-d2, pix2);
				}
			}
		}
	}

	/////////
	// Render image to know the zones in O(1)
	/////////

	_tmpZones[layerIndex] = &zones;
	Window::addToStack(layerIndex);
	std::chrono::milliseconds timespan(5);
	while(_tmpImages[layerIndex] == nullptr) std::this_thread::sleep_for(timespan);

	//////////
	// Compute the two vector images
	//////////

	// vector field
	_vectorField[layerIndex].resize(nearest.size());
	for(int i = 0; i < nearest.size(); ++i) {
		if(nearest[i] == -1) {
			std::cerr << "Nearest not filled" << std::endl;
			exit(1);
		}
		bool R = _tmpImages[layerIndex][3*i] > 128;
		bool G = _tmpImages[layerIndex][3*i+1] > 128;
		int y = i / _imgWidth;
		if(R || G) {
			int x = i % _imgWidth;
			glm::vec2 p((x + .5f) * Globals::_SVGSize.x / _imgWidth, (y + .5f) * Globals::_SVGSize.y / _imgHeight);
			float angle = segments[nearest[i]].normal_angle(p);
			if(G) angle += M_PI_2;
			_vectorField[layerIndex][i].x = std::cos(angle);
			_vectorField[layerIndex][i].y = std::sin(angle);
		} else {
			_vectorField[layerIndex][i].x = 0.f;
			_vectorField[layerIndex][i].y = 0.f;
		}
	}

	// vector field sum
	_vectorFieldSum[layerIndex] = _vectorField[layerIndex];
	for(int i = _imgWidth; i < nearest.size(); ++i)
		_vectorFieldSum[layerIndex][i] += _vectorFieldSum[layerIndex][i-_imgWidth];
	for(int i = 1; i < nearest.size(); ++i) if(i % _imgWidth != 0)
		_vectorFieldSum[layerIndex][i] += _vectorFieldSum[layerIndex][i-1];

	// smoothing
	for(int i = 0; i < nearest.size(); ++i) {
		int y = i / _imgWidth;
		int x = i % _imgWidth;
		if(_vectorField[layerIndex][i] == glm::vec2(0., 0.)) continue;
		const int K = 60;
		glm::vec2 s(0.f, 0.f);
		for(int k = K; k >= 10; k >>= 1) {
			const int x0 = std::max(0, x-k);
			const int x1 = std::min(_imgWidth-1, x+k);
			const int y0 = std::max(0, y-k);
			const int y1 = std::min(_imgHeight-1, y+k);
			s += _vectorFieldSum[layerIndex][x1 + y1*_imgWidth];
			if(x0 > 0) s -= _vectorFieldSum[layerIndex][x0-1 + y1*_imgWidth];
			if(y0 > 0) s -= _vectorFieldSum[layerIndex][x1 + (y0-1)*_imgWidth];
			if(x0 > 0 && y0 > 0) s += _vectorFieldSum[layerIndex][x0-1 + (y0-1)*_imgWidth];
		}
		float angle = 2. * std::atan2(s.y, s.x);
		_vectorField[layerIndex][i].x = std::cos(angle);
		_vectorField[layerIndex][i].y = std::sin(angle);
	}

	// vector field sum
	_vectorFieldSum[layerIndex] = _vectorField[layerIndex];
	for(int i = _imgWidth; i < nearest.size(); ++i)
		_vectorFieldSum[layerIndex][i] += _vectorFieldSum[layerIndex][i-_imgWidth];

	delete[] _tmpImages[layerIndex];
}

void Image::computeImage(int layer) {
	GLuint framebuffer;
	glGenFramebuffers(1, &framebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

	GLuint renderedTexture;
	glGenTextures(1, &renderedTexture);
	glBindTexture(GL_TEXTURE_2D, renderedTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _imgWidth, _imgHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderedTexture, 0);

	assert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);

	glClearColor(0.f, 0.f, 0.f, 0.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, _imgWidth, _imgHeight);

	// Draw
	glLoadIdentity();
	gluOrtho2D(0.f, Globals::_SVGSize.x, 0.f, Globals::_SVGSize.y);
	glBegin(GL_TRIANGLES);
	for(const std::vector<Shape> &zones : *_tmpZones[layer]) {
		for(const Shape &zone : zones) if(IS_VECTOR(zone._fillColor)) {
				if(IS_ORTHO(zone._fillColor)) glColor3f(1.f, 0.f, 0.f);
				else glColor3f(0.f, 1.f, 0.f);
				for(uint i : zone._triangles[0])
					glVertex2f(zone._points[i].x, zone._points[i].y);
				glColor3f(0.f, 0.f, 0.f);
				for(uint i = 0; i < zone._holes.size(); ++i)
					for(uint j : zone._triangles[i+1])
						glVertex2f(zone._holes[i][j].x, zone._holes[i][j].y);
			}
	}
	glEnd();

	glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));

	GLubyte* pixels = new GLubyte[3*_imgWidth*_imgHeight];
	GLint pack = 7;
	while(_imgWidth&pack) pack >>= 1;
	++ pack;
	glPixelStorei(GL_PACK_ALIGNMENT, pack);
	glReadPixels(0, 0, _imgWidth, _imgHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels);

	// Clean up
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDeleteTextures(1, &renderedTexture);
	glDeleteFramebuffers(1, &framebuffer);

	_tmpImages[layer] = pixels;
}

Image::Segment::Segment(const glm::vec2 &a, const glm::vec2 &b) {
	_a = a;
	_b = b;
	_v = b - a;
	_len = std::sqrt(_v.x*_v.x + _v.y*_v.y);
	_v /= _len;
}

inline float Image::Segment::dist2(const glm::vec2 &p) const {
	glm::vec2 q = p - _a;
	float x = _v.x * q.x + _v.y * q.y;
	float y = _v.y * q.x - _v.x * q.y;
	if(x < 0) return x*x + y*y;
	if(x > _len) {
		x -= _len;
		return x*x + y*y;
	}
	return y*y;
}

inline float Image::Segment::normal_angle(const glm::vec2 &p) const {
	float t = std::min(_len, std::max(0.f, _v.x * (p.x - _a.x) + _v.y * (p.y - _a.y)));
	return std::atan2(p.y - _a.y - t * _v.y, p.x - _a.x - t * _v.x);
}


//////////////////
//
//      SHAPE
//
//////////////////

// is p inside of _points
bool Shape::isInside(const glm::vec2 &p) const {
	if(Globals::isInPoly(_points, p)) {
		for(const std::vector<glm::vec2> &hole : _holes)
			if(Globals::isInPoly(hole, p))
				return false;
		return true;
	}
	return false;
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

