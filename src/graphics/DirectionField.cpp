#include "graphics/DirectionField.h"
#include "graphics/Window.h"

#define GL_GLEXT_PROTOTYPES
#include <GL/freeglut.h>

#include <iostream>
#include <queue>

struct Segment {
	glm::vec2 _a, _b, _v;
	float _len;

	Segment(const glm::vec2 &a, const glm::vec2 &b) {
		_a = a;
		_b = b;
		_v = b - a;
		_len = std::sqrt(_v.x*_v.x + _v.y*_v.y);
		_v /= _len;
	}
	
	inline float dist2(const glm::vec2 &p) const {
		const glm::vec2 q = p - _a;
		float x = _v.x * q.x + _v.y * q.y;
		const float y = _v.y * q.x - _v.x * q.y;
		if(x < 0) return x*x + y*y;
		if(x > _len) {
			x -= _len;
			return x*x + y*y;
		}
		return y*y;
	}
	
	inline float normal_angle(const glm::vec2 &p) const {
		const float t = std::min(_len, std::max(0.f, _v.x * (p.x - _a.x) + _v.y * (p.y - _a.y)));
		return std::atan2(p.y - _a.y - t * _v.y, p.x - _a.x - t * _v.x);
	}
};

void DirectionField::init(int layerNb) {
	DirectionField::_imgWidth = std::sqrt((_pixelsWanted * Globals::_SVGSize.x) / Globals::_SVGSize.y);
	DirectionField::_imgHeight = _pixelsWanted / _imgWidth;
	_vectorField.resize(layerNb);
	_vectorFieldSum.resize(layerNb);
	_tmpImages.resize(layerNb);
}

// get vector from the pixel of the vectorfield
// image corresponding to the position of point
glm::vec2 DirectionField::getVecAtPos(const glm::vec2 &point, int layerIndex) {
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

glm::vec2 DirectionField::getVecUnder(const glm::vec2 &a, const glm::vec2 &b, int layerIndex) {
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
void DirectionField::initVectorField(const std::vector<std::vector<Shape>> &zones, const std::vector<Shape> &borders, int layerIndex) {
	std::vector<Segment> segments;
	std::vector<int> nearest;

	// 1. creates segments
	const auto addShapeSegments = [&segments](const Shape &shape)->void {
		for(int i = 1; i < (int) shape._points.size(); ++i)
			segments.emplace_back(shape._points[i-1], shape._points[i]);
		for(const std::vector<glm::vec2> &hole : shape._holes)
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
	typedef std::pair<float, int> QEl;
	std::priority_queue<QEl, std::vector<QEl>, std::greater<QEl>> Q;
	// vector of distances
	std::vector<float> D(_imgWidth * _imgHeight, std::numeric_limits<float>::max());
	const int imS[2] { _imgWidth, _imgHeight };
	const float rat[2] { Globals::_SVGSize.x / _imgWidth, Globals::_SVGSize.y / _imgHeight };

	// init Q
	for(int i = 0; i < (int) segments.size(); ++i) {
		const Segment &seg = segments[i];
		const int h = std::abs(seg._v.x) < std::abs(seg._v.y);
		int x0 = seg._a[h] / rat[h], x1 = seg._b[h] / rat[h];
		if(x0 > x1) std::swap(x0, x1);
		x0 = std::max(0, x0-1);
		x1 = std::min(imS[h]-1, x1+1);
		for(int x = x0; x <= x1; ++x) {
			glm::vec2 p;
			p[h] = (x + 0.5f) * rat[h];
			const float y = seg._a[h^1] + (p[h] - seg._a[h]) * seg._v[h^1] / seg._v[h];
			const int y0 = std::max(0, int(y / rat[h^1] - 1.f));
			const int y1 = std::min(imS[h^1]-1, y0 + 2);
			for(int y = y0; y <= y1; ++y) {
				p[h^1] = (y + 0.5f) * rat[h^1];
				const float d = seg.dist2(p);
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
			float xf = (a + 0.5f) * rat[0];
			for(int b = y0; b < y1; ++b) {
				int pix2 = a + b * _imgWidth;
				if(nearest[pix2] == seg_ind) continue;
				const glm::vec2 p(xf, (b + 0.5f) * rat[1]);
				const float d2 = segments[seg_ind].dist2(p);
				if(d2 < D[pix2]) {
					D[pix2] = d2;
					nearest[pix2] = seg_ind;
					Q.emplace(d2, pix2);
				}
			}
		}
	}

	/////////
	// Render image to know the zones in O(1)
	/////////

	std::future<u_char*> image_future = _tmpImages[layerIndex].get_future();
	Window::addToStack(layerIndex, &zones);
	_vectorField[layerIndex].resize(nearest.size());
	u_char* image = image_future.get();

	//////////
	// Compute the two vector images
	//////////

	// vector field
	for(int i = 0; i < (int) nearest.size(); ++i) {
		if(nearest[i] == -1) THROW_ERROR("Nearest not filled!");
		bool R = image[3*i] > 128;
		bool G = image[3*i+1] > 128;
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
	for(int i = _imgWidth; i < (int) nearest.size(); ++i)
		_vectorFieldSum[layerIndex][i] += _vectorFieldSum[layerIndex][i-_imgWidth];
	for(int i = 1; i < (int) nearest.size(); ++i) if(i % _imgWidth != 0)
		_vectorFieldSum[layerIndex][i] += _vectorFieldSum[layerIndex][i-1];

	// smoothing
	for(int i = 0; i < (int) nearest.size(); ++i) {
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
	for(int i = _imgWidth; i < (int) nearest.size(); ++i)
		_vectorFieldSum[layerIndex][i] += _vectorFieldSum[layerIndex][i-_imgWidth];

	delete[] image;
}

void DirectionField::computeImage(int layer, const std::vector<std::vector<Shape>> *zones) {
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
	for(const std::vector<Shape> &zones : *zones) {
		for(const Shape &zone : zones) if(IS_VECTOR(zone._objcetive)) {
				if(IS_ORTHO(zone._objcetive)) glColor3f(1.f, 0.f, 0.f);
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

	_tmpImages[layer].set_value(pixels);
}
