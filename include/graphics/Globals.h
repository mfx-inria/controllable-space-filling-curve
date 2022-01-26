//
// Created by adrien_bedel on 18/11/19.
//

#ifndef HAMILTON_GLOBALS_H
#define HAMILTON_GLOBALS_H

#include <vector>
#include <thread>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>

typedef unsigned int uint;

// Objectives
enum { GREY,        BLUE,      RED,           GREEN,         YELLOW,       CYAN,        BLACK   };
enum OBJECTIVE { ANISOTROPY,  ISOTROPY,  VECTOR_FIELD,  ORTHO_VECTOR,  VECTOR_ZONE,  ORTHO_ZONE,  NOTHING };
const std::vector<glm::vec3> COLORS = {
	{ 0.5f, 0.5f, 0.5f }, // GREY
	{ 0.0f, 0.0f, 1.0f }, // BLUE
	{ 1.0f, 0.0f, 0.0f }, // RED
	{ 0.0f, 1.0f, 0.0f }, // GREEN
	{ 1.0f, 1.0f, 0.0f }, // YELLOW
	{ 0.0f, 1.0f, 1.0f }, // CYAN
	{ 0.0f, 0.0f, 0.0f }  // BLACK
};
bool ADD_BORDER(int);
bool IS_ISOTROPY(int);
bool IS_VECTOR(int);
bool IS_ORTHO(int);

// Class for graphs
class Graph {
public:
	std::vector<glm::vec2>              _points;
	std::vector<std::vector<int>>       _originalLinks;
	std::vector<std::vector<int>>       _cells;
public:
	Graph(){}
	static glm::vec2 getCellCenter(const std::vector<int> &, const std::vector<glm::vec2> &);
};

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
	bool isInside(const glm::vec2 &p) const;
	void triangulate();
};

// Class containing constants and some geometry primitives
class Globals {
public:
	// Printer parameters
	inline static double        _frameRayIn = 1.75 / 2.0;
	inline static double        _frameInArea;
	inline static double        _outRPrismeWidth = 0.4; // in mm == Buze Diametre out
	inline static double        _outRPrismeHeight = 0.2; // in mm == outPutHeight
	inline static double        _extrusionHeight = 0;
	inline static double        _buzeVolume = 6.1; // in mm3
	inline static double        _lengthToClear; // in frame in length

	// Parameters of randomness and multithreading
	inline static unsigned int  _seed = 5;
	inline static unsigned int  _nbThread = std::thread::hardware_concurrency();

	// Size of SVG file
	inline static glm::vec2     _SVGSize;
	inline static float			_d = 1.2; // spacing for the graph construction

public:
	static void		initVariables(int layerNb);
	static float	polygonArea(const std::vector<glm::vec2> &);
	static bool		isInPoly(const std::vector<glm::vec2> &, const glm::vec2 &);
	static bool		intersect(const glm::vec2 &, const glm::vec2 &, const glm::vec2 &, const glm::vec2 &);
	static bool		intersect(const glm::vec2 &, const glm::vec2 &, const glm::vec2 &, const glm::vec2 &, float &);
	static void		getInter(const glm::vec2 &, const glm::vec2 &, const Shape &, std::vector<float> &);
};

//===============
// USEFULL STUFF 
//===============

template <typename Scalar>
struct Box {
	Scalar x0 = std::numeric_limits<Scalar>::max();
	Scalar x1 = std::numeric_limits<Scalar>::lowest();
	Scalar y0 = std::numeric_limits<Scalar>::max();
	Scalar y1 = std::numeric_limits<Scalar>::lowest();

	Box() = default;
	template <typename Scalar2>
	Box(const Box<Scalar2> &b): x0(b.x0), x1(b.x1), y0(b.y0), y1(b.y1) {}

	template <typename Scalar2>
	inline void update(const glm::vec<2, Scalar2> &v) {
		x0 = std::min(x0, (Scalar) v.x);
		x1 = std::max(x1, (Scalar) v.x);
		y0 = std::min(y0, (Scalar) v.y);
		y1 = std::max(y1, (Scalar) v.y);
	}
	inline Scalar W() const { return x1-x0; } 
	inline Scalar H() const { return y1-y0; }
	inline glm::vec<2, Scalar> min() const { return {x0, y0}; }
	inline glm::vec<2, Scalar> max() const { return {x1, y1}; }
	inline Scalar diag() const { return glm::length(max() - min()); }
};

template<typename ...Args>
std::string str_format(const std::string &s, Args ...args) {
	int size = std::snprintf(nullptr, 0, s.c_str(), args...) + 1;
	auto buf = std::make_unique<char[]>(size);
	std::snprintf(buf.get(), size, s.c_str(), args...);
	return std::string(buf.get(), buf.get()+size-1);
}

namespace glm {
	template <typename T>
	inline T length2(const glm::vec<2, T> &v) { return dot(v, v); }
	template <typename T>
	inline T distance2(const glm::vec<2, T> &u, const glm::vec<2, T> &v) { return length2(v-u); }
}

struct CSFCerror : public std::exception {
	const std::string m_msg;
	CSFCerror(const std::string &msg, const std::string &file, int line):
		m_msg(msg + "\n    at " + file + ":" + std::to_string(line)) {}
	const char* what() const throw() { return m_msg.c_str(); }
};
#define THROW_ERROR(msg) throw CSFCerror((msg), __FILE__, __LINE__)

#endif //HAMILTON_GLOBALS_H
