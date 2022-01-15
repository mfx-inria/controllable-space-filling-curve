//
// Created by adrien_bedel on 18/11/19.
//

#ifndef HAMILTON_GLOBALS_H
#define HAMILTON_GLOBALS_H

#include <random>
#include <thread>
#include <vector>
#include <glm/glm.hpp>

enum { GREY,        BLUE,      RED,           GREEN,         YELLOW,       CYAN,        BLACK   };
enum { ANISOTROPY,  ISOTROPY,  VECTOR_FIELD,  ORTHO_VECTOR,  VECTOR_ZONE,  ORTHO_ZONE,  NOTHING };
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

class Shape;

class Globals
{
public:
    inline static double        _frameRayIn = 1.75 / 2.0;
    inline static double        _outRPrismeWidth = 0.4; // in mm == Buze Diametre out

public:
    static inline bool          _isRandom = true;
    static inline std::random_device _rd;
    static inline unsigned int  _seed = 5;
    inline static double        _frameInArea;
    inline static glm::vec2     _SVGSize;


    inline static double        _outRPrismeHeight = 0.2; // in mm == outPutHeight
    inline static double        _extrusionHeight = 0;
    inline static unsigned int  _nbThread = std::thread::hardware_concurrency();
    inline static double        _millimeter = 1.0; // in mm
    inline static double        _buzeVolume = 6.1; // in mm3
    inline static double        _lengthToClear; // in frame in length

public:
    static void         initVariables(int layerNb);
    static float        polygonArea(const std::vector<glm::vec2> &);
    static bool         isInPoly(const std::vector<glm::vec2> &, const glm::vec2 &);
    static bool         intersect(const glm::vec2 &, const glm::vec2 &, const glm::vec2 &, const glm::vec2 &);
    static bool         intersect(const glm::vec2 &, const glm::vec2 &, const glm::vec2 &, const glm::vec2 &, float &);
    static void         getInter(const glm::vec2 &, const glm::vec2 &, const Shape &, std::vector<float> &);
    static glm::vec2    getCenter(const std::vector<glm::vec2> &);
    static float        getDist(const glm::vec3 &, const glm::vec3 &);
};

class Image
{

struct Segment {
    glm::vec2 _a, _b, _v;
    float _len;
    Segment(const glm::vec2 &a, const glm::vec2 &b);
    float dist2(const glm::vec2 &p) const;
    float normal_angle(const glm::vec2 &p) const;
};

private:
    static inline std::vector<std::vector<glm::vec2>>   _vectorField;
    static inline std::vector<std::vector<glm::vec2>>   _vectorFieldSum;

public:
    static inline int                                   _pixelsWanted = 1750000;
    static inline int                                   _imgWidth;
    static inline int                                   _imgHeight;

    static inline std::vector<u_char*>                                  _tmpImages;
    static inline std::vector<const std::vector<std::vector<Shape>> *>  _tmpZones;

public:
    static void         resize(int);
    static void         initVectorField(const std::vector<std::vector<Shape>> &, const std::vector<Shape> &, int);
    static void         computeImage(int);
    static glm::vec2    getVecAtPos(const glm::vec2 &, int);
    static glm::vec2    getVecUnder(const glm::vec2 &, const glm::vec2 &, int);

};

class Shape
{
public:
    float                               _area;
    int                                 _fillColor;
    int                                 _strokeColor = 0;

    std::vector<glm::vec2>              _points;
    std::vector<std::vector<glm::vec2>> _holes;
    std::vector<std::vector<uint>>      _triangles;
public:
    Shape(){}
    Shape(const std::vector<glm::vec2> &pts) { _points = pts; }
    bool isInside(const glm::vec2 &p) const;
    void triangulate();
};

class Graph
{
public:
    std::vector<glm::vec2>              _points;
    std::vector<std::vector<int>>       _originalLinks;
    std::vector<std::vector<int>>       _cells;
public:
    Graph(){}
    static glm::vec2 getCellCenter(const std::vector<int> &, const std::vector<glm::vec2> &);
};

template<typename ...Args>
std::string str_format(const std::string &s, Args ...args) {
	int size = std::snprintf(nullptr, 0, s.c_str(), args...) + 1;
	auto buf = std::make_unique<char[]>(size);
	std::snprintf(buf.get(), size, s.c_str(), args...);
	return std::string(buf.get(), buf.get()+size-1);
}

#endif //HAMILTON_GLOBALS_H
