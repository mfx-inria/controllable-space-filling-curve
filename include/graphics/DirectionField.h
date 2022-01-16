#include "graphics/Globals.h"

#include <glm/vec2.hpp>

#include <vector>

class DirectionField {

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
	static void			init(int);
	static void         resize(int);
	static void         initVectorField(const std::vector<std::vector<Shape>> &, const std::vector<Shape> &, int);
	static void         computeImage(int);
	static glm::vec2    getVecAtPos(const glm::vec2 &, int);
	static glm::vec2    getVecUnder(const glm::vec2 &, const glm::vec2 &, int);

};