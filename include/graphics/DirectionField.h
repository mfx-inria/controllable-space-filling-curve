#include "graphics/Globals.h"

#include <glm/vec2.hpp>

#include <vector>
#include <future>

class DirectionField {

private:
	static inline std::vector<std::vector<glm::vec2>>   _vectorField;
	static inline std::vector<std::vector<glm::vec2>>   _vectorFieldSum;

public:
	static inline int                                   _pixelsWanted = 1750000;
	static inline int                                   _imgWidth;
	static inline int                                   _imgHeight;

	static inline std::vector<std::promise<u_char*>>	_tmpImages;

public:
	static void			init(int);
	static void         initVectorField(const std::vector<std::vector<Shape>> &, const std::vector<Shape> &, int);
	static void         computeImage(int, const std::vector<std::vector<Shape>> *);
	static glm::vec2    getVecAtPos(const glm::vec2 &, int);
	static glm::vec2    getVecUnder(const glm::vec2 &, const glm::vec2 &, int);

};