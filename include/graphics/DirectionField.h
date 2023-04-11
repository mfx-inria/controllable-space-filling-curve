#include "graphics/Shape.h"

#include <future>

class DirectionField {
public:
	constexpr static inline int		_pixelsWanted = 1750000;
	static inline int				_imgWidth;
	static inline int				_imgHeight;

public:
	static void			init(int layerNb);
	static void         initVectorField(const std::vector<std::vector<Shape>> &zones, const std::vector<Shape> &borders, int layerIndex);
	static glm::dvec2   getVecAtPos(const glm::dvec2 &point, int layerIndex);
	static glm::dvec2   getVecUnder(const glm::dvec2 &a, const glm::dvec2 &b, int layerIndex);

private:
	static inline std::vector<std::vector<glm::dvec2>>   _vectorField;
	static inline std::vector<std::vector<glm::dvec2>>   _vectorFieldSum;
};