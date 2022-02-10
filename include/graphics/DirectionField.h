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
	static glm::vec2    getVecAtPos(const glm::vec2 &point, int layerIndex);
	static glm::vec2    getVecUnder(const glm::vec2 &a, const glm::vec2 &b, int layerIndex);

private:
	static inline std::vector<std::vector<glm::vec2>>   _vectorField;
	static inline std::vector<std::vector<glm::vec2>>   _vectorFieldSum;
};