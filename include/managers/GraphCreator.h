//
// Created by bedela on 18/05/2020.
//

#ifndef HAMILTON_GRAPHCREATOR_H
#define HAMILTON_GRAPHCREATOR_H

#include "graphics/Shape.h"

class GraphCreator {
public:
    static bool graphFromSvg(const Shape &shape, Graph &graph, int layerIndex);
};

#endif //HAMILTON_GRAPHCREATOR_H
