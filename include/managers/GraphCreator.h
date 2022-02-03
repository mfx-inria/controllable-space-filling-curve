//
// Created by bedela on 18/05/2020.
//

#ifndef HAMILTON_GRAPHCREATOR_H
#define HAMILTON_GRAPHCREATOR_H

#include "graphics/Globals.h"

class GraphCreator {
public:
    static void graphFromSvg(const std::string &fileName,
								std::vector<Shape> &shapes,
								std::vector<std::vector<Shape>> &objZones,
								std::vector<std::vector<Shape>> &colorZones,
								std::vector<Graph> &graphs,
								int layerIndex);
};

#endif //HAMILTON_GRAPHCREATOR_H
