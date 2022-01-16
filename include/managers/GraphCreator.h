//
// Created by bedela on 18/05/2020.
//

#ifndef HAMILTON_GRAPHCREATOR_H
#define HAMILTON_GRAPHCREATOR_H

#include "graphics/Globals.h"

class GraphCreator {
public:
    static void graphFromSvg(const std::string &,
                             std::vector<Shape> &,
                             std::vector<std::vector<Shape>> &,
                             std::vector<std::vector<Shape>> &,
                             std::vector<Graph> &,
                             int);

private:
    static void                 remove2coPoints(Shape &, Graph &);
    static void                 getShapeFromSVG(const std::string &, std::vector<Shape> &);
    static std::vector<std::vector<Shape>> fuseShapes(std::vector<Shape> &, float);
};

#endif //HAMILTON_GRAPHCREATOR_H
