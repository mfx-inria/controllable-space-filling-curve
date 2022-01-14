//
// Created by bedela on 18/05/2020.
//

#ifndef HAMILTON_GRAPHCREATOR_H
#define HAMILTON_GRAPHCREATOR_H

#include "graphics/Globals.h"

#include <string>

class GraphCreator
{
private:
    static inline bool _isSVGEdit = true;
    static inline bool _isInkscape = false;
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
    static int                  getColor(unsigned int);
    static std::vector<std::vector<Shape>> fuseShapes(std::vector<Shape> &);
    static std::vector<std::vector<Shape>> fuseInkscapeShapes(std::vector<Shape> &);
    static void                 checkInkscape(const std::string &);
};

#endif //HAMILTON_GRAPHCREATOR_H
