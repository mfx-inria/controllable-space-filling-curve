//
// Created by adrien on 17/11/2020.
//

#ifndef HAMILTON_OBJECTIVEFUNCTIONS_H
#define HAMILTON_OBJECTIVEFUNCTIONS_H

#include "graphics/Globals.h"

class ObjectiveFunctions
{

struct Edge {
    int zone;
    int i, j;
    float len, l2;
    Edge() = default;
    Edge(int z): zone(z) {}
};
struct Segment {
    float               vecW, vecS;
    std::vector<Edge>   iso;
    int                 zon;
    Segment(): vecW(0.f), vecS(0.f), iso(), zon(0) {}
};

protected:
    int                                     _layerIndex;
    float                                   _score = 0.f;
    // ISOTROPY
    const static int                        _nSamples = 100;
    std::vector<std::vector<float>>         _distribs;
    // VECTOR
    float                                   _segCellRadius;
    float                                   _vecArea;
    float                                   _vecWeight;
    float                                   _vecScore;
    // ZONING
    int                                     _nCrosses;
    
    int                                     _nbUpdates;
    std::vector<std::vector<Segment>>       _segments;

    std::vector<glm::vec2>          _points;
    std::vector<std::vector<int>>   _cLinks;
    std::vector<std::vector<int>>   _originalLinks;
    std::vector<Shape>              _zones;
    Shape                           _border;

public:
    std::vector<int>                _zone;

public:
    ObjectiveFunctions() = default;
    ObjectiveFunctions(std::vector<Shape> &&, Shape &&, int);

    float   checkDirection(const std::vector<int> &, const std::vector<std::pair<int, int>> &, const std::vector<std::pair<int, int>> &);
    void    applyShift(const std::vector<int> &, const std::vector<std::pair<int, int>> &, const std::vector<std::pair<int, int>> &);

    float   getScore() const;
    void    calculateScore();
    const std::vector<glm::vec2>&                                                                                               getPoints() const;
    const std::vector<std::vector<int>> &                                                                                       getLinks() const;
    std::tuple<const std::vector<glm::vec2> &, const std::vector<std::vector<int>> &, const std::vector<std::vector<int>> &>    getGraph() const;
    std::pair<const std::vector<glm::vec2> &, const std::vector<std::vector<int>> &>                                            getCycle() const;
    const Shape &               getBorder() const;
    const std::vector<Shape>&   getZones() const;
    int                         getStrokeColor(int) const;
    float                       getArea() const;

protected:
    void                        computeZone();
    void                        computeData();

private:
    float                       getMeanScore();
    const Segment&              getSegment(int, int);
    void                        addSegment(int, int);
    void                        rmSegment(int, int);
    void                        removeLink(int, int);
    void                        createLink(int, int);

    void    createEdgeVec(const glm::vec2 &, const glm::vec2 &, float &, float &);
    Edge    createEdgeIso(int, const glm::vec2 &);
    float   getWassersteinDistance(int) const;

};

#endif //HAMILTON_OBJECTIVEFUNCTIONS_H
