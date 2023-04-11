//
// Created by adrien on 17/11/2020.
//

#include "localAlgo/ObjectiveFunctions.h"
#include "graphics/DirectionField.h"

#include <algorithm>

ObjectiveFunctions::ObjectiveFunctions(Shape &&shape, std::vector<Shape> &&objZones, int layerIndex)
        : _shape(shape), _objZones(objZones), _layerIndex(layerIndex)
{ }

void ObjectiveFunctions::computeZone() {
    _zone.resize(_points.size());
    for(int i = 0; i < (int) _points.size(); ++i) {
        for(int j = 0; j < (int) _objZones.size(); ++j) {
            if(_objZones[j].isInside(_points[i])) {
                _zone[i] = j;
                break;
            }
        }
    }
}

void ObjectiveFunctions::computeData() {
    // Init usefull data
    _distribs.resize(_objZones.size());
    _vecWeight = 0.;
    _vecScore = 0.;
    _vecArea = 0.;
    for(int i = 0; i < (int) _objZones.size(); ++i) {
        if(IS_ISOTROPY(_objZones[i]._objcetive))
            _distribs[i].assign(_nSamples, 0.);
        else if(IS_VECTOR(_objZones[i]._objcetive))
            _vecArea += _objZones[i]._area;
    }
    _nCrosses = 0;

    // Add selected edges
    for(int i = 0; i < (int) _points.size(); ++i)
        for(int j : _cLinks[i]) if(i < j)
                addSegment(i, j);

    // Get final score
    _score = getMeanScore();
    _nbUpdates = 0;
}

void ObjectiveFunctions::calculateScore() {
    // Compte segment cell radius for alignment objective
    _segCellRadius = .2f * _shape._area / _points.size();
    int nLink = 0;
    double sumLenLink = 0.;
    for(int i = 1; i < (int) _points.size(); ++i)
        for(int j : _links[i]) if(j < i) {
                ++ nLink;
                sumLenLink += glm::distance(_points[i], _points[j]);
            }
    _segCellRadius *= nLink / sumLenLink;

    // Compute edges
    _segments.resize(_points.size());
    for(int i = 0; i < (int) _points.size(); ++i) {
        _segments[i].clear();
        _segments[i].reserve(_links[i].size());
        int k = _zone[i];
        for(int j : _links[i]) {
            _segments[i].emplace_back();
            std::vector<double> ts;
            _objZones[k].getInter(_points[i], _points[j], ts);
            if(ts.empty()) {
                if(IS_VECTOR(_objZones[k]._objcetive))
                    createEdgeVec(_points[i], _points[j], _segments[i].back().vecW, _segments[i].back().vecS);
                else if(IS_ISOTROPY(_objZones[k]._objcetive))
                    _segments[i].back().iso.push_back(createEdgeIso(k, _points[j] - _points[i]));
                continue;
            }
            glm::dvec2 v = _points[j] - _points[i];
            ts.push_back(0.);
            std::vector<std::pair<double, int>> tss;
            int l = k;
            while(true) {
                std::sort(ts.begin(), ts.end());
                if(_zone[j] == l) ts.push_back(1.);
                for(double t : ts) tss.emplace_back(t, _objZones[l]._printColor);
                if(IS_VECTOR(_objZones[l]._objcetive))
                    for(int m = 0; m < (int) ts.size(); m += 2)
                        createEdgeVec(_points[i]+ts[m]*v, _points[i]+ts[m+1]*v, _segments[i].back().vecW, _segments[i].back().vecS);
                else if(IS_ISOTROPY(_objZones[l]._objcetive)) {
                    double t = 0;
                    for(int m = 0; m < (int) ts.size(); m += 2) t += ts[m+1] - ts[m];
                    _segments[i].back().iso.push_back(createEdgeIso(l, t*v));
                }
                ts.clear();
                do {
                    l = (l+1) % _objZones.size();
                    if(l == k) break;
                    if(_objZones[l]._objcetive != NOTHING)
                        _objZones[l].getInter(_points[i], _points[j], ts);
                } while(ts.empty());
                if(l == k) break;
            }
            std::sort(tss.begin(), tss.end());
            for(l = 2; l < (int) tss.size(); l += 2)
                if(tss[l-1].second != tss[l].second)
                    ++ _segments[i].back().zon;
        }
    }

    computeData();
}

const ObjectiveFunctions::Segment& ObjectiveFunctions::getSegment(int i, int j) {
    int k = 0;
    while(k < (int) _links[i].size() && _links[i][k] != j) ++k;
    return _segments[i][k];
}

void ObjectiveFunctions::addSegment(int i, int j) {
    const Segment &s = getSegment(i, j);
    _vecWeight += s.vecW;
    _vecScore += s.vecS;
    _nCrosses += s.zon;
    for(const Edge &e : s.iso) {
        _distribs[e.zone][e.i] += e.len;
        _distribs[e.zone][e.j] += e.l2;
    }
}

void ObjectiveFunctions::rmSegment(int i, int j) {
    const Segment &s = getSegment(i, j);
    _vecWeight -= s.vecW;
    _vecScore -= s.vecS;
    _nCrosses -= s.zon;
    for(const Edge &e : s.iso) {
        _distribs[e.zone][e.i] -= e.len;
        _distribs[e.zone][e.j] -= e.l2;
    }
}

double ObjectiveFunctions::getMeanScore() {
    double isoScore = 0.;
    for(int i = 0; i < (int) _objZones.size(); ++i) {
        if(_objZones[i]._objcetive == ANISOTROPY)
            isoScore += _objZones[i]._area * (1. - getWassersteinDistance(i));
        else if(_objZones[i]._objcetive == ISOTROPY)
            isoScore += _objZones[i]._area * getWassersteinDistance(i);
    }
    double vecScore = _vecArea * (_vecWeight <= 0. ? .5f : _vecScore / _vecWeight);
    return (isoScore + vecScore) / _shape._area + _nCrosses;
}

double ObjectiveFunctions::checkDirection(const std::vector<int> &points,
                                         const std::vector<std::pair<int, int>> &current,
                                         const std::vector<std::pair<int, int>> &candidat)
{
    // Maybe will need to save data then reload data if numerical errors occur
    for(const auto& [i, j] : current) rmSegment(points[i], points[j]);
    for(const auto& [i, j] : candidat) addSegment(points[i], points[j]);
    double diff = getMeanScore() - _score;
    for(const auto& [i, j] : candidat) rmSegment(points[i], points[j]);
    for(const auto& [i, j] : current) addSegment(points[i], points[j]);
    return diff;
}

void ObjectiveFunctions::applyShift(const std::vector<int> &points,
                                    const std::vector<std::pair<int, int>> &current,
                                    const std::vector<std::pair<int, int>> &candidat)
{
    for(const auto& [i, j] : current) {
        rmSegment(points[i], points[j]);
        removeLink(points[i], points[j]);
    }
    for(const auto& [i, j] : candidat) {
        addSegment(points[i], points[j]);
        createLink(points[i], points[j]);
    }
    if(++_nbUpdates > 1000) computeData();
    else _score = getMeanScore();
}

////////////////
//
// VECTOR FIELD
//
////////////////

void ObjectiveFunctions::createEdgeVec(const glm::dvec2 &a, const glm::dvec2 &b, double &W, double &S) {
    glm::dvec2 vecAB = b - a;
    double angleAB = std::atan2(vecAB.y, vecAB.x);
    vecAB *= _segCellRadius / glm::length(vecAB);
    std::swap(vecAB.x, vecAB.y);
    vecAB.y = -vecAB.y;
    glm::dvec2 V = DirectionField::getVecUnder(a - vecAB, b - vecAB, _layerIndex);
    V += DirectionField::getVecUnder(b - vecAB, b + vecAB, _layerIndex);
    V += DirectionField::getVecUnder(b + vecAB, a + vecAB, _layerIndex);
    V += DirectionField::getVecUnder(a + vecAB, a - vecAB, _layerIndex);
    double lV = glm::length(V);
    if(lV < 1e-5f) return;

    double angle = std::atan2(V.y, V.x) / 2. - angleAB;
    while(angle > M_PI_2) angle -= M_PI;
    while(angle < -M_PI_2) angle += M_PI;
    angle /= M_PI_2;

    //edge.l2 = edge.len * std::abs(angle);

    W += lV;
    S += lV * std::abs(angle);
}

////////////
//
// ISOTROPY
//
////////////

ObjectiveFunctions::Edge ObjectiveFunctions::createEdgeIso(int z, const glm::dvec2 &vec) {
    Edge edge(z);
    edge.len = glm::length(vec);
    double angle = vec.x == 0. ? M_PI_2 : std::atan(vec.y / vec.x);
    if(angle < 0.) angle += M_PI;
    double d = M_PI / _nSamples;
    edge.i = angle / d;
    double mid = (edge.i + .5f) * d;
    double diff = angle - mid;
    if(diff > 0.) {
        edge.l2 = edge.len * diff / d;
        edge.j = edge.i+1;
        if(edge.j == _nSamples) edge.j = 0;
    } else {
        edge.l2 = - edge.len * diff / d;
        edge.j = edge.i == 0 ? _nSamples-1 : edge.i-1;
    }
    edge.len -= edge.l2;
    return edge;
}

// Return the Wasserstein distance between the current orientation distribution
// and the uniform distribution
double ObjectiveFunctions::getWassersteinDistance(int z) const {
    double totLength = 0.;
    for(double l : _distribs[z]) totLength += l;
    if(totLength == 0.) return 0.;
    double bin = totLength / _nSamples;
    double alpha = 0.;
    for(int i = 1; i < _nSamples; ++i)
        alpha += (_distribs[z][i] - bin) * i;
    alpha /= totLength;
    int i = 0, j = 0;
    double next1 = bin, next2 = _distribs[z][0];
    double pred = 0.;
    double dist = 0.;
    while(i < _nSamples || j < _nSamples) {
        double dij = j - i - alpha;
        if(next1 < next2) {
            double dt = next1 - pred;
            dist += dt * dij*dij;
            pred = next1;
            ++ i;
            next1 += bin;
        } else {
            double dt = next2 - pred;
            dist += dt * dij*dij;
            pred = next2;
            ++ j;
            if(j < _nSamples) next2 += _distribs[z][j];
            else next2 += totLength;
        }
    }
    return std::sqrt(12. * dist / (totLength * _nSamples * _nSamples));
}

///////////
//
//  GETTERS
//
///////////

const std::vector<glm::dvec2>& ObjectiveFunctions::getPoints() const {
    return _points;
}

const std::vector<std::vector<int>>& ObjectiveFunctions::getLinks() const
{
    return _cLinks;
}

std::tuple<const std::vector<glm::dvec2> &, const std::vector<std::vector<int>> &, const std::vector<std::vector<int>> &> ObjectiveFunctions::getGraph() const
{
    return {_points, _cLinks, _links};
}

std::pair<const std::vector<glm::dvec2> &, const std::vector<std::vector<int>> &> ObjectiveFunctions::getCycle() const
{
    return {_points, _cLinks};
}

const std::vector<Shape>& ObjectiveFunctions::getObjZones() const {
    return _objZones;
}

double ObjectiveFunctions::getScore() const {
    return _score;
}

double ObjectiveFunctions::getArea() const {
    return _shape._area;
}

int ObjectiveFunctions::getColor(int i) const {
    return _objZones[_zone[i]]._printColor;
}

const Shape& ObjectiveFunctions::getBorder() const
{
    return _shape;
}


////////////////
//
//  TOOLS
//
////////////////

inline void ObjectiveFunctions::removeLink(int pointA, int pointB)
{
    if(_cLinks[pointA].size() == 1) _cLinks[pointA].clear();
    else {
        if(_cLinks[pointA][0] == pointB) _cLinks[pointA][0] = _cLinks[pointA][1];
        _cLinks[pointA].pop_back();
    }
    if(_cLinks[pointB].size() == 1) _cLinks[pointB].clear();
    else {
        if(_cLinks[pointB][0] == pointA) _cLinks[pointB][0] = _cLinks[pointB][1];
        _cLinks[pointB].pop_back();
    }
}

inline void ObjectiveFunctions::createLink(int pointA, int pointB)
{
    _cLinks[pointA].push_back(pointB);
    _cLinks[pointB].push_back(pointA);
}
