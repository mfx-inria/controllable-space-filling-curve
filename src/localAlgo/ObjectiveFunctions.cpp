//
// Created by adrien on 17/11/2020.
//

#include "localAlgo/ObjectiveFunctions.h"

#include <algorithm>

ObjectiveFunctions::ObjectiveFunctions(std::vector<Shape> &&zones, Shape &&border, int layerIndex)
        : _zones(zones), _border(border), _layerIndex(layerIndex)
{ }

void ObjectiveFunctions::computeZone() {
    _zone.resize(_points.size());
    for(int i = 0; i < _points.size(); ++i) {
        for(int j = 0; j < _zones.size(); ++j) {
            if(_zones[j].isInside(_points[i])) {
                _zone[i] = j;
                break;
            }
        }
    }
}

void ObjectiveFunctions::computeData() {
    // Init usefull data
    _distribs.resize(_zones.size());
    _vecWeight = 0.f;
    _vecScore = 0.f;
    _vecArea = 0.f;
    for(int i = 0; i < _zones.size(); ++i) {
        if(IS_ISOTROPY(_zones[i]._objcetive))
            _distribs[i].assign(_nSamples, 0.f);
        else if(IS_VECTOR(_zones[i]._objcetive))
            _vecArea += _zones[i]._area;
    }
    _nCrosses = 0;

    // Add selected edges
    for(int i = 0; i < _points.size(); ++i)
        for(int j : _cLinks[i]) if(i < j)
                addSegment(i, j);

    // Get final score
    _score = getMeanScore();
    _nbUpdates = 0;
}

void ObjectiveFunctions::calculateScore() {
    // Compte segment cell radius for alignment objective
    _segCellRadius = .2f * _border._area / _points.size();
    int nLink = 0;
    float sumLenLink = 0.f;
    for(int i = 1; i < _points.size(); ++i)
        for(int j : _originalLinks[i]) if(j < i) {
                ++ nLink;
                sumLenLink += glm::distance(_points[i], _points[j]);
            }
    _segCellRadius *= nLink / sumLenLink;

    // Compute edges
    _segments.resize(_points.size());
    for(int i = 0; i < _points.size(); ++i) {
        _segments[i].clear();
        _segments[i].reserve(_originalLinks[i].size());
        int k = _zone[i];
        for(int j : _originalLinks[i]) {
            _segments[i].emplace_back();
            std::vector<float> ts;
            Globals::getInter(_points[i], _points[j], _zones[k], ts);
            if(ts.empty()) {
                if(IS_VECTOR(_zones[k]._objcetive))
                    createEdgeVec(_points[i], _points[j], _segments[i].back().vecW, _segments[i].back().vecS);
                else if(IS_ISOTROPY(_zones[k]._objcetive))
                    _segments[i].back().iso.push_back(createEdgeIso(k, _points[j] - _points[i]));
                continue;
            }
            glm::vec2 v = _points[j] - _points[i];
            ts.push_back(0.f);
            std::vector<std::pair<float, int>> tss;
            int l = k;
            while(true) {
                std::sort(ts.begin(), ts.end());
                if(_zone[j] == l) ts.push_back(1.f);
                for(float t : ts) tss.emplace_back(t, _zones[l]._printColor);
                if(IS_VECTOR(_zones[l]._objcetive))
                    for(int m = 0; m < ts.size(); m += 2)
                        createEdgeVec(_points[i]+ts[m]*v, _points[i]+ts[m+1]*v, _segments[i].back().vecW, _segments[i].back().vecS);
                else if(IS_ISOTROPY(_zones[l]._objcetive)) {
                    float t = 0;
                    for(int m = 0; m < ts.size(); m += 2) t += ts[m+1] - ts[m];
                    _segments[i].back().iso.push_back(createEdgeIso(l, t*v));
                }
                ts.clear();
                do {
                    l = (l+1) % _zones.size();
                    if(l == k) break;
                    if(_zones[l]._objcetive != NOTHING)
                        Globals::getInter(_points[i], _points[j], _zones[l], ts);
                } while(ts.empty());
                if(l == k) break;
            }
            std::sort(tss.begin(), tss.end());
            for(l = 2; l < tss.size(); l += 2)
                if(tss[l-1].second != tss[l].second)
                {
                    if (tss[l-1].second != -1 && tss[l].second != -1)
                        _segments[i].back().zon += 10;
                    else
                        _segments[i].back().zon++;
                }
        }
    }

    computeData();

    // Get final score
    _score = getMeanScore();
}

const ObjectiveFunctions::Segment& ObjectiveFunctions::getSegment(int i, int j) {
    int k = 0;
    while(k < _originalLinks[i].size() && _originalLinks[i][k] != j) ++k;
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

float ObjectiveFunctions::getMeanScore() {
    float isoScore = 0.f;
    for(int i = 0; i < _zones.size(); ++i) {
        if(_zones[i]._objcetive == ANISOTROPY)
            isoScore += _zones[i]._area * (1.f - getWassersteinDistance(i));
        else if(_zones[i]._objcetive == ISOTROPY)
            isoScore += _zones[i]._area * getWassersteinDistance(i);
    }
    float vecScore = _vecArea * (_vecWeight <= 0.f ? .5f : _vecScore / _vecWeight);
    return (isoScore + vecScore) / _border._area + _nCrosses;
}

float ObjectiveFunctions::checkDirection(const std::vector<int> &points,
                                         const std::vector<std::pair<int, int>> &current,
                                         const std::vector<std::pair<int, int>> &candidat)
{
    // Maybe will need to save data then reload data if numerical errors occur
    for(const auto& [i, j] : current) rmSegment(points[i], points[j]);
    for(const auto& [i, j] : candidat) addSegment(points[i], points[j]);
    float diff = getMeanScore() - _score;
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
    if(++_nbUpdates > 2500) computeData();
    else _score = getMeanScore();
}

////////////////
//
// VECTOR FIELD
//
////////////////

void ObjectiveFunctions::createEdgeVec(const glm::vec2 &a, const glm::vec2 &b, float &W, float &S) {
    glm::vec2 vecAB = b - a;
    float angleAB = std::atan2(vecAB.y, vecAB.x);
    vecAB *= _segCellRadius / glm::length(vecAB);
    std::swap(vecAB.x, vecAB.y);
    vecAB.y = -vecAB.y;
    glm::vec2 V = Image::getVecUnder(a - vecAB, b - vecAB, _layerIndex);
    V += Image::getVecUnder(b - vecAB, b + vecAB, _layerIndex);
    V += Image::getVecUnder(b + vecAB, a + vecAB, _layerIndex);
    V += Image::getVecUnder(a + vecAB, a - vecAB, _layerIndex);
    float lV = glm::length(V);
    if(lV < 1e-5f) return;

    float angle = std::atan2(V.y, V.x) / 2.f - angleAB;
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

ObjectiveFunctions::Edge ObjectiveFunctions::createEdgeIso(int z, const glm::vec2 &vec) {
    Edge edge(z);
    edge.len = glm::length(vec);
    float angle = vec.x == 0.f ? M_PI_2 : std::atan(vec.y / vec.x);
    if(angle < 0.f) angle += M_PI;
    float d = M_PI / _nSamples;
    edge.i = angle / d;
    float mid = (edge.i + .5f) * d;
    float diff = angle - mid;
    if(diff > 0.f) {
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
float ObjectiveFunctions::getWassersteinDistance(int z) const {
    float totLength = 0.f;
    for(float l : _distribs[z]) totLength += l;
    if(totLength == 0.f) return 0.f;
    float bin = totLength / _nSamples;
    float alpha = 0.f;
    for(int i = 1; i < _nSamples; ++i)
        alpha += (_distribs[z][i] - bin) * i;
    alpha /= totLength;
    int i = 0, j = 0;
    float next1 = bin, next2 = _distribs[z][0];
    float pred = 0.f;
    float dist = 0.f;
    while(i < _nSamples || j < _nSamples) {
        float dij = j - i - alpha;
        if(next1 < next2) {
            float dt = next1 - pred;
            dist += dt * dij*dij;
            pred = next1;
            ++ i;
            next1 += bin;
        } else {
            float dt = next2 - pred;
            dist += dt * dij*dij;
            pred = next2;
            ++ j;
            if(j < _nSamples) next2 += _distribs[z][j];
            else next2 += totLength;
        }
    }
    return std::sqrt(12.f * dist / (totLength * _nSamples * _nSamples));
}

///////////
//
//  GETTERS
//
///////////

const std::vector<glm::vec2>& ObjectiveFunctions::getPoints() const {
    return _points;
}

const std::vector<std::vector<int>>& ObjectiveFunctions::getLinks() const
{
    return _cLinks;
}

std::tuple<const std::vector<glm::vec2> &, const std::vector<std::vector<int>> &, const std::vector<std::vector<int>> &> ObjectiveFunctions::getGraph() const
{
    return {_points, _cLinks, _originalLinks};
}

std::pair<const std::vector<glm::vec2> &, const std::vector<std::vector<int>> &> ObjectiveFunctions::getCycle() const
{
    return {_points, _cLinks};
}

const std::vector<Shape>& ObjectiveFunctions::getZones() const {
    return _zones;
}

float ObjectiveFunctions::getScore() const {
    return _score;
}

float ObjectiveFunctions::getArea() const {
    return _border._area;
}

int ObjectiveFunctions::getStrokeColor(int i) const {
    return _zones[_zone[i]]._printColor;
}

const Shape& ObjectiveFunctions::getBorder() const
{
    return _border;
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
