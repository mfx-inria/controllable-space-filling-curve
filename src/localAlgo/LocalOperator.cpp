//
// Created by adrien_bedel on 11/12/19.
//

#include "localAlgo/LocalOperator.h"

#include "LBFGS/cvt.hpp"
#include "tools/Matching.h"
#include "tools/Random.h"

const std::vector<std::vector<std::pair<int, int>>> LocalOperator::_segments = {
        {{0, 1}, {2, 3}, {4, 5}},           // flip curr
        {{0, 3}, {2, 5}, {1, 4}},           // flip cand
        {{0, 1}, {1, 2}, {3, 4}, {5, 6}},   // transpose curr
        {{0, 3}, {2, 6}, {1, 4}, {1, 5}},   // transpose cand
        {{0, 1}, {1, 2}, {3, 4}},           // cross curr
        {{0, 2}, {1, 3}, {1, 4}},           // cross cand
        {{0, 1}, {2, 3}},                   // zigzag curr
        {{0, 2}, {1, 3}}                    // zigzag cand
};

LocalOperator::LocalOperator(std::vector<Shape> &zones, std::vector<Shape> &&strokeZones, Shape &border, int layerIndex)
        : ObjectiveFunctions(std::move(zones), std::move(border), layerIndex), _strokeZones(strokeZones), _gen(std::mt19937(Globals::_seed)) {}

void LocalOperator::setGraph(Graph &graph) {
    Matching m(_border, graph);
    _points = std::move(m._points);
    computeZone();
    _originalLinks = std::move(m._originalLinks);
    _cLinks = std::move(m._cLinks);
}

void LocalOperator::updateState(bool state) {
    _isEnd = state;
}

///////////
//
//  COND PTRS
//
///////////

inline bool LocalOperator::switchConditions(float score_dif) {
    if(score_dif <= 0.f) return true;
    if(_isEnd || score_dif > 0.4f) return false;
    if(_iter % 400 == 0) return true;
    return UniformReal<float>(0, 1.f + score_dif * _points.size())(_gen) < 1.f;
}

////////////////////////
//
//     GENERAL OP
//
///////////////////////

bool LocalOperator::isLinked(const std::vector<int> &links, int node) {
    return std::find(links.begin(), links.end(), node) != links.end();
}

bool LocalOperator::checkPaperOp(int currentNode) {
    std::vector<int> firstDir = { currentNode, _cLinks[currentNode][0] };
    std::vector<int> secondDir = { currentNode, _cLinks[currentNode][1] };

    for (int i = 0; i < 2; i++) {
        if(_cLinks[firstDir.back()][0] == firstDir[firstDir.size()-2]) firstDir.push_back(_cLinks[firstDir.back()][1]);
        else firstDir.push_back(_cLinks[firstDir.back()][0]);

        if(_cLinks[secondDir.back()][0] == secondDir[secondDir.size()-2]) secondDir.push_back(_cLinks[secondDir.back()][1]);
        else secondDir.push_back(_cLinks[secondDir.back()][0]);
    }

    std::array<std::pair<float, std::vector<int>>, 8> results;

    results[0] = checkFlip(firstDir);
    results[1] = checkFlip(secondDir);
    results[2] = checkTranspose(firstDir);
    results[3] = checkTranspose(secondDir);
    results[4] = checkCross(firstDir);
    results[5] = checkCross(secondDir);
    results[6] = checkZigZag(firstDir);
    results[7] = checkZigZag(secondDir);

    int idx = std::distance(results.begin(), std::min_element(results.begin(), results.end(), [](auto &a, auto &b){
        return a.first < b.first;
    }));

    if(results[idx].second.empty()) return false;

    if(!_isEnd && _iter % 400 == 0) {
        idx = _gen()&7u;
        while(results[idx].second.empty()) idx = _gen()&7u;
    }

    if(switchConditions(results[idx].first)) {
        ++ _iter;
        int dist = idx / 2;
        applyShift(results[idx].second, _segments[2*dist], _segments[2*dist+1]);
        if(dist == 3) {
            if(results[idx].second[1] != 0 && results[idx].second[2] != 0) std::swap(_index[results[idx].second[1]], _index[results[idx].second[2]]);
            else computeIndex(0, 0);
        } else {
            auto [start, end] = std::minmax_element(results[idx].second.begin(), results[idx].second.end(), [&](int a, int b) {
                return _index[a] < _index[b];
            });
            computeIndex(*start, *end);
        }
        return true;
    }
    return false;
}

void LocalOperator::computeIndex(int start = 0, int end = 0) {
    int prev = start, next;
    if(start == 0) {
        _index.resize(_points.size());
        _index[0] = end = 0;
        next = _cLinks[0][0];
    } else next = _index[_cLinks[start][0]] < _index[start] ? _cLinks[start][1] : _cLinks[start][0];
    int i = _index[start];
    while(next != end) {
        _index[next] = ++i;
        start = next;
        next = _cLinks[next][0] == prev ? _cLinks[next][1] : _cLinks[next][0];
        prev = start;
    }
}

void LocalOperator::startShuffling(int multiplier, int champ) {
    _gen = std::mt19937(Globals::_seed + champ);
    computeIndex();
    std::vector<int> order(_points.size());
    for(int i = 0; i < (int) order.size(); ++i) order[i] = i;
    int N = order.size();
    UniformInt<int> dis(0, N-1);
    bool pred_isEnd = _isEnd;
    for(int s = 0; s < 2; ++s) {
        if(s) updateState(true);
        int nIter = multiplier * _points.size() * (1 + pred_isEnd);
        for(int i = 0; i < nIter; i++) {
            int random0 = dis(_gen);
            int random = order[random0];

            if(checkPaperOp(random)) N = order.size();
            else {
                if(--N == 0) break;
                std::swap(order[random0], order[N]);
            }
        }
        if(N == 0) break;
    }

    updateState(pred_isEnd);
}

void LocalOperator::optimize()
{
    // Create the vector of variables to optimize
    Eigen::VectorXd x(2 * _points.size());
    int prev = _cLinks[0][0];
    int next = 0;
    for(int i = 0; i < (int) _points.size(); ++i) {
        x(2*i) = _points[next].x;
        x(2*i+1) = _points[next].y;
        prev = _cLinks[next][0] == prev ? _cLinks[next][1] : _cLinks[next][0];
        std::swap(next, prev);
    }

    // Smoother
    Smoother smoother(&_border, &_zones, &_strokeZones, _vecArea, _layerIndex);
    smoother.optimize(x);

    // Update Link
    PointCVT cvt(&_border._points, &_border._holes);
    cvt.updateLink(x, _points, _originalLinks, _cLinks);
    computeZone();

    // Compute radii for final
    smoother.computeRadii(x, _final.first);
    _final.second.clear();
    _final.second.reserve(_final.first.size());

    // get final for printing
    for(const glm::vec3 &p : _final.first) {
        glm::vec2 v(p.x, p.y);
        int col = 0;
        for(const Shape &zone : _strokeZones) if (zone.isInside(v)) {
                col = zone._printColor;
                break;
            }

        if (col == 0)
            _final.second.emplace_back(0, 0, 0);
        else
            _final.second.emplace_back((col & 0xff) / 255.f, ((col >> 8) & 0xff) / 255.f, ((col >> 16) & 0xff) / 255.f);

    }

    // deduce values for points that failed
    int N = _final.first.size();
    for (int i = 0; i < N; i++)
    {
        if (_final.first[i].z < 1e-4 || _final.first[i].z > 1.f)
        {
            int past = 1;
            while (_final.first[(i + N - past) % N].z < 1e-4 || _final.first[(i + N - past) % N].z > 1.f && -past < N)
                past--;
            int future = 1;
            while (_final.first[(i + future) % N].z < 1e-4 || _final.first[(i + future) % N].z > 1.f && future < N)
                future++;
            if (future >= N || -past >= N)
                continue;
            _final.first[i].z = (_final.first[(i + future) % N].z + _final.first[(i + N - past) % N].z) / 2.f;
        }
        if (glm::length(_final.second[i]) < 1e-4)
        {
            int past = 1;
            while (glm::length(_final.second[(i + N - past) % N]) < 1e-4 && -past < N)
                past--;
            int future = 1;
            while (glm::length(_final.second[(i + future) % N]) < 1e-4 && future < N)
                future++;
            if (future >= N || -past >= N)
                continue;
            _final.second[i] = (_final.second[(i + future) % N] + _final.second[(i + N - past) % N]) / 2.f;
        }
    }
    for (auto &col : _final.second)
        col = col / (col.x + col.y + col.z);
}

////////////////////////
//
//     FLIP
//
///////////////////////

std::pair<float, std::vector<int>> LocalOperator::checkFlip(const std::vector<int> &nodes) {
    std::pair<float, std::vector<int>> ans = { std::numeric_limits<float>::max(), {} };
    std::vector<int> tmp_nodes(nodes.begin(), nodes.end());

    if (!isLinked(_originalLinks[nodes[0]], nodes[3]))
        return ans;

    std::vector<int> link2;
    for (int secondLink : _originalLinks[nodes[2]])
        if(secondLink != nodes[0] && secondLink != nodes[1] && secondLink != nodes[3])
            link2.push_back(secondLink);
    for (int firstLink : _originalLinks[nodes[1]])
        if(firstLink != nodes[0] && firstLink != nodes[2] && firstLink != nodes[3])
            for (int secondLink : link2)
                if(isLinked(_cLinks[firstLink], secondLink)) {
                    tmp_nodes.push_back(firstLink);
                    tmp_nodes.push_back(secondLink);
                    float score = checkDirection(tmp_nodes, _segments[0], _segments[1]);
                    if(score < ans.first) ans = { score, tmp_nodes };
                    tmp_nodes.resize(4);
                }
    return ans;
}

////////////////////////
//
//     TRANSPOSE
//
///////////////////////

std::pair<float, std::vector<int>> LocalOperator::checkTranspose(const std::vector<int> &nodes) {
    std::pair<float, std::vector<int>> ans = { std::numeric_limits<float>::max(), {} };
    std::vector<int> tmp_nodes(nodes.begin(), nodes.begin()+3);

    int inds[3] = { _index[nodes[0]], _index[nodes[1]], _index[nodes[2]] };

    std::vector<int> link1;
    for (int v1 : _originalLinks[nodes[1]])
        if(v1 != nodes[0] && v1 != nodes[2])
            link1.push_back(v1);

    std::vector<std::pair<int, int>> first_pairs;
    for(int v0 : _originalLinks[nodes[0]])
        if(v0 != nodes[2] && !isLinked(_cLinks[v0], nodes[0]))
            for(int v1 : link1)
                if(isLinked(_cLinks[v0], v1))
                    first_pairs.emplace_back(v0, v1);

    if(first_pairs.empty()) return ans;

    for(int v2 : _originalLinks[nodes[2]])
        if(v2 != nodes[0] && !isLinked(_cLinks[v2], nodes[2]))
            for(int v1 : link1)
                if(isLinked(_cLinks[v1], v2))
                    for(const std::pair<int, int> &w : first_pairs)
                        if(v1 != w.first && v1 != w.second && v2 != w.first && v2 != w.second) {
                            bool ok = false;
                            if(((inds[0]+1) % (int) _points.size()) == inds[1]) { // ... --> 0 --> 1
                                int i = (inds[0] - _index[v1] + _points.size()) % _points.size();
                                ok = i < ((inds[0] - _index[v2] + (int) _points.size()) % (int) _points.size())
                                     && i < ((inds[0] - _index[w.first] +  (int) _points.size()) % (int) _points.size())
                                     && i < ((inds[0] - _index[w.second] + (int) _points.size()) % (int) _points.size());
                            } else { // ... <-- 0 <-- 1
                                int i = (_index[v1] - inds[0] + _points.size()) % _points.size();
                                ok = i < ((_index[v2] - inds[0] + (int) _points.size()) % (int) _points.size())
                                     && i < ((_index[w.first] - inds[0] +  (int) _points.size()) % (int) _points.size())
                                     && i < ((_index[w.second] - inds[0] + (int) _points.size()) % (int) _points.size());
                            }
                            if(!ok) {
                                if(((inds[2]+1) % (int) _points.size()) == inds[1]) { // ... --> 2 --> 1
                                    int i = (inds[2] - _index[w.second] + _points.size()) % _points.size();
                                    ok = i < ((inds[2] - _index[v2] + (int) _points.size()) % (int) _points.size())
                                         && i < ((inds[2] - _index[w.first] + (int) _points.size()) % (int) _points.size())
                                         && i < ((inds[2] - _index[v1] +      (int) _points.size()) % (int) _points.size());
                                } else { // ... <-- 2 <-- 1
                                    int i = (_index[w.second] - inds[2] + _points.size()) % _points.size();
                                    ok = i < ((_index[v2] - inds[2] + (int) _points.size()) % (int) _points.size())
                                         && i < ((_index[w.first] - inds[2] + (int) _points.size()) % (int) _points.size())
                                         && i < ((_index[v1] - inds[2] +      (int) _points.size()) % (int) _points.size());
                                }
                            }
                            if(ok) {
                                tmp_nodes.push_back(w.first);
                                tmp_nodes.push_back(w.second);
                                tmp_nodes.push_back(v1);
                                tmp_nodes.push_back(v2);
                                float score = checkDirection(tmp_nodes, _segments[2], _segments[3]);
                                if(score < ans.first) ans = { score, tmp_nodes };
                                tmp_nodes.resize(3);
                            }
                        }

    return ans;
}

////////////////////////
//
//     CROSS
//
///////////////////////

std::pair<float, std::vector<int>> LocalOperator::checkCross(const std::vector<int> &nodes)
{
    std::pair<float, std::vector<int>> ans = { std::numeric_limits<float>::max(), {} };
    std::vector<int> tmp_nodes(nodes.begin(), nodes.begin()+3);

    if (!isLinked(_originalLinks[nodes[0]], nodes[2]))
        return ans;

    const std::vector<int> &link1 = _originalLinks[nodes[1]];
    for(int i = 1; i < (int) link1.size(); ++i) if(link1[i] != nodes[0] && link1[i] != nodes[2])
            for(int j = 0; j < i; ++j)
                if(isLinked(_cLinks[link1[i]], link1[j]) && link1[j] != nodes[0] && link1[j] != nodes[2]) {
                    tmp_nodes.push_back(link1[i]);
                    tmp_nodes.push_back(link1[j]);
                    float score = checkDirection(tmp_nodes, _segments[4], _segments[5]);
                    if(score < ans.first) ans = { score, tmp_nodes };
                    tmp_nodes.resize(3);
                }
    return ans;
}

////////////////////////
//
//     ZIGZAG
//
///////////////////////

std::pair<float, std::vector<int>> LocalOperator::checkZigZag(const std::vector<int> &nodes)
{
    if (isLinked(_originalLinks[nodes[0]], nodes[2]) &&
        isLinked(_originalLinks[nodes[1]], nodes[3]))
        return { checkDirection(nodes, _segments[6], _segments[7]), nodes };
    return { std::numeric_limits<float>::max(), {} };
}

////////////////////////
//
//     GETTERS
//
///////////////////////

const std::pair<std::vector<glm::vec3>, std::vector<glm::vec3>>& LocalOperator::getFinal() const {
    return _final;
}

const std::vector<Shape>& LocalOperator::getStrokeZones() const {
    return _strokeZones;
}