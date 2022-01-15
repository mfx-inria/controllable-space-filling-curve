//
// Created by bedela on 01/07/2020.
//

#include "tools/Matching.h"

#include <algorithm>
#include <set>
#include <iostream>
#include <fstream>

const bool printFuseStats = false;

Tree::Tree(int nodeIdx, bool needEdge, std::shared_ptr<Tree> parent = nullptr)
        : _parent(parent), _nodeIdx(nodeIdx), _needEdge(needEdge)
{ }

bool Tree::operator==(int nodeIdx) const
{
    return (this->_nodeIdx == nodeIdx);
}

Matching::Matching(const Shape &border, Graph &graph) {
    _points = std::move(graph._points);
    _originalLinks = std::move(graph._originalLinks);
    _cLinks.resize(_originalLinks.size());

    if(checkInit())
        match();
    else {
        std::cout << "FAAAAILLL TRIANGULATE" << std::endl;
        _succes = false;
    }
    if(!_succes) return;
    for (const std::vector<int> &cell : graph._cells) {
        if(cell.size() < 3) continue;

        // Compute the center of the cell
        glm::vec2 center = Graph::getCellCenter(cell, _points);
        if(!border.isInside(center)) continue;

        // connect cell vertices to the center of the cell
        int lastIdx = _points.size();
        _originalLinks.emplace_back();
        int count = 0;
        for(int idx : cell) {
            bool bad = false; // bad will be true if the new edge intersects another
            for(int i : cell) if(i != idx) {
                for(int j : _originalLinks[i])
                    if(j < i && j != idx && Globals::intersect(_points[idx], _points[i], center, _points[j])) {
                        bad = true;
                        break;
                    }
                if(bad) break;
            }
            if(bad) continue;
            _originalLinks[lastIdx].push_back(idx);
            _originalLinks[idx].push_back(lastIdx);
            if(!_cLinks[idx].empty()) ++ count;
        }
        if(count < 3) {
            for(int i : _originalLinks.back()) _originalLinks[i].pop_back();
            _originalLinks.pop_back();
            continue;
        }
        _points.push_back(center);
        _cLinks.emplace_back();

        // Connect the center to the rest of the path
        int A = -1, B;
        int tmp = lastIdx;
        std::set<int> seen = {tmp};
        while (A == -1) {
            // A = B       A --- B
            //  \ /   ===>  \\ //
            //   T            T
            for (int i = 1; i < _originalLinks[tmp].size(); i++) {
                int a = _originalLinks[tmp][i];
                for(int j = 0; j < i; ++j) {
                    int b = _originalLinks[tmp][j];
                    if(isLinked(_cLinks[a], b)) {
                        A = a;
                        B = b;
                    }
                }
                if(A != -1) break;
            }

            if (A == -1) {  // (A, B) not found
                int C = -1, D;
                //    C             C = new T
                //  // \\         /   \
                // B --- D  ==>  B --- D
                //  \   /         \\ //
                //    T             T
                for (int b : _originalLinks[tmp]) {
                    if(C != -1) break;
                    for (int c : _cLinks[b]) if(!seen.count(c)) {
                        int d = _cLinks[c][0] == b ? _cLinks[c][1] : _cLinks[c][0];
                        bool linkedB = false, linkedTmp = false;
                        for(int link : _originalLinks[d]) {
                            linkedB |= link == b;
                            linkedTmp |= link == tmp;
                        }
                        if (linkedB && linkedTmp) {
                            B = b;
                            C = c;
                            D = d;
                            break;
                        }
                    }
                }
                if (C != -1) {
                    removeLink(B, C);
                    removeLink(C, D);
                    createLink(B, tmp);
                    createLink(tmp, D);
                    tmp = C;
                    seen.insert(tmp);
                } else {
                    std::cerr << "fail linking cells" << std::endl;
                    _succes = false;
                    return;
                }
            } else { // (A, B) found
                removeLink(A, B);
                createLink(A, tmp);
                createLink(tmp, B);
                ++ _nbConnectedPoints;
            }
        }
    }
    initUnion();
    fuseIslands();
    removeUnused(border);
}

void Matching::initUnion() {
    _union = Union(_points.size());
    for(int i = 0; i < _points.size(); ++i)
        for(int j : _cLinks[i])
            _union.merge(i, j);
}

std::vector<int> Matching::getPath(int start, int end) {
    if(std::find(_originalLinks[start].begin(), _originalLinks[start].end(), end) != _originalLinks[start].end())
        return {start, end};
    std::vector<int>::iterator it = _originalLinks[start].begin();
    while(true)  {
        std::vector<int> path = { start };
        it = std::find_if(it, _originalLinks[start].end(), [&](int link) { return _cLinks[link].empty(); });
        int prev = start;
        int node = *it;
        while(node != -1) {
            path.push_back(node);
            int next = -1;
            for(int i : _originalLinks[node]) if(i != prev) {
                if(_cLinks[i].empty()) next = i;
                else if(i == end) {
                    path.push_back(end);
                    return path;
                }
            }
            prev = node;
            node = next;
        }
    }
}

void Matching::removeUnused(const Shape &border) {
    std::vector<int> newIndex(_points.size(), -1);
    int newSize = 0;
    for(int i = 0; i < newIndex.size(); ++i) {
        if(_cLinks[i].empty()) {
            for(int j : _originalLinks[i]) if(!_cLinks[j].empty()) {
                std::vector<int> js = getIdxs(i, j);
                js.push_back(j);
                for(int k = 1; k < js.size(); ++k)
                    for(int l = 0; l < k; ++l)
                        if(std::find(_originalLinks[js[k]].begin(), _originalLinks[js[k]].end(), js[l]) == _originalLinks[js[k]].end()) {
                            bool bad = false;
                            glm::vec2 w = _points[js[l]] - _points[js[k]];
                            w *= 1e-5f / glm::length(w);
                            glm::vec2 a = _points[js[k]] + w, b = _points[js[l]] - w;
                            for(int m : js) if(!bad)
                                for(int n : _originalLinks[m]) if(!_cLinks[n].empty())
                                    if(Globals::intersect(a, _points[m], b, _points[n])) {
                                        bad = true;
                                        break;
                                    }
                            if(bad) continue;
                            for(int m = 1; m < border._points.size(); ++m)
                                if(Globals::intersect(_points[js[l]], border._points[m-1], _points[js[k]], border._points[m])) {
                                    bad = true;
                                    break;
                                }
                            if(bad) continue;
                            for(const std::vector<glm::vec2> &hole : border._holes) {
                                for(int m = 1; m < hole.size(); ++m)
                                    if(Globals::intersect(_points[js[l]], hole[m-1], _points[js[k]], hole[m])) {
                                        bad = true;
                                        break;
                                    }
                                if(bad) break;
                            }
                            if(bad) continue;
                            _originalLinks[js[k]].push_back(js[l]);
                            _originalLinks[js[l]].push_back(js[k]);
                        }
                break;
            }
        } else newIndex[i] = newSize++;
    }
    for(int i = 0; i < newIndex.size(); ++i) {
        int k = newIndex[i];
        if(k == -1) continue;
        for(int &j : _cLinks[i]) j = newIndex[j];
        int s = 0;
        for(int j : _originalLinks[i]) if(newIndex[j] != -1)
            _originalLinks[i][s++] = newIndex[j]; 
        _originalLinks[i].resize(s);
        if(k < i) {
            _points[k] = _points[i];
            _originalLinks[k] = std::move(_originalLinks[i]);
            _cLinks[k] = std::move(_cLinks[i]);
        }
    }
    _points.resize(newSize);
    _originalLinks.resize(newSize);
    _cLinks.resize(newSize);
}

std::vector<int> Matching::getIdxs(int node, int parent) {
    if(!_cLinks[node].empty()) return {node};
    std::vector<int> ans;
    int prev = parent;
    while(node != -1) {
        int next = -1;
        for(int i : _originalLinks[node]) if(i != parent && i != prev) {
            if(_cLinks[i].empty()) next = i;
            else if(std::find(ans.begin(), ans.end(), i) == ans.end()) ans.push_back(i);
        }
        prev = node;
        node = next;
    }
    return ans;
}

void Matching::fuseIslands() {
    if(_union.fullyMerged(_nbConnectedPoints)) return;
    int flip = 0;
    int N = _points.size();
    // i ==== k       i ---- k
    // |      |  ==>  ||    ||
    // j ==== l       j ---- l
    for(int i = 0; i < N; ++i) if(!_cLinks[i].empty()) {
        for(int j0 : _originalLinks[i]) for(int j : getIdxs(j0, i)) {
            if(!_union.same(i, j)) {
                bool found = false;
                for(int k : _cLinks[i]) if(!found) {
                    for(int l0 : _originalLinks[k]) if(!found) for(int l : getIdxs(l0, k)) {
                        if(l == _cLinks[j][0] || l == _cLinks[j][1]) {
                            std::vector<int> path = getPath(i, j), path2 = getPath(l, k);
                            _nbConnectedPoints += path.size() + path2.size() - 4;
                            removeLink(i, k);
                            removeLink(j, l);
                            for (int inc = 1; inc < path.size(); ++inc) {
                                _union.merge(path[inc], path[inc - 1]);
                                createLink(path[inc], path[inc - 1]);
                            }
                            for (int inc = 1; inc < path2.size(); ++inc) {
                                _union.merge(path2[inc], path2[inc - 1]);
                                createLink(path2[inc], path2[inc - 1]);
                            }
                            ++ flip;
                            if(_union.fullyMerged(_nbConnectedPoints))
                                {
                                    if(printFuseStats) std::cerr << "fuseIslands:  " << flip << " flips,  0 segment doubled" << std::endl;
                                    return;
                                }
                            found = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    // i        a -- b
    // |  ==>  ||    ||
    // j        c -- d
    int doubled = 0;
    for(int i = 0; i < N; ++i) if(!_cLinks[i].empty()) {
        for(int j : _originalLinks[i]) if(j < N && !_union.same(i, j) && !_cLinks[j].empty()) {
                ++ doubled;
                // Maybe we will need to optimize the position of new points
                glm::vec2 a = _points[i];
                glm::vec2 b = _points[i];
                glm::vec2 c = _points[j];
                glm::vec2 d = _points[j];
                glm::vec2 u = _points[_cLinks[i][0]] - _points[i];
                glm::vec2 v = _points[_cLinks[i][1]] - _points[i];
                if(glm::dot(u / glm::length(u) - v / glm::length(v), _points[j] - _points[i]) > 0.f) a += 2e-2f * u;
                else b += 2e-2f * v;
                u = _points[_cLinks[j][0]] - _points[j];
                v = _points[_cLinks[j][1]] - _points[j];
                if(glm::dot(u / glm::length(u) - v / glm::length(v), _points[i] - _points[j]) > 0.f) c += 2e-2f * u;
                else d += 2e-2f * v;
                if(Globals::intersect(a, b, c, d)) {
                    std::swap(a, b);
                    std::swap(_cLinks[i][0], _cLinks[i][1]);
                }
                _points[i] = a;
                _points[j] = c;
                int k = _points.size();
                _points.push_back(b);
                _points.push_back(d);
                _originalLinks.emplace_back();
                _originalLinks.emplace_back();
                _cLinks.emplace_back();
                _cLinks.emplace_back();
                int o = _cLinks[i][1];
                removeLink(i, o);
                createLink(k, o);
                o = _cLinks[j][1];
                removeLink(j, o);
                createLink(k+1, o);
                createLink(i, j);
                createLink(k, k+1);
                for(int x = 0; x < _originalLinks[i].size();) {
                    int l = _originalLinks[i][x];
                    const glm::vec2 &e = _points[l];
                    if(Globals::intersect(a, b, e, d)) {
                        _originalLinks[i][x] = _originalLinks[i].back();
                        _originalLinks[i].pop_back();
                        int y = 0;
                        while(_originalLinks[l][y] != i) ++y;
                        _originalLinks[l][y] = _originalLinks[l].back();
                        _originalLinks[l].pop_back();
                    } else ++x;
                    if(!Globals::intersect(b, a, e, c)) {
                        _originalLinks[k].push_back(l);
                        _originalLinks[l].push_back(k);
                    }
                }
                for(int x = 0; x < _originalLinks[j].size();) {
                    int l = _originalLinks[j][x];
                    const glm::vec2 &e = _points[l];
                    if(Globals::intersect(c, b, e, d)) {
                        _originalLinks[j][x] = _originalLinks[j].back();
                        _originalLinks[j].pop_back();
                        int y = 0;
                        while(_originalLinks[l][y] != j) ++y;
                        _originalLinks[l][y] = _originalLinks[l].back();
                        _originalLinks[l].pop_back();
                    } else ++x;
                    if(!Globals::intersect(d, a, e, c)) {
                        _originalLinks[k+1].push_back(l);
                        _originalLinks[l].push_back(k+1);
                    }
                }
                _originalLinks[i].push_back(k);
                _originalLinks[k].push_back(i);
                _originalLinks[j].push_back(k+1);
                _originalLinks[k+1].push_back(j);
                _originalLinks[k].push_back(k+1);
                _originalLinks[k+1].push_back(k);
                _union.merge(i, j);
                if(_union.fullyMerged(_nbConnectedPoints)) {
                    if(printFuseStats) std::cerr << "fuseIslands:  " << flip << " flips,  " << doubled << " segments doubled" << std::endl;
                    return;
                }
            }
    }
    std::cout << "failed fusing islands" << std::endl;
    _succes = false;
}

void Matching::match()
{
    augment();
    if (_succes) {
		std::ofstream file("video/matching.txt");
		file << "POINTS " << _points.size() << '\n';
		for(const glm::vec2 &p : _points)
			file << p.x << ' ' << p.y << '\n';
		size_t ne = 0, nm = 0;
		for(const auto link : _originalLinks) ne += link.size();
		for(const auto link : _cLinks) nm += link.size();
		ne /= 2; nm /= 2;
		file << "EDGE " << ne << '\n';
		for(int i = 0; i < (int) _originalLinks.size(); ++i)
			for(int j : _originalLinks[i]) if(i < j)
				file << i << ' ' << j << '\n';
		file << "MATCH " << nm << '\n';
		for(int i = 0; i < (int) _cLinks.size(); ++i)
			for(int j : _cLinks[i]) if(i < j)
				file << i << ' ' << j << '\n';
		file.close();
		// ---------- //
        switchLink();
		// ---------- //
		file.open("video/path.txt");
		file << "POINTS " << _points.size() << '\n';
		for(const glm::vec2 &p : _points)
			file << p.x << ' ' << p.y << '\n';
		ne = 0; nm = 0;
		for(const auto link : _originalLinks) ne += link.size();
		for(const auto link : _cLinks) nm += link.size();
		ne /= 2; nm /= 2;
		file << "EDGE " << ne << '\n';
		for(int i = 0; i < (int) _originalLinks.size(); ++i)
			for(int j : _originalLinks[i]) if(i < j)
				file << i << ' ' << j << '\n';
		file << "PATH " << nm << '\n';
		for(int i = 0; i < (int) _cLinks.size(); ++i)
			for(int j : _cLinks[i]) if(i < j)
				file << i << ' ' << j << '\n';
		file.close();
	} else
        std::cout << "FAAAAILLL AUGMENTINGG" << std::endl;
}

bool Matching::isLinked(const std::vector<int> &links, int node)
{
    return std::find(links.begin(), links.end(), node) != links.end();
}

void Matching::switchLink()
{
    _nbConnectedPoints = 0;
    for (int i = 0; i < _cLinks.size(); i++)
    {
        if (_originalLinks[i].size() == 2)
        {
            if (_cLinks[i].size() == 2)
                _cLinks[i].clear();
            else {
                _cLinks[i] = _originalLinks[i];
                ++ _nbConnectedPoints;
            }
        }
        else
        {
            int j = _cLinks[i][0];
            _cLinks[i].clear();
            for (int link : _originalLinks[i])
                if(link != j)
                    _cLinks[i].push_back(link);
            ++ _nbConnectedPoints;
        }
    }
}

void Matching::augment()
{
	std::vector<int> order(_points.size());
	for(int i = 0; i < (int) order.size(); ++i) {
		order[i] = i;
		std::random_shuffle(_originalLinks[i].begin(), _originalLinks[i].end());
	}
	std::random_shuffle(order.begin(), order.end());
    for (int i : order)
        if (_cLinks[i].empty() && _originalLinks[i].size() == 3)
            augmentPath(i);
    print();
}

// distance between node and the first parent with index idx
// if no such parent exists, return -1
int Matching::distInTree(std::shared_ptr<Tree> node, int idx)
{
    int count = 0;
    while (node != nullptr && node->_nodeIdx != idx)
    {
        ++ count;
        node = node->_parent;
    }
    if (node != nullptr)
        return count;
    return -1;
}

int Matching::getIdx(int node, int parent)
{
    while (_originalLinks[node].size() == 2)
    {
        auto it = std::find_if(_originalLinks[node].begin(), _originalLinks[node].end(), [parent](int n){ return parent != n; });
        parent = node;
        node = *it;
    }
    return node;
}

void Matching::augmentPath(int idx)
{
    bool found = false;
    std::shared_ptr<Tree> endPoint = nullptr;
    std::shared_ptr<Tree> root(std::make_shared<Tree>(idx, false));

    for (int oriL : _originalLinks[idx])
        root->_childrens.push_back(std::make_shared<Tree>(getIdx(oriL, root->_nodeIdx), true, root));
    std::vector<std::shared_ptr<Tree>> childs = root->_childrens;
    std::vector<std::shared_ptr<Tree>> tmpChilds;
    tmpChilds.clear();

    while (!found)
    {
        if (childs.empty())
        {
            std::cout << "not found" << std::endl;
            found = true;
        }
        for (int i = 0; i < childs.size() && !found; i++)
        {
            std::shared_ptr<Tree> child = childs[i];
            idx = child->_nodeIdx;
            int dist = distInTree(child->_parent, idx);
            if (dist % 2 == 0)
            {
                std::vector<int> blossomNodes;
                std::shared_ptr<Tree> node = child->_parent;
                while (node->_nodeIdx != idx)
                {
                    blossomNodes.push_back(node->_nodeIdx);
                    node = node->_parent;
                }
                childs[i] = node;
                childs[i]->_childrens.clear();

                for (int k = 0; k < blossomNodes.size(); k++)
                {
                    int bNode = blossomNodes[k];
                    auto it = std::find_if(_originalLinks[bNode].begin(), _originalLinks[bNode].end(), [&blossomNodes, idx](int leaf) {
                        return leaf != idx && std::find(blossomNodes.begin(), blossomNodes.end(), leaf) == blossomNodes.end();
                    });
                    if (it != _originalLinks[bNode].end())
                    {
                        bool outNEdge = isLinked(_cLinks[bNode], *it);
                        std::shared_ptr<Tree> goAlong;
                        if ((k + outNEdge) % 2 == 1)
                        {
                            childs[i]->_childrens.push_back(std::make_shared<Tree>(getIdx(blossomNodes[0], childs[i]->_nodeIdx), true, childs[i]));
                            goAlong = childs[i]->_childrens.back();
                            bool edge = false;
                            for (int p = 1; p < k; p++)
                            {
                                goAlong->_childrens.push_back(std::make_shared<Tree>(getIdx(blossomNodes[p], goAlong->_nodeIdx), edge, goAlong));
                                goAlong = goAlong->_childrens[0];
                                edge = !edge;
                            }
                        }
                        else
                        {
                            childs[i]->_childrens.push_back(std::make_shared<Tree>(getIdx(blossomNodes.back(), childs[i]->_nodeIdx), true, childs[i]));
                            goAlong = childs[i]->_childrens.back();
                            bool edge = false;
                            for (int p = blossomNodes.size() - 2; p > k; p--)
                            {
                                goAlong->_childrens.push_back(std::make_shared<Tree>(getIdx(blossomNodes[p], goAlong->_nodeIdx), edge, goAlong));
                                goAlong = goAlong->_childrens[0];
                                edge = !edge;
                            }
                        }
                        goAlong->_childrens.push_back(std::make_shared<Tree>(getIdx(blossomNodes[k], goAlong->_nodeIdx), outNEdge, goAlong));
                        goAlong = goAlong->_childrens[0];
                        goAlong->_childrens.push_back(std::make_shared<Tree>(getIdx(*it, goAlong->_nodeIdx), true, goAlong));
                    }
                }
            }
            else if (dist == -1 && _cLinks[idx].empty())
            {
                endPoint = child;
                found = true;
            }
            else if (dist == -1)
            {
                if (child->_needEdge)
                    childs[i]->_childrens.push_back(std::make_shared<Tree>(getIdx(_cLinks[idx][0], childs[i]->_nodeIdx), false, childs[i]));
                else
                    for (int oriL : _originalLinks[idx])
                        if (oriL != _cLinks[idx][0])
                            childs[i]->_childrens.push_back(std::make_shared<Tree>(getIdx(oriL, childs[i]->_nodeIdx), true, childs[i]));
            }
        }
        for (const auto &child : childs)
            tmpChilds.insert(tmpChilds.end(), child->_childrens.begin(), child->_childrens.end());
        childs.clear();
        for (const auto &tmpChild : tmpChilds)
            childs.push_back(tmpChild);
        for (int i = 0; i < childs.size(); i++)
            while (!childs[i]->_childrens.empty())
                childs[i] = childs[i]->_childrens[0];
        tmpChilds.clear();
        for (const auto &child : childs)
            if (std::find_if(tmpChilds.begin(), tmpChilds.end(), [child](auto &node){ return (node->_nodeIdx == child->_nodeIdx);}) == tmpChilds.end())
                tmpChilds.push_back(child);
        childs = tmpChilds;
        tmpChilds.clear();
    }

    if (endPoint == nullptr)
    {
        std::cout << "too much" << std::endl;
        return ;
    }
    else
    {
        bool nEdge = true;
        while (endPoint->_parent != nullptr)
        {
            if (!isLinked(_originalLinks[endPoint->_nodeIdx], endPoint->_parent->_nodeIdx))
            {
                int node = -1;
                int loop = 0;
                std::vector<int> path;
                while (node != endPoint->_parent->_nodeIdx)
                {
                    path.clear();
                    path.push_back(endPoint->_nodeIdx);
                    int prev = endPoint->_nodeIdx;
                    auto id = std::find_if(_originalLinks[prev].begin() + loop, _originalLinks[prev].end(), [this, nEdge](int link) { return (this->_originalLinks[link].size() == 2 && ((this->_cLinks[link].empty() && nEdge) || (!this->_cLinks[link].empty() && !nEdge))); });
                    node = *id;
                    path.push_back(node);
                    while (_originalLinks[*id].size() == 2)
                    {
                        id = std::find_if(_originalLinks[node].begin(), _originalLinks[node].end(), [prev](int n) { return prev != n; });
                        prev = node;
                        node = *id;
                        path.push_back(node);
                    }
                    loop++;
                }
                if (nEdge)
                    for (int i = 0; i < path.size() - 1; i++)
                        createLink(path[i], path[i + 1]);
                else
                    for (int i = 0; i < path.size() - 1; i++)
                        removeLink(path[i], path[i + 1]);
            }
            else if (isLinked(_cLinks[endPoint->_nodeIdx], endPoint->_parent->_nodeIdx))
                removeLink(endPoint->_nodeIdx, endPoint->_parent->_nodeIdx);
            else
                createLink(endPoint->_nodeIdx, endPoint->_parent->_nodeIdx);
            endPoint = endPoint->_parent;
            nEdge = !nEdge;
        }
    }
}

void Matching::print()
{
    int zerros = 0;
    for (int i = 0; i < _cLinks.size(); i++)
        if (_cLinks[i].size() != 1 && _originalLinks[i].size() == 3)
            zerros++;

    _succes = (zerros == 0);
    if(!_succes) std::cout << "errors = " << zerros << std::endl;
}

void Matching::createLink(int pointA, int pointB)
{
    _cLinks[pointA].push_back(pointB);
    _cLinks[pointB].push_back(pointA);
}

void Matching::createOriLink(int pointA, int pointB)
{
    _originalLinks[pointA].push_back(pointB);
    _originalLinks[pointB].push_back(pointA);
}

void Matching::removeLink(int pointA, int pointB)
{
    *std::find(_cLinks[pointA].begin(), _cLinks[pointA].end(), pointB) = _cLinks[pointA].back();
    _cLinks[pointA].pop_back();
    *std::find(_cLinks[pointB].begin(), _cLinks[pointB].end(), pointA) = _cLinks[pointB].back();
    _cLinks[pointB].pop_back();
}

void Matching::removeOriLink(int pointA, int pointB)
{
    _originalLinks[pointA].erase(std::find(_originalLinks[pointA].begin(), _originalLinks[pointA].end(), pointB));
    _originalLinks[pointB].erase(std::find(_originalLinks[pointB].begin(), _originalLinks[pointB].end(), pointA));
}

bool Matching::checkInit()
{
    if (_originalLinks.empty())
    {
        return false;
    }
    while (splitFourConnected());
    auto maxLink = _originalLinks[0].size();
    auto minLink = _originalLinks[0].size();
    for (int i = 0; i < _originalLinks.size(); i++)
    {
        maxLink = std::max(_originalLinks[i].size(), maxLink);
        minLink = std::min(_originalLinks[i].size(), minLink);
    }
    if(!(maxLink == 3 && minLink >= 2)) std::cout << maxLink << " " << minLink << std::endl;
    return (maxLink == 3 && minLink >= 2);
}

bool Matching::splitFourConnected()
{
    bool isFourCo = false;
    for (int i = 0; i < _originalLinks.size(); i++)
    {
        if (_originalLinks[i].size() == 4)
        {
            isFourCo = true;
            int tmpJ;
            double minAngle = 10.0;
            for (int j = 0; j < 4; j++)
            {
                auto vec = _points[_originalLinks[i][j]] - _points[i];
                auto vecNext = _points[_originalLinks[i][(j + 1) % 4]] - _points[i];
                if (double angle = acos(glm::dot(vec, vecNext) / (glm::length(vec) * glm::length(vecNext))); angle > 1e-6 && angle < minAngle)
                {
                    minAngle = angle;
                    tmpJ = j;
                }
            }
            auto middleVec = (_points[_originalLinks[i][tmpJ]] - _points[i]) + (_points[_originalLinks[i][(tmpJ + 1) % 4]] - _points[i]);
            _points.push_back(_points[i] + (-middleVec / 5.f));
            _originalLinks.emplace_back();
            _cLinks.emplace_back();
            _points[i] += (middleVec / 5.f);
            int extA = _originalLinks[i][(tmpJ + 2) % 4];
            int extB = _originalLinks[i][(tmpJ + 3) % 4];
            removeOriLink(i, extA);
            removeOriLink(i, extB);

            createOriLink(_points.size() - 1, extA);
            createOriLink(_points.size() - 1, extB);
            createOriLink(_points.size() - 1, i);
        }
    }
    return isFourCo;
}