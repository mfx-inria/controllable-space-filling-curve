//
// Created by bedela on 01/07/2020.
//

#include "initialization/CycleCreator.h"
#include "initialization/Matching.h"
#include "tools/Random.h"

#include <algorithm>
#include <set>
#include <unordered_set>

CycleCreator::CycleCreator(const Shape &shape, Graph &graph) {
	_points = std::move(graph._points);
	_links = std::move(graph._links);
	_cLinks.resize(_links.size());
	for(const std::vector<int> &link : _links) if(link.size() > 3)
		THROW_ERROR(str_format("Graph is not 3-connected! (valence %d found)", link.size()));
	
	perfectMatching();
	switchLink();
	addCenters(shape, graph);
	_union = UnionFind(_points.size());
	for(int i = 0; i < (int) _points.size(); ++i)
		for(int j : _cLinks[i])
			_union.merge(i, j);
	fuseIslands();
	removeUnused(shape);
}

int CycleCreator::getNext(int node, int parent) {
	while(_links[node].size() == 2) {
		parent = _links[node][_links[node][0] == parent];
		std::swap(parent, node);
	}
	return node;
}

void CycleCreator::createLink(int pointA, int pointB) {
	_cLinks[pointA].push_back(pointB);
	_cLinks[pointB].push_back(pointA);
}

void CycleCreator::removeLink(int pointA, int pointB) {
	*std::find(_cLinks[pointA].begin(), _cLinks[pointA].end(), pointB) = _cLinks[pointA].back();
	_cLinks[pointA].pop_back();
	*std::find(_cLinks[pointB].begin(), _cLinks[pointB].end(), pointA) = _cLinks[pointB].back();
	_cLinks[pointB].pop_back();
}

void CycleCreator::perfectMatching() {
	// Reduce graph to 3-connected vertices
	std::vector<int> order;
	order.reserve(_points.size());
	std::mt19937 gen(Globals::_seed);
	for(int i = 0; i < (int) _points.size(); ++i) if(_links[i].size() == 3) {
		order.push_back(i);
		shuffle(_links[i].begin(), _links[i].end(), gen);
	}
	shuffle(order.begin(), order.end(), gen);
	std::vector<int> inv_order(_points.size());
	for(int i = 0; i < (int) order.size(); ++i) inv_order[order[i]] = i;

	// Compute perfect matching
	Matching<std::array<int, 3>> m(order.size());
	for(int i = 0; i < (int) order.size(); ++i)
		for(int j = 0; j < 3; ++j)
			m.G[i+1][j] = inv_order[getNext(_links[order[i]][j], order[i])]+1;
	if(2*m.solve() != (int) order.size()) THROW_ERROR("Matching failed");

	// Back to the original graph
	for(int i = 0; i < (int) order.size(); ++i) {
		int a = order[i];
		if(!_cLinks[a].empty()) continue;
		const int b = order[m.mate[i+1]-1];
		int c = 0;
		while(getNext(_links[a][c], a) != b) ++c;
		c = _links[a][c];
		while(true) {
			createLink(a, c);
			if(c == b) break;
			a = _links[c][_links[c][0] == a];
			std::swap(a, c);
		}
	}
	for(int i = 0; i < (int) _cLinks.size(); i++)
		if(_cLinks[i].size() != 1 && _links[i].size() == 3)
			THROW_ERROR("Matching of 3-connected vertices failed!");
}

void CycleCreator::switchLink() {
	_nbConnectedPoints = 0;
	for(int i = 0; i < (int) _cLinks.size(); i++) {
		if(_links[i].size() == 2) {
			if(_cLinks[i].size() == 2)
				_cLinks[i].clear();
			else {
				_cLinks[i] = _links[i];
				++ _nbConnectedPoints;
			}
		} else {
			const int j = _cLinks[i][0];
			_cLinks[i].clear();
			for(int link : _links[i]) if(link != j)
				_cLinks[i].push_back(link);
			++ _nbConnectedPoints;
		}
	}
}

void CycleCreator::addCenters(const Shape &shape, const Graph &graph) {
	for(const std::vector<int> &cell : graph._cells) {
		if(cell.size() < 3) continue;

		// Compute the center of the cell
		const glm::dvec2 center = Graph::getCellCenter(cell, _points);
		if(!shape.isInside(center)) continue;

		// connect cell vertices to the center of the cell
		const int centerId = _points.size();
		_links.emplace_back();
		int count = 0;
		for(int id : cell) {
			for(int i : cell) if(i != id)
				for(int j : _links[i])
					if(j < i && j != id && Globals::intersect(_points[id], _points[i], center, _points[j]))
						goto for_end;
			_links[centerId].push_back(id);
			_links[id].push_back(centerId);
			if(!_cLinks[id].empty()) ++ count;
			for_end: continue;
		}
		if(count < 3) {
			for(int i : _links.back()) _links[i].pop_back();
			_links.pop_back();
			continue;
		}
		_points.push_back(center);
		_cLinks.emplace_back();

		// Connect the center to the rest of the path
		int a, b;
		int tmp = centerId;
		std::unordered_set<int> seen = {tmp};
		
		findConnection:
		// A = B       A --- B
		//  \ /   ===>  \\ //
		//   T            T
		for(int i = 1; i < (int) _links[tmp].size(); ++i) {
			a = _links[tmp][i];
			for(int j = 0; j < i; ++j) {
				b = _links[tmp][j];
				if(std::find(_cLinks[a].begin(), _cLinks[a].end(), b) != _cLinks[a].end())
					goto connectCenter;
			}
		}
		//    D             D = new T //
		//  // \\         /   \       //
		// C --- E  ==>  C --- E      //
		//  \   /         \\ //       //
		//    T             T         //
		for(int c : _links[tmp]) {
			for(int d : _cLinks[c]) if(!seen.count(d)) {
				const int e = _cLinks[d][_cLinks[d][0] == c];
				if(std::find(_links[e].begin(), _links[e].end(), tmp) != _links[e].end()) {
					removeLink(c, d);
					removeLink(d, e);
					createLink(c, tmp);
					createLink(tmp, e);
					seen.insert(tmp = d);
					goto findConnection;
				}
			}
		}
		THROW_ERROR("Failed linking cell center to the rest of the graph!");
		
		connectCenter:
		removeLink(a, b);
		createLink(a, tmp);
		createLink(tmp, b);
		++ _nbConnectedPoints;
	}
}

std::vector<int> CycleCreator::getPath(int start, int end) {
	if(std::find(_links[start].begin(), _links[start].end(), end) != _links[start].end())
		return {start, end};
	for(int node : _links[start]) {
		std::vector<int> path = { start };
		int prev = start;
		while(node != -1) {
			path.push_back(node);
			int next = -1;
			for(int i : _links[node]) if(i != prev) {
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
	THROW_ERROR("No path found!");
}

std::vector<int> CycleCreator::getIdxs(int i) {
	std::set<int> ans;
	for(int node : _links[i]) if(_cLinks[node].empty()) {
		int prev = i;
		while(node != -1) {
			int next = -1;
			for(int i : _links[node]) if(i != prev) {
				if(_cLinks[i].empty()) next = i;
				else ans.insert(i);
			}
			prev = node;
			node = next;
		}
	} else ans.insert(node);
	return std::vector<int>(ans.begin(), ans.end());
}

void CycleCreator::fuseIslands() {
	if(_union.fullyMerged(_nbConnectedPoints)) return;
	const int N = _points.size();
	// i ==== k       i ---- k
	// |      |  ==>  ||    ||
	// j ==== l       j ---- l
	for(int i = 0; i < N; ++i) if(!_cLinks[i].empty()) {
		for(int j : getIdxs(i)) if(!_union.same(i, j)) {
			for(int k : _cLinks[i]) for(int l : getIdxs(k))
				if(l == _cLinks[j][0] || l == _cLinks[j][1]) {
					const std::vector<int> path = getPath(i, j), path2 = getPath(l, k);
					_nbConnectedPoints += path.size() + path2.size() - 4;
					removeLink(i, k);
					removeLink(j, l);
					for(const std::vector<int> *p : {&path, &path2})
						for(int i = 1; i < (int) p->size(); ++i) {
							_union.merge(p->at(i), p->at(i-1));
							createLink(p->at(i), p->at(i-1));
						}
					if(_union.fullyMerged(_nbConnectedPoints)) return;
					goto found_flip;
				}
			found_flip: continue;
		}
	}
	// i        a -- b
	// |  ==>  ||    ||
	// j        c -- d
	for(int i = 0; i < N; ++i) if(!_cLinks[i].empty())
		for(int j : _links[i]) if(j < N && !_union.same(i, j) && !_cLinks[j].empty()) {
			glm::dvec2 a = _points[i];
			glm::dvec2 b = _points[i];
			glm::dvec2 c = _points[j];
			glm::dvec2 d = _points[j];
			glm::dvec2 u = _points[_cLinks[i][0]] - _points[i];
			glm::dvec2 v = _points[_cLinks[i][1]] - _points[i];
			if(glm::dot(u / glm::length(u) - v / glm::length(v), _points[j] - _points[i]) > 0.) a += 2e-2 * u;
			else b += 2e-2 * v;
			u = _points[_cLinks[j][0]] - _points[j];
			v = _points[_cLinks[j][1]] - _points[j];
			if(glm::dot(u / glm::length(u) - v / glm::length(v), _points[i] - _points[j]) > 0.) c += 2e-2 * u;
			else d += 2e-2 * v;
			if(Globals::intersect(a, b, c, d)) {
				std::swap(a, b);
				std::swap(_cLinks[i][0], _cLinks[i][1]);
			}
			_points[i] = a;
			_points[j] = c;
			const int k = _points.size();
			_points.push_back(b);
			_points.push_back(d);
			_links.emplace_back();
			_links.emplace_back();
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
			for(int x = 0; x < (int) _links[i].size();) {
				const int l = _links[i][x];
				const glm::dvec2 &e = _points[l];
				if(Globals::intersect(a, b, e, d)) {
					_links[i][x] = _links[i].back();
					_links[i].pop_back();
					*std::find(_links[l].begin(), _links[l].end(), i) = _links[l].back();
					_links[l].pop_back();
				} else ++x;
				if(!Globals::intersect(b, a, e, c)) {
					_links[k].push_back(l);
					_links[l].push_back(k);
				}
			}
			for(int x = 0; x < (int) _links[j].size();) {
				const int l = _links[j][x];
				const glm::dvec2 &e = _points[l];
				if(Globals::intersect(c, b, e, d)) {
					_links[j][x] = _links[j].back();
					_links[j].pop_back();
					*std::find(_links[l].begin(), _links[l].end(), j) = _links[l].back();
					_links[l].pop_back();
				} else ++x;
				if(!Globals::intersect(d, a, e, c)) {
					_links[k+1].push_back(l);
					_links[l].push_back(k+1);
				}
			}
			_links[i].push_back(k);
			_links[k].push_back(i);
			_links[j].push_back(k+1);
			_links[k+1].push_back(j);
			_links[k].push_back(k+1);
			_links[k+1].push_back(k);
			_union.merge(i, j);
			if(_union.fullyMerged(_nbConnectedPoints)) return;
		}
	THROW_ERROR("Failed connecting disjoint cycles!");
}

void CycleCreator::removeUnused(const Shape &shape) {
	std::vector<int> newIndex(_points.size(), -1);
	int newSize = 0;
	for(int i = 0; i < (int) newIndex.size(); ++i) {
		if(_cLinks[i].empty()) {
			std::vector<int> js = getIdxs(i);
			for(int k = 1; k < (int) js.size(); ++k) for(int l = 0; l < k; ++l) {
				if(std::find(_links[js[k]].begin(), _links[js[k]].end(), js[l]) == _links[js[k]].end()) {
					glm::dvec2 w = _points[js[l]] - _points[js[k]];
					w *= 1e-5f / glm::length(w);
					const glm::dvec2 a = _points[js[k]] + w, b = _points[js[l]] - w;
					for(int m : js) for(int n : _links[m]) if(!_cLinks[n].empty())
						if(Globals::intersect(a, _points[m], b, _points[n]))
							goto intersect;
					if(shape.intersect(_points[js[l]], _points[js[k]])) continue;
					_links[js[k]].push_back(js[l]);
					_links[js[l]].push_back(js[k]);
					intersect: continue;
				}
			}
		} else newIndex[i] = newSize++;
	}
	for(int i = 0; i < (int) newIndex.size(); ++i) {
		const int k = newIndex[i];
		if(k == -1) continue;
		for(int &j : _cLinks[i]) j = newIndex[j];
		int s = 0;
		for(int j : _links[i]) if(newIndex[j] != -1)
			_links[i][s++] = newIndex[j]; 
		_links[i].resize(s);
		_links[i].shrink_to_fit();
		if(k < i) {
			_points[k] = _points[i];
			_links[k] = std::move(_links[i]);
			_cLinks[k] = std::move(_cLinks[i]);
		}
	}
	_points.resize(newSize);
	_links.resize(newSize);
	_cLinks.resize(newSize);
}
