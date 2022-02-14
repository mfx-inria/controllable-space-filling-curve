//
// Created by bedela on 01/07/2020.
//

#include "initialization/CycleCreator.h"
#include "initialization/Matching.h"
#include "tools/Random.h"

#include <algorithm>
#include <unordered_set>
#include <iostream>

CycleCreator::CycleCreator(const Shape &shape, Graph &graph) {
	_points = std::move(graph._points);
	_links = std::move(graph._links);
	_cLinks.resize(_links.size());
	for(const std::vector<int> &link : _links) if(link.size() > 3)
		THROW_ERROR(str_format("Graph is not 3-connected! (valence %d found)", link.size()));
	
	std::cerr << "here" << std::endl;
	perfectMatching();
	std::cerr << "here2" << std::endl;
	switchLink();
	for (const std::vector<int> &cell : graph._cells) {
		if(cell.size() < 3) continue;

		// Compute the center of the cell
		const glm::vec2 center = Graph::getCellCenter(cell, _points);
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
		int A = -1, B;
		int tmp = centerId;
		std::unordered_set<int> seen = {tmp};
		while (A == -1) {
			// A = B       A --- B
			//  \ /   ===>  \\ //
			//   T            T
			for (int i = 1; i < (int) _links[tmp].size(); i++) {
				int a = _links[tmp][i];
				for(int j = 0; j < i; ++j) {
					const int b = _links[tmp][j];
					if(std::find(_cLinks[a].begin(), _cLinks[a].end(), b) != _cLinks[a].end()) {
						A = a;
						B = b;
					}
				}
				if(A != -1) break;
			}

			if (A == -1) {  // (A, B) not found
				int C = -1, D;
				//    C             C = new T //
				//  // \\         /   \       //
				// B --- D  ==>  B --- D      //
				//  \   /         \\ //       //
				//    T             T         //
				for (int b : _links[tmp]) {
					if(C != -1) break;
					for (int c : _cLinks[b]) if(!seen.count(c)) {
						int d = _cLinks[c][0] == b ? _cLinks[c][1] : _cLinks[c][0];
						bool linkedB = false, linkedTmp = false;
						for(int link : _links[d]) {
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
				if(C != -1) {
					removeLink(B, C);
					removeLink(C, D);
					createLink(B, tmp);
					createLink(tmp, D);
					tmp = C;
					seen.insert(tmp);
				} else THROW_ERROR("Failed linking cell center to the rest of the graph!");
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
	removeUnused(shape);
}

void CycleCreator::initUnion() {
	_union = Union(_points.size());
	for(int i = 0; i < (int) _points.size(); ++i)
		for(int j : _cLinks[i])
			_union.merge(i, j);
}

std::vector<int> CycleCreator::getPath(int start, int end) {
	if(std::find(_links[start].begin(), _links[start].end(), end) != _links[start].end())
		return {start, end};
	std::vector<int>::iterator it = _links[start].begin();
	while(true)  {
		std::vector<int> path = { start };
		it = std::find_if(it, _links[start].end(), [&](int link) { return _cLinks[link].empty(); });
		int prev = start;
		int node = *it;
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
}

void CycleCreator::removeUnused(const Shape &border) {
	std::vector<int> newIndex(_points.size(), -1);
	int newSize = 0;
	for(int i = 0; i < (int) newIndex.size(); ++i) {
		if(_cLinks[i].empty()) {
			for(int j : _links[i]) if(!_cLinks[j].empty()) {
				std::vector<int> js = getIdxs(i, j);
				js.push_back(j);
				for(int k = 1; k < (int) js.size(); ++k)
					for(int l = 0; l < k; ++l)
						if(std::find(_links[js[k]].begin(), _links[js[k]].end(), js[l]) == _links[js[k]].end()) {
							bool bad = false;
							glm::vec2 w = _points[js[l]] - _points[js[k]];
							w *= 1e-5f / glm::length(w);
							glm::vec2 a = _points[js[k]] + w, b = _points[js[l]] - w;
							for(int m : js) if(!bad)
								for(int n : _links[m]) if(!_cLinks[n].empty())
									if(Globals::intersect(a, _points[m], b, _points[n])) {
										bad = true;
										break;
									}
							if(bad) continue;
							for(int m = 1; m < (int) border._points.size(); ++m)
								if(Globals::intersect(_points[js[l]], border._points[m-1], _points[js[k]], border._points[m])) {
									bad = true;
									break;
								}
							if(bad) continue;
							for(const std::vector<glm::vec2> &hole : border._holes) {
								for(int m = 1; m < (int) hole.size(); ++m)
									if(Globals::intersect(_points[js[l]], hole[m-1], _points[js[k]], hole[m])) {
										bad = true;
										break;
									}
								if(bad) break;
							}
							if(bad) continue;
							_links[js[k]].push_back(js[l]);
							_links[js[l]].push_back(js[k]);
						}
				break;
			}
		} else newIndex[i] = newSize++;
	}
	for(int i = 0; i < (int) newIndex.size(); ++i) {
		int k = newIndex[i];
		if(k == -1) continue;
		for(int &j : _cLinks[i]) j = newIndex[j];
		int s = 0;
		for(int j : _links[i]) if(newIndex[j] != -1)
			_links[i][s++] = newIndex[j]; 
		_links[i].resize(s);
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

std::vector<int> CycleCreator::getIdxs(int node, int parent) {
	if(!_cLinks[node].empty()) return {node};
	std::vector<int> ans;
	int prev = parent;
	while(node != -1) {
		int next = -1;
		for(int i : _links[node]) if(i != parent && i != prev) {
			if(_cLinks[i].empty()) next = i;
			else if(std::find(ans.begin(), ans.end(), i) == ans.end()) ans.push_back(i);
		}
		prev = node;
		node = next;
	}
	return ans;
}

void CycleCreator::fuseIslands() {
	if(_union.fullyMerged(_nbConnectedPoints)) return;
	int flip = 0;
	int N = _points.size();
	// i ==== k       i ---- k
	// |      |  ==>  ||    ||
	// j ==== l       j ---- l
	for(int i = 0; i < N; ++i) if(!_cLinks[i].empty()) {
		for(int j0 : _links[i]) for(int j : getIdxs(j0, i)) {
			if(!_union.same(i, j)) {
				bool found = false;
				for(int k : _cLinks[i]) if(!found) {
					for(int l0 : _links[k]) if(!found) for(int l : getIdxs(l0, k)) {
						if(l == _cLinks[j][0] || l == _cLinks[j][1]) {
							std::vector<int> path = getPath(i, j), path2 = getPath(l, k);
							_nbConnectedPoints += path.size() + path2.size() - 4;
							removeLink(i, k);
							removeLink(j, l);
							for (int inc = 1; inc < (int) path.size(); ++inc) {
								_union.merge(path[inc], path[inc - 1]);
								createLink(path[inc], path[inc - 1]);
							}
							for (int inc = 1; inc < (int) path2.size(); ++inc) {
								_union.merge(path2[inc], path2[inc - 1]);
								createLink(path2[inc], path2[inc - 1]);
							}
							++ flip;
							if(_union.fullyMerged(_nbConnectedPoints)) return;
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
		for(int j : _links[i]) if(j < N && !_union.same(i, j) && !_cLinks[j].empty()) {
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
					int l = _links[i][x];
					const glm::vec2 &e = _points[l];
					if(Globals::intersect(a, b, e, d)) {
						_links[i][x] = _links[i].back();
						_links[i].pop_back();
						int y = 0;
						while(_links[l][y] != i) ++y;
						_links[l][y] = _links[l].back();
						_links[l].pop_back();
					} else ++x;
					if(!Globals::intersect(b, a, e, c)) {
						_links[k].push_back(l);
						_links[l].push_back(k);
					}
				}
				for(int x = 0; x < (int) _links[j].size();) {
					int l = _links[j][x];
					const glm::vec2 &e = _points[l];
					if(Globals::intersect(c, b, e, d)) {
						_links[j][x] = _links[j].back();
						_links[j].pop_back();
						int y = 0;
						while(_links[l][y] != j) ++y;
						_links[l][y] = _links[l].back();
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
	}
	THROW_ERROR("Failed connecting disjoint cycles!");
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
			m.node(i).link[j] = inv_order[getNext(_links[order[i]][j], order[i])];
	if(2 * m.compute() != (int) order.size()) THROW_ERROR("Matching failed");

	// Back to the original graph
	for(int i = 0; i < (int) order.size(); ++i) {
		int a = order[i];
		if(!_cLinks[a].empty()) continue;
		const int b = order[m.node(i).getMatch()];
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
