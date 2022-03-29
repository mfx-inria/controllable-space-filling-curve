//
// Created by bedela on 18/05/2020.
//

#include "initialization/GraphCreator.h"
#include "graphics/DirectionField.h"
#include "graphics/Window.h"
#include "LBFGS/cvt.hpp"
#include "LBFGS.h"
#include "tools/Random.h"

#include <iostream>

// get voronoi cell's center from its points
glm::vec2 Graph::getCellCenter(const std::vector<int> &cell, const std::vector<glm::vec2> &points) {
	glm::vec2 center(0.f, 0.f);
	float area = 0.;
	for(int i = 0; i < (int) cell.size(); ++i) {
		const glm::vec2 &a = points[cell[i]], &b = points[cell[(i+1)%cell.size()]];
		const float dx = b.x - a.x;
		const glm::vec2 s = a+b;
		area += dx * s.y / 2.;
		center += dx * (b.y*b + s.y*s + a.y*a) / glm::vec2(6.f, 12.f);
	}
	center /= area;
	return center;
}

//////////////////
//
//  SEGMENT CVT
//
//////////////////

struct GraphCVT : public CVT {
public:
    GraphCVT(const Shape *shape, const Box<double> &box, int layerIndex);
	Graph optimize(std::vector<glm::vec2> &points);

    double operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad);

private:
    std::vector<Vec2> _points;
    std::vector<Segment> _segments;
	std::vector<uint> _points_ind, _segments_ind;
	double _wantedMaxSize;
	int _layerIndex;

    void construct_voro(const Eigen::VectorXd &x, VD *vd);
    Graph getGraph(const Eigen::VectorXd &x);
};

//////////////////
//
//   Seeds initialization and post-process
//
//////////////////

// If the point is on the border we pushed it inside the shape and return true
bool push_inside(const std::vector<glm::vec2> &border, glm::vec2 &a, const float EPS=1e-4f) {
	for(int k = 1; k < (int) border.size(); ++k) {
		glm::vec2 v = border[k-1] - border[k];
		float l = glm::length(v);
		v /= l;
		const float t = std::clamp(glm::dot(v, a-border[k]), 0.f, l);
		glm::vec2 p = border[k] + t * v;
		if(glm::distance(a, p) > EPS) continue;
		if(t <= EPS) {
			int j = k+1;
			if(j == (int) border.size()) j = 1;
			const glm::vec2 w = border[k] - border[j];
			v += w / glm::length(w);
			v /= glm::length(v);
			p = border[k];
		} else if(t+EPS >= l) {
			int j = k-2;
			if(j < 0) j = border.size() - 2;
			const glm::vec2 w = border[j] - border[k-1];
			v += w / glm::length(w);
			v /= glm::length(v);
			p = border[k-1];
		}
		v *= EPS;
		a.x = p.x - v.y;
		a.y = p.y + v.x;
		return true;
	}
	return false;
}

void remove2coPoints(const Shape &border, Graph &graph) {
	// Stack containing points to remove
	std::vector<int> stack;
	for(int i = 0; i < (int) graph._points.size(); ++i)
		if(graph._links[i].size() < 3 && !border.isNear(graph._points[i]))
			stack.push_back(i);

	// Remove loop (update links)
	while(!stack.empty()) {
		int i = stack.back();
		stack.pop_back();
		std::vector<int> &link = graph._links[i];
		if(link.size() == 2) {
			for(int step = 2; step--;) {
				std::vector<int> &link2 = graph._links[link[0]];
				const std::vector<int>::iterator it = std::find(link2.begin(), link2.end(), i);
				if(std::find(link2.begin(), link2.end(), link[1]) == link2.end()) *it = link[1];
				else {
					*it = link2.back();
					link2.pop_back();
					if(link2.size() < 3 && !border.isNear(graph._points[link[0]])) stack.push_back(link[0]);
				}
				std::swap(link[0], link[1]);
			}
		} else if(link.size() == 1) {
			std::vector<int> &link2 = graph._links[link[0]];
			const std::vector<int>::iterator it = std::find(link2.begin(), link2.end(), i);
			*it = link2.back();
			link2.pop_back();
			if(link2.size() < 3 && !border.isNear(graph._points[link[0]])) stack.push_back(link[0]);
		}
		link.clear();
	}

	// Create new indices for remaining points
	std::vector<int> newIndices(graph._points.size(), -1);
	int newSize = 0;
	for(int i = 0; i < (int) newIndices.size(); ++i)
		if(!graph._links[i].empty())
			newIndices[i] = newSize++;

	// Update cells
	for(std::vector<int> &cell : graph._cells) {
		int s = 0;
		for(int i : cell) {
			int k = newIndices[i];
			if(k != -1) cell[s++] = k;
		}
		cell.resize(s);
		cell.shrink_to_fit();
	}

	// Swapping points and links
	for(int i = 0; i < (int) newIndices.size(); ++i) {
		const int k = newIndices[i];
		if(k == -1) continue;
		for(int &l : graph._links[i]) l = newIndices[l];
		if(k == i) continue;
		graph._points[k] = std::move(graph._points[i]);
		graph._links[k] = std::move(graph._links[i]);
	}
	graph._points.resize(newSize);
	graph._points.shrink_to_fit();
	graph._links.resize(newSize);
	graph._links.shrink_to_fit();
}

// Create Graph from an SVG file.
bool Graph::initGraph(const Shape &shape, Graph &graph, int layerIndex) {
	UniformReal<float> dis(-Globals::_d / 20.f, Globals::_d / 20.f);
	std::mt19937 gen(Globals::_seed + layerIndex);
	Box<float> box;
	for(const glm::vec2 &point : shape._points) box.update(point);
	std::vector<glm::vec2> centroids;
	for(float x = box.x0; x <= box.x1; x += Globals::_d) {
		int j = 1;
		for (float y = box.y0; y <= box.y1; y += sqrt(0.75) * Globals::_d) {
			glm::vec2 point(x, y);
			if((++j)&1) point.x -= .5f * Globals::_d;
			point.x += dis(gen);
			point.y += dis(gen);
			if(shape.isInside(point)) centroids.push_back(point);
		}
	}
	if(centroids.size() < 3) return false;

	GraphCVT cvt(&shape, box, layerIndex);
	graph = cvt.optimize(centroids);

	// remove 2co
	remove2coPoints(shape, graph);
	Window::add2Q(layerIndex, graph._points, graph._links, "init");

	// push points inside
	const double EPS = 1e-5f * box.diag();
	for(glm::vec2 &p : graph._points) {
		if(push_inside(shape._points, p, EPS)) continue;
		for(const std::vector<glm::vec2> &in : shape._holes)
			if(push_inside(in, p, EPS)) break;
	}
	Window::add2Q(layerIndex, graph._points, graph._links, "init");

	return true;
}

//////////////////
//
//   SEGMENT CVT
//
//////////////////

inline void getYMoments(const Vec2 &a, const Vec2 &b, double &M1, double &M2) {
	double dx = b.x - a.x;
	double bb = b.y*b.y, ba = b.y*a.y, aa = a.y*a.y;
	M1 += dx * (bb + ba + aa) / 6.;
	M2 += dx * (b.y+a.y) * (bb+aa) / 12.;
}

GraphCVT::GraphCVT(const Shape *shape, const Box<double> &box, int layerIndex): CVT(shape, box), _layerIndex(layerIndex) {}

void GraphCVT::construct_voro(const Eigen::VectorXd &x, VD *vd) {
	uint N = x.size()/2;

	////////////////
	// FIRST VORONOI
	////////////////

	// Compute Voronoi
	Box box = _shape_box;
	for(uint i = 0; i < N; ++i) box.update(Vec2(x(2*i), x(2*i+1)));
	updateScale(box);
	updatePoints(_points, x);
	boost::polygon::construct_voronoi(_points.begin(), _points.end(), vd);
	updateClipBorder();

	// Compute needles
	std::vector<Vec2> needles(N, Vec2(0., 0.));
	const double multArea = .5 * (DirectionField::_imgWidth * DirectionField::_imgHeight) / (Globals::_SVGSize.x * Globals::_SVGSize.y);
	for(const VD::cell_type &vd_c : vd->cells()) {
		if(vd_c.is_degenerate()) continue;
		uint ind = vd_c.source_index();
		if(ind < 4) continue; // it is one of the four cells added
		ind -= 4;
		// Create polygon cell
		ClipperLib::Paths ps(1);
		const VD::edge_type *e = vd_c.incident_edge();
		do {
			ps[0].emplace_back(std::round(e->vertex0()->x()), std::round(e->vertex0()->y()));
			e = e->prev();
		} while(e != vd_c.incident_edge());
		// We clip with the border
		ClipperLib::Paths clipped = clip(ps);
		double area = 0.;
		std::vector<std::vector<Vec2>> Ps;
		for(ClipperLib::Path &P0 : clipped) {
			const int n = P0.size()+1;
			Ps.emplace_back();
			std::vector<Vec2> &P = Ps.back();
			P.reserve(n);
			for(const ClipperLib::IntPoint &p : P0) P.emplace_back(p.X / _scale + _mid.x, p.Y / _scale + _mid.y);
			P.push_back(P[0]);
			for(int i = 1; i < n; ++i) {
				needles[ind] += DirectionField::getVecUnder(P[i], P[i-1], _layerIndex);
				area += (P[i-1].x - P[i].x) * (P[i].y + P[i-1].y);
			}
		}
		if(area <= 1e-6) {
			needles[ind].x = 0.;
			needles[ind].y = 0.;
			continue;
		}
		double l = std::min(1., glm::length(needles[ind]) / (multArea * area));
		if(l < 1e-4) {
			needles[ind].x = 0.;
			needles[ind].y = 0.;
			continue;
		}
		l *= _wantedMaxSize;
		double angle = std::atan2(needles[ind].y, needles[ind].x) / 2.;
		needles[ind].x = l * std::cos(angle);
		needles[ind].y = l * std::sin(angle);
		double T = 1.;
		Vec2 a(x(2*ind) - needles[ind].x, x(2*ind+1) - needles[ind].y);
		for(const std::vector<Vec2> &P : Ps) {
			for(int i = 1; i < (int) P.size(); ++i) {
				Vec2 e = P[i-1] - P[i];
				double div = needles[ind].x * e.y - needles[ind].y * e.x;
				if(div == 0.) continue;
				double t = (P[i].x - a.x) * e.y - (P[i].y - a.y) * e.x, u;
				if(std::abs(e.x) > std::abs(e.y)) u = (a.x + t*needles[ind].x - P[i].x) / e.x;
				else u = (a.y + t*needles[ind].y - P[i].y) / e.y;
				if(u >= 0. && u <= 1.) T = std::min(T, .99*std::abs(t / div - 1.));
			}
		}
		needles[ind] *= T;
	}

	/////////////////
	// SECOND VORONOI
	/////////////////

	_points_ind.resize(4);
	_segments_ind.clear();
	for(uint i = 0; i < N; ++i) {
		if(glm::length(needles[i]) <= 1e-4 * _wantedMaxSize) _points_ind.push_back(i);
		else _segments_ind.push_back(i);
	}
	if(_segments_ind.empty()) return;
	_points.resize(_points_ind.size());
	_segments.resize(_segments_ind.size());

	// Compute box
	for(uint i = 0; i < _segments_ind.size(); ++i) {
		uint j = _segments_ind[i];
		_segments[i].first.x = x(2*j)  - needles[j].x;
		_segments[i].first.y = x(2*j+1) - needles[j].y;
		_segments[i].second.x = x(2*j) + needles[j].x;
		_segments[i].second.y = x(2*j+1) + needles[j].y;
		box.update(_segments[i].first);
		box.update(_segments[i].second);
	}
	updateScale(box);

	for(uint i = 4; i < _points.size(); ++i) {
		_points[i].x = _scale * (x(2*_points_ind[i]) - _mid.x);
		_points[i].y = _scale * (x(2*_points_ind[i]+1) - _mid.y);
	}
	for(Segment &seg : _segments) {
		seg.first = _scale * (seg.first - _mid);
		seg.second = _scale * (seg.second - _mid);
	}

	vd->clear();
	boost::polygon::construct_voronoi(_points.begin(), _points.end(), _segments.begin(), _segments.end(), vd);
	updateClipBorder();
}

double GraphCVT::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad) {
	double f = 0;
	grad.setZero(x.size());
	VD vd;
	construct_voro(x, &vd);

	// Iterate over cells
	for(const VD::cell_type &vd_c : vd.cells()) {
		if(vd_c.is_degenerate()) continue;
		uint ind = vd_c.source_index();
		if(ind < 4) continue; // it is one of the four points added

		// Create polygon cell
		ClipperLib::Paths ps;
		const double err = cell2Clipper(vd_c, 4, ps, _points, _segments, x, grad);
		if(err) return err;
		ClipperLib::Paths clipped;
		try {
			clipped = clip(ps);
		} catch(const ClipperLib::clipperException &e) {
			return foundError(x, grad, "clipper error");
		}

		Vec2 a, v;
		if(ind < _points.size()) {
			a = _points[ind] / _scale;
			ind = _points_ind[ind];
		} else {
			ind -= _points.size();
			if(vd_c.source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) a = _segments[ind].first / _scale;
			else if(vd_c.source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) a = _segments[ind].second / _scale;
			else {
				a = _segments[ind].first / _scale;
				v = _segments[ind].second / _scale - a;
				v /= std::sqrt(v.x*v.x + v.y*v.y);
			}
			ind = _segments_ind[ind];
		}

		// Iteration over every polygon of the clipped triangle
		for(ClipperLib::Path &P0 : clipped) {
			std::vector<Vec2> P;
			P.reserve(P0.size()+1);
			for(const ClipperLib::IntPoint &p : P0) P.emplace_back(p.X / _scale, p.Y / _scale);
			P.push_back(P[0]);
			if(vd_c.contains_point()) {
				Vec2 g;
				f += point_fun_grad(P, a, g);
				grad(2*ind) += g.x;
				grad(2*ind+1) += g.y;
			} else {
				for(Vec2 &u : P) {
					u -= a;
					u = Vec2(v.x*u.x + v.y*u.y, -v.y*u.x + v.x*u.y);
				}
				double M1 = 0., M2 = 0.;
				for(int i = 1; i < (int) P.size(); ++i) getYMoments(P[i], P[i-1], M1, M2);
				f += .5 * M2;
				grad(2*ind) += v.y * M1;
				grad(2*ind+1) -= v.x * M1;
			}
		}
	}

	if(f < _prevF) {
		_prevF = f;
		_prevX = x;
		Graph g = getGraph(x);
		Window::add2Q(_layerIndex, g._points, g._links, "init");
	}
	return f;
}

Graph GraphCVT::getGraph(const Eigen::VectorXd &x) {
	VD vd;
	construct_voro(x, &vd);
	Graph graph;
	graph._cells.resize(_points.size() + _segments.size() - 4);

	// Iterate over cells
	const auto vec2Comp = [](const glm::vec2 &a, const glm::vec2 &b) { return a.x < b.x || (a.x == b.x && a.y < b.y); };
	std::map<glm::vec2, int, decltype(vec2Comp)> m(vec2Comp);
	const float eps = 5e-5f;
	for(const VD::cell_type &vd_c : vd.cells()) {
		if(vd_c.is_degenerate()) continue;
		uint ind = vd_c.source_index();
		if(ind < 4) continue;

		// Create polygon cell
		ClipperLib::Paths ps(1);
		Eigen::VectorXd grad;
		cell2Clipper(vd_c, 1, ps, _points, _segments, x, grad);
		ClipperLib::Paths clipped = clip(ps);

		// Iteration over every polygon of the clipped cell
		const auto link = [&](int i, int j)->void {
			if(i == j || std::find(graph._links[i].begin(), graph._links[i].end(), j) != graph._links[i].end()) return;
			if(ind >= _points.size()) {
				glm::vec2 A = _segments[ind - _points.size()].first / _scale + _mid;
				glm::vec2 B = _segments[ind - _points.size()].second / _scale + _mid;
				const glm::vec2 &p = graph._points[i];
				glm::vec2 v = graph._points[j] - p;
				float len = glm::length(v);
				v /= len;
				if(glm::distance(p + v * std::min(len, std::max(0.f, glm::dot(v, A - p))), A) < 4.f*eps
				   || glm::distance(p + v * std::min(len, std::max(0.f, glm::dot(v, B - p))), B) < 4.f*eps) return;
			}
			graph._links[i].push_back(j);
			graph._links[j].push_back(i);
		};
		std::vector<int> &cell = graph._cells[ind-4];
		for(ClipperLib::Path &P0 : clipped) {
			int start=-1, last=-1;
			for(const ClipperLib::IntPoint &p0 : P0) {
				const glm::vec2 p(p0.X / _scale + _mid.x, p0.Y / _scale + _mid.y);
				int idx = -1;
				for(auto it = m.lower_bound(p - glm::vec2(eps, 0.f)); it != m.end() && it->first.x < p.x+eps; ++it) {
					if(glm::distance(p, it->first) < eps) {
						idx = it->second;
						break;
					}
				}
				if(idx == -1) {
					m[p] = idx = graph._points.size();
					graph._points.push_back(p);
					graph._links.emplace_back();
				}
				if(std::find(cell.begin(), cell.end(), idx) == cell.end()) cell.push_back(idx);
				if(last == -1) start = idx;
				else link(last, idx);
				last = idx;
			}
			link(last, start);
		}
	}

	// Reorder cell
	for(std::vector<int> &cell : graph._cells) {
		if(cell.empty()) continue;
		std::set<int> S(cell.begin()+1, cell.end());
		for(int i = 1; i < (int) cell.size(); ++i) {
			for(int j : graph._links[cell[i-1]]) {
				std::set<int>::iterator it = S.find(j);
				if(it != S.end()) {
					cell[i] = *it;
					S.erase(it);
					break;
				}
			}
		}
	}

	return graph;
}

Graph GraphCVT::optimize(std::vector<glm::vec2> &points) {
	Eigen::VectorXd x(2 * points.size());
	for(int i = 0; i < (int) points.size(); ++i) {
		x(2*i) = points[i].x;
		x(2*i+1) = points[i].y;
	}
	_prevX = x;
	_prevF = std::numeric_limits<double>::max();
	_wantedMaxSize = .66 * std::sqrt(_shape->_area / (M_PI * points.size()));
	Graph g = getGraph(x);
	Window::add2Q(_layerIndex, g._points, g._links, "init");
	g = Graph();

	// LBFGS
	LBFGSpp::LBFGSParam<double> param;
	param.m = 9;
	param.max_iterations = 80;
	param.epsilon = 0.1;
	param.max_linesearch = 7;
	LBFGSpp::LBFGSSolver<double> solver(param);
	double f;
	try {
		solver.minimize(*this, x, f);
	} catch(const std::runtime_error &e) {
		std::cerr << "[GraphCVT]: " << e.what() << std::endl;
	} catch(const std::logic_error &e) {
		std::cerr << "[GraphCVT Logic Error]: " << e.what() << std::endl;
	} catch(const self_intersection_error &e) {
		std::cerr << "[GraphCVT Self Intersection Error]: " << e.what() << std::endl;
	}
	_prevF = std::numeric_limits<double>::max();
	return getGraph(_prevX);
}
