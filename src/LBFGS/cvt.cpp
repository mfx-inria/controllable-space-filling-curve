#include "LBFGS/cvt.hpp"
#include "LBFGS.h"
#include "graphics/DirectionField.h"
#include "graphics/Window.h"

#include <iostream>

/////////////
//
//   TOOLS
//
/////////////

inline void getMomentsSeg(const Vec2 &a, const Vec2 &b, double &M0, Vec2 &M1, Vec2 &M2) {
	Vec2 d(a.y - b.y, b.x - a.x);
	Vec2 s = a+b;
	M0 += d.x * s.x / 2.;
	M1 += d * (a*s + b*b) / 6.;
	M2 += d * (a*a + b*b) * s / 12.;
}

inline void getYMoments2(const Vec2 &a, const Vec2 &b, double l, double &M1a, double &M1b, double &M2) {
	double dx = b.x - a.x;
	double ta = a.x/l, tb = b.x/l;
	double bb = b.y*b.y, ba = b.y*a.y, aa = a.y*a.y;
	double m1b = dx * ((3.*tb+ta)*bb + 2.*(ta+tb)*ba + (3.*ta+tb)*aa) / 24.;
	M1b += m1b;
	M1a += dx * (bb + ba + aa) / 6. - m1b;
	M2 += dx * (b.y+a.y) * (bb+aa) / 12.;
}

inline double length(const Eigen::VectorXd &x) {
	double L = 0;
	int N = x.size();
	for(int i = 0; i < N; i += 2) {
		int j = (i+2)%N;
		double dx = x(i)-x(j), dy = x(i+1)-x(j+1);
		L += std::sqrt(dx*dx + dy*dy);
	}
	return L;
}

inline void addCurvedEdge(const VD::edge_type *e, const std::vector<Vec2> &points, const std::vector<Segment> &segments, ClipperLib::Path &p, int ni) {
	if(e == nullptr || e->vertex1() == nullptr || !e->is_curved() || e->twin()->cell()->source_index() < 4 || ni < 2) return;
	const VD::vertex_type *start = e->vertex1(), *end = e->vertex0();
	uint ind = e->cell()->source_index();
	uint ind2 = e->twin()->cell()->source_index();
	if(e->cell()->contains_point()) std::swap(ind, ind2);
	else e = e->twin();
	// ind is segment, (ind2, e) is point
	ind -= points.size();
	Vec2 a = segments[ind].first, b = segments[ind].second, c;
	if(e->cell()->source_category() == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) c = points[ind2];
	else if(e->cell()->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) c = segments[ind2-points.size()].second;
	else c = segments[ind2-points.size()].first;
	b -= a;
	b /= std::sqrt(b.x*b.x + b.y*b.y);
	const Vec2 b2(-b.y, b.x);
	c -= a;
	c = Vec2(c.x*b.x + c.y*b.y, c.x*b2.x + c.y*b2.y);
	const double x0 = (start->x() - a.x)*b.x + (start->y() - a.y)*b.y;
	const double x1 = (end->x() - a.x)*b.x + (end->y() - a.y)*b.y - x0;
	for(int i = 1; i < ni; ++i) {
		const double x = x0 + (x1 * i) / ni;
		const double dx = x - c.x;
		const double y = .5 * (c.y + dx*dx/c.y);
		p.emplace_back(std::round(a.x + x*b.x + y*b2.x), std::round(a.y + x*b.y + y*b2.y));
	}
}

/////////////////////
//
//   COMMON CVT
//
/////////////////////

inline Box<double> createBox(const Shape *shape) {
	Box<double> box;
	for(const glm::vec2 &p : shape->_points) box.update(p);
	return box;
}

inline ClipperLib::Paths shape2clip(const Shape *shape) {
	ClipperLib::Paths clipPath(1 + shape->_holes.size());
	clipPath[0].resize(shape->_points.size()-1);
	for(int i = 0; i < (int) shape->_holes.size(); ++i)
		clipPath[i+1].resize(shape->_holes[i].size()-1);
	return clipPath;
}

inline void rescale(const Shape *shape, ClipperLib::Paths &paths, double scale, const Vec2 &mid) {
	for(uint i = 0; i < paths[0].size(); ++i) {
		paths[0][i].X = scale * (shape->_points[i].x - mid.x);
		paths[0][i].Y = scale * (shape->_points[i].y - mid.y);
	}
	for(uint i = 0; i < shape->_holes.size(); ++i)
		for(uint j = 0; j < paths[i+1].size(); ++j) {
			paths[i+1][j].X = scale * (shape->_holes[i][j].x - mid.x);
			paths[i+1][j].Y = scale * (shape->_holes[i][j].y - mid.y);
		}
}

inline ClipperLib::Paths clipCell(const ClipperLib::Paths &path, const ClipperLib::Paths &border) {
	ClipperLib::Paths clipped;
	ClipperLib::Clipper cl;
	cl.AddPaths(path, ClipperLib::ptSubject, true);
	cl.AddPaths(border, ClipperLib::ptClip, true);
	cl.Execute(ClipperLib::ctIntersection, clipped, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
	return clipped;
}

Voronoi::Voronoi(const Shape *shape): Voronoi(shape, createBox(shape)) {}

Voronoi::Voronoi(const Shape *shape, const Box<double> &box) : _shape(shape), _shape_box(box), _clipBorder(shape2clip(shape)) {}

void Voronoi::updateScale(const Box<double> &box) {
	_mid = (box.min() + box.max()) / 2.;
	_scale = INT32_MAX / (2.5 * std::max(box.W(), box.H()));
}

void Voronoi::updateClipBorder() { rescale(_shape, _clipBorder, _scale, _mid); }

void Voronoi::updatePoints(std::vector<Vec2> &points, const Eigen::VectorXd &x) {
	const int N = x.size()/2;
	points.resize(N+4);
	points[0] = Vec2(-INT32_MAX, -INT32_MAX);
	points[1] = Vec2(-INT32_MAX, INT32_MAX);
	points[2] = Vec2(INT32_MAX, -INT32_MAX);
	points[3] = Vec2(INT32_MAX, INT32_MAX);
	for(int i = 0; i < N; ++i) {
		points[i+4].x = _scale * (x(2*i) - _mid.x);
		points[i+4].y = _scale * (x(2*i+1) - _mid.y);
	}
}

ClipperLib::Paths Voronoi::clip(const ClipperLib::Paths &path) { return clipCell(path, _clipBorder); }

double CVT::foundError(const Eigen::VectorXd &x, Eigen::VectorXd &grad, const char* msg) {
	if(_prevF != std::numeric_limits<double>::max()) {
		grad = x - _prevX;
		double len = grad.norm();
		grad *= 100./len;
		return _prevF + 100.*len;
	}
	throw self_intersection_error(msg);
}

double CVT::cell2Clipper(const VD::cell_type &vd_c, int ni, ClipperLib::Paths &path,
						const std::vector<Vec2> points, const std::vector<Segment> &segments,
						const Eigen::VectorXd &x, Eigen::VectorXd &grad) {
	path.resize(1);
	ClipperLib::Path &p = path[0];
	const VD::edge_type *e = vd_c.incident_edge();
	int countEloop = 0;
	do {
		if(++ countEloop > 1000) return foundError(x, grad, "infinite looping over edges");
		if(e->is_infinite()) return foundError(x, grad, "infinite edge");
		addCurvedEdge(e, points, segments, p, ni);
		p.emplace_back(std::round(e->vertex0()->x()), std::round(e->vertex0()->y()));
		e = e->prev();
	} while(e != vd_c.incident_edge());
	ClipperLib::cInt area = (p[0].X - p.back().X) * (p[0].Y + p.back().Y);
	for(int i = 1; i < (int) p.size(); ++i) area += (p[i].X - p[i-1].X) * (p[i].Y + p[i-1].Y);
	if(area < 0.) return foundError(x, grad, "negative area");
	return 0.;
}

double CVT::point_fun_grad(const std::vector<Vec2> &P, const Vec2 &a, Vec2 &grad) {
	double M0 = 0.;
	Vec2 M1(0., 0.), M2(0., 0.);
	for(int i = 1; i < (int) P.size(); ++i) getMomentsSeg(P[i], P[i-1], M0, M1, M2);
	Vec2 add = .5 * (M2 - 2.*M1*a + M0*a*a);
	grad = M0*a - M1;
	return add.x + add.y;
}

//////////////////
//
//   POINT CVT
//
//////////////////

void PointCVT::updateLink(const Eigen::VectorXd &x,
						  std::vector<glm::vec2> &points,
						  std::vector<std::vector<int>> &oriLink,
						  std::vector<std::vector<int>> &cLink)
{
	int N = x.size()/2;
	points.resize(N);
	oriLink.resize(N);
	cLink.resize(N);
	for(int i = 0; i < N; ++i) {
		points[i].x = x(2*i);
		points[i].y = x(2*i+1);
		cLink[i] = oriLink[i] = {(i+1)%N, (i+N-1)%N};
	}

	VD vd;
	Box box = _shape_box;
	for(int i = 0; i < N; ++i) box.update(Vec2(x(2*i), x(2*i+1)));
	updateScale(box);
	updatePoints(_points, x);
	boost::polygon::construct_voronoi(_points.begin(), _points.end(), &vd);
	updateClipBorder();

	for(const VD::cell_type &vd_c : vd.cells()) {
		int i = vd_c.source_index();
		if(i < 4) continue;
		i -= 4;
		const VD::edge_type *e = vd_c.incident_edge();
		do {
			const int j = (int)e->twin()->cell()->source_index()-4;
			if(j < 0 || std::find(oriLink[i].begin(), oriLink[i].end(), j) != oriLink[i].end()) goto nextEdge;
			for(int k = 0; k < N; ++k) if(k != i && k != j) {
				const int l = (k+1)%N;
				if(l != i && l != j && Globals::intersect(points[i], points[k], points[j], points[l])) goto nextEdge;
			}
			if(_shape->intersect(points[i], points[j])) goto nextEdge;
			oriLink[i].push_back(j);
			oriLink[j].push_back(i);
			nextEdge:
			e = e->prev();
		} while(e != vd_c.incident_edge());
	}
}

//////////////////
//
//   SMOOTHER
//
//////////////////

const std::vector<Vec2> Smoother::_points = {
		Vec2(-INT32_MAX, -INT32_MAX),
		Vec2(-INT32_MAX, INT32_MAX),
		Vec2(INT32_MAX, -INT32_MAX),
		Vec2(INT32_MAX, INT32_MAX)
};

Smoother::Smoother(const Shape *shape, const std::vector<Shape> *objZones, const std::vector<Shape> *colorZones, double vecArea, int layerIndex):
			CVT(shape), _objZones(objZones), _colorZones(colorZones),
			_paths(colorZones->size()), _segments(colorZones->size()),
			_layerIndex(layerIndex), _vecArea(vecArea) {
	_clipZones.reserve(colorZones->size());
	for(const Shape &zone : *colorZones) _clipZones.push_back(shape2clip(&zone));
}

void Smoother::construct_voro(const Eigen::VectorXd &x) {
	int N = x.size()/2;
	Box box = _shape_box;
	Vec2 b(x(0), x(1));
	int k = 0;
	while(k < (int) _colorZones->size() && !(*_colorZones)[k].isInside(b)) ++k;
	if(k >= (int) _colorZones->size()) k = -1;
	for(std::vector<std::tuple<int, double, double>> &p : _paths) p.clear();
	for(int i = 0; i < N; ++i) {
		const int j = (i+1)%N;
		Vec2 a(x(2*j), x(2*j+1));
		std::swap(a, b);
		box.update(a);
		std::vector<float> ts;
		// TODO: check if this can be false
		if(k >= 0) {
			(*_colorZones)[k].getInter(a, b, ts);
			if(ts.empty()) {
				_paths[k].emplace_back(i, 0., 1.);
				continue;
			}
			ts.push_back(0.f);
		} else (*_colorZones)[k = 0].getInter(a, b, ts);
		int l = k, k2 = -1;
		do {
			if(ts.size()&1) {
				k2 = l;
				ts.push_back(1.f);
			}
			std::sort(ts.begin(), ts.end());
			for(int m = 0; m < (int) ts.size(); m += 2)
				_paths[l].emplace_back(i, ts[m], ts[m+1]);
			ts.clear();
			do {
				l = (l+1) % _colorZones->size();
				if(l == k) break;
				(*_colorZones)[l].getInter(a, b, ts);
			} while(ts.empty());
		} while(l != k);
		k = k2;
	}
	updateScale(box);

	// compute segments
	for(k = 0; k < (int) _colorZones->size(); ++k) {
		_segments[k].clear();
		_segments[k].reserve(_paths[k].size());
		for(const auto &[i, a, b] : _paths[k]) {
			const int j = (i+1) % N;
			_segments[k].emplace_back(Vec2(_scale * ((1.-a)*x(2*i) + a*x(2*j) - _mid.x), _scale * ((1.-a)*x(2*i+1) + a*x(2*j+1) - _mid.y)),
									  Vec2(_scale * ((1.-b)*x(2*i) + b*x(2*j) - _mid.x), _scale * ((1.-b)*x(2*i+1) + b*x(2*j+1) - _mid.y)));
		}
		rescale(&(*_colorZones)[k], _clipZones[k], _scale, _mid);
	}
}

inline double Smoother::vectorfieldEnergy(const Eigen::VectorXd &x, std::vector<double> &grad, double &div, int j, int k, std::vector<Vec2> &P) {
	Vec2 V(0., 0.), vecAB(x(2*k)-x(2*j), x(2*k+1)-x(2*j+1));
	for(int i = 1; i < (int) P.size(); ++i) V += DirectionField::getVecUnder(P[i], P[i-1], _layerIndex);
	double area = glm::length(V);
	if(area < 1e-5) return 0.;
	div += area;
	double alpha = std::atan2(V.y, V.x) / 2.;

	double theta = std::atan2(vecAB.y, vecAB.x);
	while(alpha > theta+M_PI_2) alpha -= M_PI;
	while(alpha < theta-M_PI_2) alpha += M_PI;

	vecAB /= (vecAB.x*vecAB.x + vecAB.y*vecAB.y);
	double d = theta - alpha;
	double dx = -2. * area * d * vecAB.y;
	double dy =  2. * area * d * vecAB.x;
	grad[2*j] -= dx;
	grad[2*j+1] -= dy;
	grad[2*k] += dx;
	grad[2*k+1] += dy;
	return area * d*d;
}

inline double Smoother::isotropyEnergy(const Eigen::VectorXd &x, Eigen::VectorXd &grad, double coeff) {
	const int N = x.size()/2;
	coeff *= 8. / (3. * M_PI);
	double f = 0.;
	std::vector<bool> prev_inside(N, false);

	for(const Shape &zone : *_objZones) if(IS_ISOTROPY(zone._objcetive)) {
		double coeffZ = coeff * zone._area;

		// ANGLE x LENGTH x INDEX
		std::vector<std::tuple<double, double, int>> distrib;
		double alpha = 0.;
		double L = 0.;
		for(int i = 0; i < N; ++i) {
			int j = (i+1)%N;
			glm::vec2 a(x(2*i), x(2*i+1)), b(x(2*j), x(2*j+1));
			std::vector<float> ts;
			if(!prev_inside[i] && zone.isInside(a)) {
				prev_inside[i] = true;
				ts.push_back(0.f);
			}
			zone.getInter(a, b, ts);
			if(ts.empty()) continue;
			std::sort(ts.begin(), ts.end());
			if(ts.size()&1) ts.push_back(1.f);
			b -= a;
			double angle = std::atan2(b.y, b.x);
			if(angle >= M_PI_2) angle -= M_PI;
			else if(angle < -M_PI_2) angle += M_PI;
			double len = 0.;
			for(int k = 0; k < (int) ts.size(); k += 2)
				len += ts[k+1] - ts[k];
			len *= glm::length(b);
			alpha += len * angle;
			L += len;
			distrib.emplace_back(angle, len, i);
		}

		if(distrib.empty()) {
			if(zone._objcetive == ISOTROPY) f += coeffZ * (M_PI*M_PI*M_PI/4.);
			continue;
		}
		if(zone._objcetive == ANISOTROPY) f += coeffZ * (M_PI*M_PI*M_PI/4.);
		else coeffZ *= std::max(1., .0123 * zone._area / _shape->_area * N);

		alpha /= L;
		alpha -= M_PI_2;
		std::sort(distrib.begin(), distrib.end());

		double t = 0.;
		const double pi_L = M_PI/L;
		for(const auto &[angle, len, i] : distrib) {
			const double u0 = pi_L*t + alpha - angle;
			t += len;
			const double u1 = pi_L*t + alpha - angle;
			const double u02 = u0*u0;
			const double u12 = u1*u1;
			if(zone._objcetive == ISOTROPY) f += coeffZ * (u1*u12 - u0*u02);
			else f -= coeffZ * (u1*u12 - u0*u02);

			const int j = (i+1) % N;
			const double vx = x(2*j)-x(2*i);
			const double vy = x(2*j+1)-x(2*i+1);
			const double diff2 = 3. * coeffZ * (u12 - u02) / (vx*vx + vy*vy);
			double dx = diff2 * vy;
			double dy = - diff2 * vx;
			if(zone._objcetive == ANISOTROPY) {
				dx = -dx;
				dy = -dy;
			}
			grad(2*i) -= dx;
			grad(2*i+1) -= dy;
			grad(2*j) += dx;
			grad(2*j+1) += dy;
		}

	}

	return f;
}

double Smoother::operator()(Eigen::VectorXd &x, Eigen::VectorXd &grad) {
	const int N = x.size()/2;

	double f = 0;
	grad.setZero(x.size());
	double vecF = 0., vecDiv = 0.;
	std::vector<double> vecGrad(x.size(), 0.);

	// Construct VD
	construct_voro(x);

	// Iterate over cells
	for(int V = 0; V < (int) _colorZones->size(); ++V) {
		VD vd;
		boost::polygon::construct_voronoi(_points.begin(), _points.end(), _segments[V].begin(), _segments[V].end(), &vd);
		for(const VD::cell_type &vd_c : vd.cells()) {
			if(vd_c.is_degenerate()) continue;
			uint ind = vd_c.source_index();
			if(ind < 4) continue;
			ind -= 4;
			// Create polygon cell
			ClipperLib::Paths ps;
			const double err = cell2Clipper(vd_c, 10, ps, _points, _segments[V], x, grad);
			if(err) return err;
			ClipperLib::Paths clipped;
			try {
				clipped = clipCell(ps, _clipZones[V]);
			} catch(const ClipperLib::clipperException &e) {
				return foundError(x, grad, "clipper error");
			}

			auto [I, A, B] = _paths[V][ind];
			int J = (I+1) % N;
			Vec2 a, v;
			double l=0.;
			const bool isPoint = vd_c.contains_point();
			if(isPoint) {
				if(vd_c.source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) {
					a = _segments[V][ind].second / _scale + _mid;
					A = B;
				} else a = _segments[V][ind].first / _scale + _mid;
			} else {
				a = _segments[V][ind].first / _scale + _mid;
				v = _segments[V][ind].second / _scale + _mid - a;
				l = std::sqrt(v.x*v.x + v.y*v.y);
				v /= l;
			}

			// Iteration over every polygon of the clipped triangle
			for(ClipperLib::Path &P0 : clipped) {
				int n = P0.size()+1;
				std::vector<Vec2> P;
				P.reserve(n);
				for(const ClipperLib::IntPoint &p : P0) P.emplace_back(p.X / _scale + _mid.x, p.Y / _scale + _mid.y);
				P.push_back(P[0]);
				if(isPoint) {
					if(A == 1.) vecF += vectorfieldEnergy(x, vecGrad, vecDiv, I, (J+1)%N, P);
					else if(A == 0.) vecF += vectorfieldEnergy(x, vecGrad, vecDiv, (I+N-1)%N, J, P);
					else vecF += vectorfieldEnergy(x, vecGrad, vecDiv, I, J, P);
					Vec2 g;
					f += point_fun_grad(P, a, g);
					grad(2*I) += (1. - A) * g.x;
					grad(2*I+1) += (1. - A) * g.y;
					grad(2*J) += A * g.x;
					grad(2*J+1) += A * g.y;

				} else {
					vecF += vectorfieldEnergy(x, vecGrad, vecDiv, I, J, P);
					for(Vec2 &u : P) {
						u -= a;
						u = Vec2(v.x*u.x + v.y*u.y, -v.y*u.x + v.x*u.y);
					}
					double M1a = 0., M1b = 0., M2 = 0.;
					for(int i = 1; i < n; ++i) getYMoments2(P[i], P[i-1], l, M1a, M1b, M2);
					f += .5 * M2;
					const double coeffI = ((1. - A) * M1a + (1. - B) * M1b);
					const double coeffJ = (A * M1a + B * M1b);
					grad(2*I) += v.y * coeffI;
					grad(2*I+1) -= v.x * coeffI;
					grad(2*J) += v.y * coeffJ;
					grad(2*J+1) -= v.x * coeffJ;
				}
			}
		}
	}
	const double cvtF = f;

	// Laplacian Regularization
	const double gamma = laplacian_coeff * _shape->_area / N;
	double lapF = 0.f;
	for(int i = 0; i < N; ++i) {
		int j = (i+N-1)%N, k = (i+1)%N;
		double dx = x(2*i) - .5*(x(2*j) + x(2*k));
		double dy = x(2*i+1) - .5*(x(2*j+1) + x(2*k+1));
		lapF += gamma * (dx*dx + dy*dy);
		grad(2*i) += 2. * gamma * dx;
		grad(2*j) -= gamma * dx;
		grad(2*k) -= gamma * dx;
		grad(2*i+1) += 2. * gamma * dy;
		grad(2*j+1) -= gamma * dy;
		grad(2*k+1) -= gamma * dy;
	}
	f += lapF;

	// Length Regularization
	const double L = length(x);
	const double dif = L - _L0;
	const double grad_len_coeff = 2. * length_coeff * _shape->_area * dif / (N*N);
	const double lenF = .5 * dif * grad_len_coeff;
	f += lenF;
	for(int i = 0; i < N; ++i) {
		const int j = (i+1)%N;
		double dx = x(2*j)-x(2*i), dy = x(2*j+1)-x(2*i+1);
		const double l = std::sqrt(dx*dx + dy*dy);
		dx *= grad_len_coeff / l;
		dy *= grad_len_coeff / l;
		grad(2*i) -= dx;
		grad(2*i+1) -= dy;
		grad(2*j) += dx;
		grad(2*j+1) += dy;
	}

	// Objective
	const double h = _shape->_area / _L0;
	const double cvtApprox = .1 * h*h;
	const double coeff = obj_coeff * cvtApprox;
	if(vecDiv > 1e-5) {
		const double vecCoeff = 2. * coeff * _vecArea;
		vecF *= vecCoeff / vecDiv;
		for(int i = 0; i < 2*N; ++i) grad(i) += vecCoeff * vecGrad[i] / vecDiv;
		f += vecF;
	}
	const double isoF = isotropyEnergy(x, grad, coeff);
	f += isoF;

	if(f < _prevF) {
		std::cout << "Energy: " << f << "   (cvt: " << cvtF << "  Lap: " << lapF << "  len: " << lenF << "  vec: " << vecF << "  iso: " << isoF << ")" << std::endl;
		std::vector<glm::vec2> points(N);
		std::vector<std::vector<int>> links(N);
		if(f > 70.) {
			for(int i = 0; i < N; ++i) {
				points[i].x = .5 * (x(2*i  ) + _prevX(2*i  ));
				points[i].y = .5 * (x(2*i+1) + _prevX(2*i+1));
				links[i] = {(i+1)%N, (i+N-1)%N};
			}
			// Window::add2Q(_layerIndex, points, links, "geom");
		}
		for(int i = 0; i < N; ++i) {
			points[i].x = x(2*i);
			points[i].y = x(2*i+1);
			links[i] = {(i+1)%N, (i+N-1)%N};
		}
		// Window::add2Q(_layerIndex, points, links, "geom");
		_prevF = f;
		_prevX = x;
	}
	return f;
}

void Smoother::optimize(Eigen::VectorXd &x) {
	_prevX.resize(x.size());

	// smooth
	const auto check = [&](const glm::vec2 &a, int i)->bool {
		int jb = (i + x.size() - 2) % x.size(), jc = (i + 2) % x.size();
		const glm::vec2 b(_prevX(jb), _prevX(jb+1)), c(_prevX(jc), _prevX(jc+1));
		if(_shape->intersect(a, b) || _shape->intersect(a, c)) return false;
		for(int k = 0; k < (int) x.size(); k += 2) if(k != i) {
			const int k2 = (k+2) % x.size();
			if(k2 == i) continue;
			const glm::vec2 d(_prevX(k), _prevX(k+1)), e(_prevX(k2), _prevX(k2+1));
			if(k != jb && k2 != jb && Globals::intersect(a, d, b, e)) return false;
			if(k != jc && k2 != jc && Globals::intersect(a, d, c, e)) return false;
		}
		return true;
	};
	for(int step = 0; step < 5; ++step) {
		_prevX = x;
		for(int i = 0; i < x.size(); i += 2) {
			const glm::vec2 a(
					(x((i+x.size()-2)%x.size()) + 10.*x(i) + x((i+2)%x.size())) / 12.,
					(x((i+x.size()-1)%x.size()) + 10.*x(i+1) + x((i+3)%x.size())) / 12.
			);
			if(check(a, i)) {
				_prevX(i) = a.x;
				_prevX(i+1) = a.y;
			}
		}
		std::swap(x, _prevX);
	}

	_prevX = x;
	_prevF = std::numeric_limits<double>::max();

	LBFGSpp::LBFGSParam<double> param;
	param.m = 7;
	param.max_linesearch = 40;
	param.epsilon = 1e-3f;
	param.max_iterations = 90 + .5 * std::sqrt(x.size());
	_L0 = length(x);
	try {
		double f;
		LBFGSpp::LBFGSSolver<double>(param).minimize(*this, x, f);
	} catch(const std::runtime_error &e) {
		std::cerr << "[Smoother Runtime Error]: " << e.what() << std::endl;
	} catch(const std::logic_error &e) {
		std::cerr << "[Smoother Logic Error]: " << e.what() << std::endl;
	} catch(const self_intersection_error &e) {
		std::cerr << "[Smoother Self Intersection Error]: " << e.what() << std::endl;
	}
	std::cerr << "Optimized" << std::endl;
	x = _prevX;
}

void Smoother::computeRadii(const Eigen::VectorXd &x, std::vector<glm::vec3> &path) {
	int N = x.size()/2;

	path.clear();
	path.reserve(N);
	Box box = _shape_box;
	for(int i = 0; i < N; ++i) {
		path.emplace_back(x(2*i), x(2*i+1), 0.f);
		box.update(Vec2(x(2*i), x(2*i+1)));
	}
	updateScale(box);

	std::vector<Segment> segments(N);
	for(int i = 0; i < N; ++i) {
		const int j = (i+1) % N;
		segments[i].first.x =  _scale * path[i].x - _mid.x;
		segments[i].first.y =  _scale * path[i].y - _mid.y;
		segments[i].second.x = _scale * path[j].x - _mid.x;
		segments[i].second.y = _scale * path[j].y - _mid.y;
	}
	updateClipBorder();
	VD vd;
	boost::polygon::construct_voronoi(_points.begin(), _points.end(), segments.begin(), segments.end(), &vd);

	// Iterate over cells
	for(const VD::cell_type &vd_c : vd.cells()) {
		if(vd_c.is_degenerate()) continue;
		uint ind = vd_c.source_index();
		if(ind < 4) continue;
		ind -= 4;

		// Create polygon cell
		ClipperLib::Paths ps;
		Eigen::VectorXd grad;
		cell2Clipper(vd_c, 10, ps, _points, segments, x, grad);
		ClipperLib::Paths clipped = clip(ps);

		if(vd_c.contains_point() && vd_c.source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) ind = (ind+1)%N;
		for(ClipperLib::Path &P : clipped) {
			ClipperLib::cInt area = (P[0].X - P.back().X) * (P[0].Y + P.back().Y);
			for(int i = 1; i < (int) P.size(); ++i) area += (P[i].X - P[i-1].X) * (P[i].Y + P[i-1].Y);
			const double A = - area / (2. * _scale * _scale);
			if(vd_c.contains_point()) {
				path[ind].z += 0.5 * A;
				path[(ind+1)%N].z += 0.5 * A;
			} else path[(ind+1)%N].z += A;
		}
	}

	std::cerr << "Radii computed" << std::endl;
}