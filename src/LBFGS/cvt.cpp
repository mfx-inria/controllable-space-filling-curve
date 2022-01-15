#include "LBFGS/cvt.hpp"
#include "LBFGS.h"
#include <set>
#include <iostream>
#include <fstream>

const bool printLineSearchError = false;

namespace boost::polygon {
    template<> struct geometry_concept<Vec2> { typedef point_concept type; };
    template<> struct point_traits<Vec2> {
        typedef double coordinate_type;
        static inline coordinate_type get(const Vec2& v, orientation_2d orient) {
            return (orient == HORIZONTAL) ? v.x : v.y;
        }
    };
    template<> struct geometry_concept<Segment> { typedef segment_concept type; };
    template<> struct segment_traits<Segment> {
        typedef double coordinate_type;
        typedef Vec2 point_type;
        static inline point_type get(const Segment& segment, direction_1d dir) {
            return dir.to_int() ? segment.second : segment.first;
        }
    };
}

struct vec2Comp {
    bool operator()(const glm::vec2 &a, const glm::vec2 &b) const {
        if(a.x < b.x) return true;
        return a.x == b.x && a.y < b.y;
    }
};

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

inline void getYMoments1(const Vec2 &a, const Vec2 &b, double &M1, double &M2) {
    double dx = b.x - a.x;
    double bb = b.y*b.y, ba = b.y*a.y, aa = a.y*a.y;
    M1 += dx * (bb + ba + aa) / 6.;
    M2 += dx * (b.y+a.y) * (bb+aa) / 12.;
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

inline void addCurvedEdge(uint ind, const VD::edge_type *e, const std::vector<Segment> &segments, ClipperLib::Path &p, int ni) {
    if(e != nullptr && e->is_curved() && e->twin()->cell()->source_index() >= 4 && e->vertex1() != nullptr) {
        Vec2 a, b, c;
        uint ind2 = e->twin()->cell()->source_index()-4;
        if(e->cell()->contains_point()) {
            a = segments[ind2].first;
            b = segments[ind2].second;
            if(e->cell()->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) c = segments[ind].second;
            else c = segments[ind].first;
        } else {
            a = segments[ind].first;
            b = segments[ind].second;
            if(e->twin()->cell()->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) c = segments[ind2].second;
            else c = segments[ind2].first;
        }
        b -= a;
        b /= std::sqrt(b.x*b.x + b.y*b.y);
        Vec2 b2(-b.y, b.x);
        c -= a;
        c = Vec2(c.x*b.x + c.y*b.y, c.x*b2.x + c.y*b2.y);
        double x0 = (e->vertex1()->x() - a.x)*b.x + (e->vertex1()->y() - a.y)*b.y;
        double x1 = (e->vertex0()->x() - a.x)*b.x + (e->vertex0()->y() - a.y)*b.y;
        x1 -= x0;
        for(int i = 1; i < ni; ++i) {
            double x = x0 + (x1 * i) / ni;
            double dx = x - c.x;
            double y = .5 * (c.y + dx*dx/c.y);
            p.emplace_back(std::round(a.x + x*b.x + y*b2.x)+1.e-6, std::round(a.y + x*b.y + y*b2.y)+1.e-6);
        }
    }
}

inline void addCurvedEdge(uint ind, const VD::edge_type *e, const std::vector<Vec2> &points, const std::vector<Segment> &segments, ClipperLib::Path &p, int ni) {
    if(e->is_curved() && e->twin()->cell()->source_index() >= 4) {
        const boost::polygon::voronoi_vertex<double> *start = e->vertex1(), *end = e->vertex0();
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
        Vec2 b2(-b.y, b.x);
        c -= a;
        c = Vec2(c.x*b.x + c.y*b.y, c.x*b2.x + c.y*b2.y);
        double x0 = (start->x() - a.x)*b.x + (start->y() - a.y)*b.y;
        double x1 = (end->x() - a.x)*b.x + (end->y() - a.y)*b.y;
        x1 -= x0;
        for(int i = 1; i < ni; ++i) {
            double x = x0 + (x1 * i) / ni;
            double dx = x - c.x;
            double y = .5 * (c.y + dx*dx/c.y);
            p.emplace_back(std::round(a.x + x*b.x + y*b2.x) + 1.e-5, std::round(a.y + x*b.y + y*b2.y) + 1.e-5);
        }
    }
}

//////////////////
//
//   POINT CVT
//
//////////////////

PointCVT::PointCVT(const std::vector<glm::vec2> *border, const std::vector<std::vector<glm::vec2>> *holes):
        _outer(border), _inners(holes) {
    _border.resize(1 + _inners->size());
    _border[0].resize(_outer->size()-1);
    for(int i = 0; i < _inners->size(); ++i)
        _border[i+1].resize((*_inners)[i].size()-1);
    for(const glm::vec2 &p : *border) _border_box.update(p);
}

void PointCVT::construct_voro(const Eigen::VectorXd &x, VD *vd) {
    int N = x.size()/2;

    // compute box
    Box box = _border_box;
    for(int i = 0; i < N; ++i) box.update(Vec2(x(2*i), x(2*i+1)));
    _mid = (box.min() + box.max()) / 2.;
    _scale = INT32_MAX / (2.5 * std::max(box.W(), box.H()));

    // Convert x to a vector of points
    _points.resize(N+4);
    for(int i = 0; i < N; ++i) {
        _points[i].x = _scale * (x(2*i) - _mid.x);
        _points[i].y = _scale * (x(2*i+1) - _mid.y);
    }

    // Usefull to obtain bounded cells
    _points[N] = Vec2(-INT32_MAX, -INT32_MAX);
    _points[N+1] = Vec2(-INT32_MAX, INT32_MAX);
    _points[N+2] = Vec2(INT32_MAX, -INT32_MAX);
    _points[N+3] = Vec2(INT32_MAX, INT32_MAX);
    boost::polygon::construct_voronoi(_points.begin(), _points.end(), vd);

    for (uint i = 0; i < _border[0].size(); ++i) {
        _border[0][i].X = std::round(_scale * ((*_outer)[i].x - _mid.x));
        _border[0][i].Y = std::round(_scale * ((*_outer)[i].y - _mid.y));
    }
    for (uint i = 0; i < _inners->size(); ++i) {
        for (uint j = 0; j < _border[i+1].size(); ++j) {
            _border[i+1][j].X = std::round(_scale * ((*_inners)[i][j].x - _mid.x));
            _border[i+1][j].Y = std::round(_scale * ((*_inners)[i][j].y - _mid.y));
        }
    }
}

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
        oriLink[i].clear();
        oriLink[i].push_back((i+1)%N);
        oriLink[i].push_back((i+N-1)%N);
        cLink[i] = oriLink[i];
    }

    VD vd;
    construct_voro(x, &vd);
    for(const VD::cell_type &vd_c : vd.cells()) {
        uint i = vd_c.source_index();
        if(i >= N) continue;
        const VD::edge_type *e = vd_c.incident_edge();
        do {
            uint j = e->twin()->cell()->source_index();
            if(j < N && std::find(oriLink[i].begin(), oriLink[i].end(), j) == oriLink[i].end()) {
                bool good = true;
                for(int k = 0; k < N; ++k) if(k != i && k != j) {
                        int l = (k+1)%N;
                        if(l != i && l != j && Globals::intersect(points[i], points[k], points[j], points[l])) {
                            good = false;
                            break;
                        }
                    }
                if(good) {
                    for(int k = 0; k < _outer->size(); ++k) {
                        int l = (k+1)%_outer->size();
                        if(Globals::intersect(points[i], (*_outer)[k], points[j], (*_outer)[l])) {
                            good = false;
                            break;
                        }
                    }
                    if(good) {
                        for(const std::vector<glm::vec2> &in : *_inners) {
                            for(int k = 0; k < in.size(); ++k) {
                                int l = (k+1)%in.size();
                                if(Globals::intersect(points[i], in[k], points[j], in[l])) {
                                    good = false;
                                    break;
                                }
                            }
                            if(!good) break;
                        }
                        if(good) {
                            oriLink[i].push_back(j);
                            oriLink[j].push_back(i);
                        }
                    }
                }
            }
            e = e->prev();
        } while(e != vd_c.incident_edge());
    }
}

//////////////////
//
//   SEGMENT CVT
//
//////////////////

SegmentCVT::SegmentCVT(const Shape *boundary, const Box &box, int layerIndex):
        _border_box(box), _boundary(boundary), _layerIndex(layerIndex) {
    _border.resize(1 + _boundary->_holes.size());
    _border[0].resize(_boundary->_points.size()-1);
    for(int i = 0; i < _boundary->_holes.size(); ++i)
        _border[i+1].resize(_boundary->_holes[i].size()-1);
}

void SegmentCVT::updateBorder() {
    for(uint i = 0; i < _border[0].size(); ++i) {
        _border[0][i].X = _scale * (_boundary->_points[i].x - _mid.x);
        _border[0][i].Y = _scale * (_boundary->_points[i].y - _mid.y);
    }
    for(uint i = 0; i < _boundary->_holes.size(); ++i)
        for(uint j = 0; j < _border[i+1].size(); ++j) {
            _border[i+1][j].X = _scale * (_boundary->_holes[i][j].x - _mid.x);
            _border[i+1][j].Y = _scale * (_boundary->_holes[i][j].y - _mid.y);
        }
}

void SegmentCVT::construct_voro(const Eigen::VectorXd &x, VD *vd) {
    int N = x.size()/2;

    ////////////////
    // FIRST VORONOI
    ////////////////

    // compute box
    Box box = _border_box;
    for(int i = 0; i < N; ++i) box.update(Vec2(x(2*i), x(2*i+1)));
    _mid = (box.min() + box.max()) / 2.;
    _scale = INT32_MAX / (2.5 * std::max(box.W(), box.H()));

    // Convert x to a vector of points
    _points.resize(N+4);
    for(int i = 0; i < N; ++i) {
        _points[i+4].x = _scale * (x(2*i) - _mid.x);
        _points[i+4].y = _scale * (x(2*i+1) - _mid.y);
    }

    // Compute CVT
    boost::polygon::construct_voronoi(_points.begin(), _points.end(), vd);
    updateBorder();

    // Compute needles
    std::vector<Vec2> needles(N, Vec2(0., 0.));
    double multArea = .5 * (Image::_imgWidth * Image::_imgHeight) / (Globals::_SVGSize.x * Globals::_SVGSize.y);
    for(const VD::cell_type &vd_c : vd->cells()) {
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
        ClipperLib::Paths clipped;
        ClipperLib::Clipper cl;
        cl.AddPaths(ps, ClipperLib::ptSubject, true);
        cl.AddPaths(_border, ClipperLib::ptClip, true);
        cl.Execute(ClipperLib::ctIntersection, clipped, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        double area = 0.;
        std::vector<std::vector<Vec2>> Ps;
        for(ClipperLib::Path &P0 : clipped) {
            size_t n = P0.size()+1;
            Ps.emplace_back();
            std::vector<Vec2> &P = Ps.back();
            P.reserve(n);
            for(const ClipperLib::IntPoint &p : P0) P.emplace_back(p.X / _scale + _mid.x, p.Y / _scale + _mid.y);
            P.push_back(P[0]);
            for(int i = 1; i < n; ++i) {
                needles[ind] += Image::getVecUnder(P[i], P[i-1], _layerIndex);
                area += (P[i-1].x - P[i].x) * (P[i].y + P[i-1].y);
            }
        }
        if(area <= 1e-6) {
            needles[ind].x = 0.;
            needles[ind].y = 0.;
            continue;
        }
        area *= multArea;
        double l = std::min(1., glm::length(needles[ind]) / area);
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
            for(int i = 1; i < P.size(); ++i) {
                Vec2 e = P[i-1] - P[i];
                double div = needles[ind].x * e.y - needles[ind].y * e.x;
                if(div == 0.) continue;
                double t = (P[i].x - a.x) * e.y - (P[i].y - a.y) * e.x;
                double u;
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

    // compute box
    for(uint i = 0; i < _segments_ind.size(); ++i) {
        uint j = _segments_ind[i];
        _segments[i].first.x = x(2*j)  - needles[j].x;
        _segments[i].first.y = x(2*j+1) - needles[j].y;
        _segments[i].second.x = x(2*j) + needles[j].x;
        _segments[i].second.y = x(2*j+1) + needles[j].y;
        box.update(_segments[i].first);
        box.update(_segments[i].second);
    }
    _mid = (box.min() + box.max()) / 2.;
    _scale = INT32_MAX / (2.5 * std::max(box.W(), box.H()));

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
    updateBorder();
}

inline double SegmentCVT::foundError(const Eigen::VectorXd &x, Eigen::VectorXd &grad, const char* msg) {
    if(prevF != std::numeric_limits<double>::max()) {
        grad = x - prevX;
        double len = grad.norm();
        grad *= 100./len;
        return prevF + 100.*len;
    }
    throw self_intersection_error(msg);
}

double SegmentCVT::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad) {
    double f = 0;
    grad.setZero(x.size());

    // Compute VD
    VD vd;
    construct_voro(x, &vd);

    // Iterate over cells
    for(const VD::cell_type &vd_c : vd.cells()) {
        if(vd_c.is_degenerate()) continue;
        uint ind = vd_c.source_index();
        if(ind < 4) continue; // it is one of the four points added

        // Create polygon cell
        ClipperLib::Paths ps(1);
        ClipperLib::Path &p = ps[0];
        const VD::edge_type *e = vd_c.incident_edge();
        do {
            if(e->is_infinite()) return foundError(x, grad, "infinite edge");
            addCurvedEdge(ind, e, _points, _segments, p, 4);
            p.emplace_back(std::round(e->vertex0()->x()), std::round(e->vertex0()->y()));
            e = e->prev();
        } while(e != vd_c.incident_edge());
        ClipperLib::cInt area = (p[0].X - p.back().X) * (p[0].Y + p.back().Y);
        for(int i = 1; i < p.size(); ++i) area += (p[i].X - p[i-1].X) * (p[i].Y + p[i-1].Y);
        if(area < 0.) return foundError(x, grad, "negative area");

        // We clip with the border
        ClipperLib::Paths clipped;
        ClipperLib::Clipper cl;
        try {
            cl.AddPaths(ps, ClipperLib::ptSubject, true);
            cl.AddPaths(_border, ClipperLib::ptClip, true);
            cl.Execute(ClipperLib::ctIntersection, clipped, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
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
            size_t n = P0.size()+1;
            std::vector<Vec2> P;
            P.reserve(n);
            for(const ClipperLib::IntPoint &p : P0) P.emplace_back(p.X / _scale, p.Y / _scale);
            P.push_back(P[0]);
            if(vd_c.contains_point()) {
                double M0 = 0.;
                Vec2 M1(0., 0.), M2(0., 0.);
                for(int i = 1; i < n; ++i) getMomentsSeg(P[i], P[i-1], M0, M1, M2);
                Vec2 add = .5 * (M2 - 2.*M1*a + M0*a*a);
                f += add.x + add.y;
                grad(2*ind) += M0*a.x - M1.x;
                grad(2*ind+1) += M0*a.y - M1.y;
            } else {
                for(Vec2 &u : P) {
                    u -= a;
                    u = Vec2(v.x*u.x + v.y*u.y, -v.y*u.x + v.x*u.y);
                }
                double M1 = 0., M2 = 0.;
                for(int i = 1; i < n; ++i) getYMoments1(P[i], P[i-1], M1, M2);
                // update f and grad
                f += .5 * M2;
                grad(2*ind) += v.y * M1;
                grad(2*ind+1) -= v.x * M1;
            }
        }
    }

    if(f < prevF) {
		static int fileCount = 0;
        prevF = f;
        prevX = x;
		saveGraph("video/graph_init_" + std::to_string(++fileCount) + ".txt", getGraph(vd));
    }
    return f;
}

Graph SegmentCVT::getGraph(const Eigen::VectorXd &x) {
    VD vd;
    construct_voro(x, &vd);
	return getGraph(vd);
}

Graph SegmentCVT::getGraph(const VD &vd) {
    Graph graph;
    graph._cells.resize(_points.size() + _segments.size() - 4);

    // Iterate over cells
    std::map<glm::vec2, int, vec2Comp> m;
    float eps = 5e-5f;
    for(const VD::cell_type &vd_c : vd.cells()) {
        if(vd_c.is_degenerate()) continue;
        uint ind = vd_c.source_index();
        if(ind < 4) continue; // it is one of the four points added

        // Create polygon cell
        ClipperLib::Paths ps(1);
        ClipperLib::Path &p = ps[0];
        const VD::edge_type *e = vd_c.incident_edge();
        do {
            if(e->is_infinite()) throw self_intersection_error("infinite edge");
            p.emplace_back(std::round(e->vertex0()->x()), std::round(e->vertex0()->y()));
            e = e->prev();
        } while(e != vd_c.incident_edge());
        ClipperLib::cInt area = (p[0].X - p.back().X) * (p[0].Y + p.back().Y);
        for(int i = 1; i < p.size(); ++i) area += (p[i].X - p[i-1].X) * (p[i].Y + p[i-1].Y);
        if(area < 0.) throw self_intersection_error("negative area");

        // We clip with the border
        ClipperLib::Paths clipped;
        ClipperLib::Clipper cl;
        cl.AddPaths(ps, ClipperLib::ptSubject, true);
        cl.AddPaths(_border, ClipperLib::ptClip, true);
        cl.Execute(ClipperLib::ctIntersection, clipped, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        // Iteration over every polygon of the clipped cell
        const auto link = [&](int i, int j)->void {
            if(i == j || std::find(graph._originalLinks[i].begin(), graph._originalLinks[i].end(), j) != graph._originalLinks[i].end()) return;
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
            graph._originalLinks[i].push_back(j);
            graph._originalLinks[j].push_back(i);
        };
        std::vector<int> &cell = graph._cells[ind-4];
        for(ClipperLib::Path &P0 : clipped) {
            int start, last = -1;
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
                    graph._originalLinks.emplace_back();
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
        for(int i = 1; i < cell.size(); ++i) {
            for(int j : graph._originalLinks[cell[i-1]]) {
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

void SegmentCVT::saveGraph(const std::string &filename, const Graph &graph) {
	std::ofstream file(filename);
	file << "SEED " << _points.size()-4 << '\n';
	for(int i = 4; i < (int) _points.size(); ++i)
        file << _points[i].x / _scale + _mid.x << ' ' << _points[i].y / _scale + _mid.y << '\n';
	file << "NEEDLE " << _segments.size() << '\n';
	for(const auto &[a, b] : _segments)
        file << a.x / _scale + _mid.x << ' ' << a.y / _scale + _mid.y << ' ' << b.x / _scale + _mid.x << ' ' << b.y / _scale + _mid.y << '\n';
	file << "POINTS " << graph._points.size() << '\n';
	for(const glm::vec2 &p : graph._points)
        file << p.x << ' ' << p.y << '\n';
	size_t ne = 0;
	for(const auto link : graph._originalLinks) ne += link.size();
	ne /= 2;
	file << "EDGE " << ne << '\n';
	for(int i = 0; i < (int) graph._originalLinks.size(); ++i)
		for(int j : graph._originalLinks[i]) if(i < j)
			file << i << ' ' << j << '\n';
	file.close();
}

Graph SegmentCVT::optimize(std::vector<glm::vec2> &points) {
    _wantedMaxSize = .66 * std::sqrt(_boundary->_area / (M_PI * points.size()));
    Eigen::VectorXd x(2 * points.size());
    for(int i = 0; i < points.size(); ++i) {
        x(2*i) = points[i].x;
        x(2*i+1) = points[i].y;
    }
    _points.resize(4);
    _points[0] = Vec2(-INT32_MAX, -INT32_MAX);
    _points[1] = Vec2(-INT32_MAX, INT32_MAX);
    _points[2] = Vec2(INT32_MAX, -INT32_MAX);
    _points[3] = Vec2(INT32_MAX, INT32_MAX);

    prevX = x;
    prevF = std::numeric_limits<double>::max();

    // LBFGS
    LBFGSpp::LBFGSParam<double> param;
    param.m = 9;
    param.max_iterations = 80;
    param.epsilon = 0.05;
    param.max_linesearch = 7;
    LBFGSpp::LBFGSSolver<double> solver(param);
    double f;
    try {
        solver.minimize(*this, x, f);
    } catch(const std::runtime_error &e) {
        if(printLineSearchError) std::cerr << "[SegmentCVT Runtime Error]: " << e.what() << std::endl;
    } catch(const std::logic_error &e) {
        std::cerr << "[SegmentCVT Logic Error]: " << e.what() << std::endl;
    } catch(const self_intersection_error &e) {
        std::cerr << "[SegmentCVT Self Intersection Error]: " << e.what() << std::endl;
    }
    return getGraph(prevX);
}

//////////////////
//
//   SMOOTHER
//
//////////////////

const Vec2 Smoother::points[4] = {
        Vec2(-INT32_MAX, -INT32_MAX),
        Vec2(-INT32_MAX, INT32_MAX),
        Vec2(INT32_MAX, -INT32_MAX),
        Vec2(INT32_MAX, INT32_MAX)
};

Smoother::Smoother(const Shape *boundary, const std::vector<Shape> *zones, const std::vector<Shape> *strokeZones, double vecArea, int layerIndex):
        _boundary(boundary), _zones(zones), _strokeZones(strokeZones), _layerIndex(layerIndex), _vecArea(vecArea) {
    _borders.reserve(_strokeZones->size());
    _paths.resize(_strokeZones->size());
    _segments.resize(_strokeZones->size());
    for(const Shape &zone : *_strokeZones) {
        _borders.emplace_back();
        _borders.back().resize(1 + zone._holes.size());
        _borders.back()[0].resize(zone._points.size());
        for(int i = 0; i < zone._holes.size(); ++i)
            _borders.back()[i+1].resize(zone._holes[i].size());
    }
    for(const glm::vec2 &v : _boundary->_points)
        _border_box.update(v);
}

void Smoother::construct_voro(const Eigen::VectorXd &x) {
    int N = x.size()/2;
    Box box = _border_box;
    Vec2 b(x(0), x(1));
    int k = 0;
    while(k < _strokeZones->size() && !(*_strokeZones)[k].isInside(b)) ++k;
    if(k >= _strokeZones->size()) k = -1;
    for(std::vector<std::tuple<int, double, double>> &p : _paths) p.clear();
    for(int i = 0; i < N; ++i) {
        int j = (i+1)%N;
        Vec2 a(x(2*j), x(2*j+1));
        std::swap(a, b);
        box.update(a);
        std::vector<float> ts;
        if(k >= 0) {
            Globals::getInter(a, b, (*_strokeZones)[k], ts);
            if(ts.empty()) {
                _paths[k].emplace_back(i, 0., 1.);
                continue;
            }
            ts.push_back(0.f);
        } else Globals::getInter(a, b, (*_strokeZones)[k = 0], ts);
        int l = k, k2 = -1;
        while(true) {
            if(ts.size()&1) {
                k2 = l;
                ts.push_back(1.f);
            }
            std::sort(ts.begin(), ts.end());
            for(int m = 0; m < ts.size(); m += 2)
                _paths[l].emplace_back(i, ts[m], ts[m+1]);
            ts.clear();
            do {
                l = (l+1) % _strokeZones->size();
                if(l == k) break;
                Globals::getInter(a, b, (*_strokeZones)[l], ts);
            } while(ts.empty());
            if(l == k) break;
        }
        k = k2;
    }
    _mid = (box.min() + box.max()) / 2.;
    _scale = INT32_MAX / (2.5 * std::max(box.W(), box.H()));

    // compute segments
    for(k = 0; k < _strokeZones->size(); ++k) {
        _segments[k].clear();
        _segments[k].reserve(_paths[k].size());
        for(const auto &[i, a, b] : _paths[k]) {
            int j = (i+1) % N;
            _segments[k].emplace_back(Vec2(_scale * ((1.-a)*x(2*i) + a*x(2*j) - _mid.x), _scale * ((1.-a)*x(2*i+1) + a*x(2*j+1) - _mid.y)),
                                      Vec2(_scale * ((1.-b)*x(2*i) + b*x(2*j) - _mid.x), _scale * ((1.-b)*x(2*i+1) + b*x(2*j+1) - _mid.y)));
        }

        for(uint i = 0; i < _borders[k][0].size(); ++i) {
            _borders[k][0][i].X = _scale * ((*_strokeZones)[k]._points[i].x - _mid.x);
            _borders[k][0][i].Y = _scale * ((*_strokeZones)[k]._points[i].y - _mid.y);
        }
        for(uint i = 0; i < (*_strokeZones)[k]._holes.size(); ++i)
            for(uint j = 0; j < _borders[k][i+1].size(); ++j) {
                _borders[k][i+1][j].X = _scale * ((*_strokeZones)[k]._holes[i][j].x - _mid.x);
                _borders[k][i+1][j].Y = _scale * ((*_strokeZones)[k]._holes[i][j].y - _mid.y);
            }
    }
}

inline double Smoother::vectorfieldEnergy(const Eigen::VectorXd &x, std::vector<double> &grad, double &div, int j, int k, std::vector<Vec2> &P) {
    Vec2 V(0., 0.), vecAB(x(2*k)-x(2*j), x(2*k+1)-x(2*j+1));
    for(int i = 1; i < P.size(); ++i) V += Image::getVecUnder(P[i], P[i-1], _layerIndex);
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

    for(const Shape &zone : *_zones) if(IS_ISOTROPY(zone._fillColor)) {
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
                Globals::getInter(a, b, zone, ts);
                if(ts.empty()) continue;
                std::sort(ts.begin(), ts.end());
                if(ts.size()&1) ts.push_back(1.f);
                b -= a;
                double angle = std::atan2(b.y, b.x);
                if(angle >= M_PI_2) angle -= M_PI;
                else if(angle < -M_PI_2) angle += M_PI;
                double len = 0.;
                for(int k = 0; k < ts.size(); k += 2)
                    len += ts[k+1] - ts[k];
                len *= glm::length(b);
                alpha += len * angle;
                L += len;
                distrib.emplace_back(angle, len, i);
            }

            if(distrib.empty()) {
                if(zone._fillColor == ISOTROPY) f += coeffZ * M_PI*M_PI*M_PI/4.;
                continue;
            }
            if(zone._fillColor == ANISOTROPY) f += coeffZ * M_PI*M_PI*M_PI/4.;
            else coeffZ *= std::max(1., .0123 * zone._area / _boundary->_area * N);

            alpha /= L;
            alpha -= M_PI_2;
            std::sort(distrib.begin(), distrib.end());

            double t = 0.;
            double pi_L = M_PI/L;
            for(const auto &[angle, len, i] : distrib) {
                double u0 = pi_L*t + alpha - angle;
                t += len;
                double u1 = pi_L*t + alpha - angle;
                double u02 = u0*u0;
                double u12 = u1*u1;
                if(zone._fillColor == ISOTROPY) f += coeffZ * (u1*u12 - u0*u02);
                else f -= coeffZ * (u1*u12 - u0*u02);

                int j = (i+1) % N;
                double vx = x(2*j)-x(2*i);
                double vy = x(2*j+1)-x(2*i+1);
                double diff2 = 3. * coeffZ * (u12 - u02) / (vx*vx + vy*vy);
                double dx = diff2 * vy;
                double dy = - diff2 * vx;
                if(zone._fillColor == ANISOTROPY) {
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

inline double Smoother::foundError(const Eigen::VectorXd &x, Eigen::VectorXd &grad, const char* msg) {
    if(_prevF != std::numeric_limits<double>::max()) {
        grad = x - _prevX;
        double len = grad.norm();
        grad *= 100./len;
        return _prevF + 100.*len;
    }
    throw self_intersection_error(msg);
}

inline void saveCycle(Eigen::VectorXd &x) {
	static int fileCount = 0;
	std::ofstream file("video/geom_opt_" + std::to_string(++fileCount) + ".txt");
	file << "POINTS " << x.size()/2 << '\n';
	for(int i = 0; i < (int) x.size(); i += 2)
		file << x(i) << ' ' << x(i+1) << '\n';
	file.close();
}

double Smoother::operator()(Eigen::VectorXd &x, Eigen::VectorXd &grad) {
    const int N = x.size()/2;

    double f = 0;
    grad.setZero(x.size());

    double vecF = 0.;
    double vecDiv = 0.;
    std::vector<double> vecGrad(x.size(), 0.);

    // Construct VD
    construct_voro(x);

    // Iterate over cells
    for(int V = 0; V < _strokeZones->size(); ++V) {
        VD vd;
        boost::polygon::construct_voronoi(points, points+4, _segments[V].begin(), _segments[V].end(), &vd);
        for(const VD::cell_type &vd_c : vd.cells()) {
            if(vd_c.source_category() == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) continue; // it is one of the four points added
            if(vd_c.is_degenerate()) continue;
            uint ind = vd_c.source_index();
            if(ind < 4) continue;
            ind -= 4;
            // Create polygon cell
            ClipperLib::Paths ps(1);
            ClipperLib::Path &p = ps[0];
            const VD::edge_type *e = vd_c.incident_edge();
            int countEloop = 0;
            do {
                if(++ countEloop > 1000) return foundError(x, grad, "infinite looping over edges");
                if(e->is_infinite()) return foundError(x, grad, "infinite edge");
                addCurvedEdge(ind, e, _segments[V], p, 10);
                p.emplace_back(std::round(e->vertex0()->x()), std::round(e->vertex0()->y()));
                e = e->prev();
            } while(e != vd_c.incident_edge());
            ClipperLib::cInt area = (p[0].X - p.back().X) * (p[0].Y + p.back().Y);
            for(int i = 1; i < p.size(); ++i) area += (p[i].X - p[i-1].X) * (p[i].Y + p[i-1].Y);
            if(area < 0.) return foundError(x, grad, "negative area");

            // We clip with the border
            ClipperLib::Paths clipped;
            ClipperLib::Clipper cl;
            try {
                cl.AddPaths(ps, ClipperLib::ptSubject, true);
                cl.AddPaths(_borders[V], ClipperLib::ptClip, true);
                cl.Execute(ClipperLib::ctIntersection, clipped, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            } catch(const ClipperLib::clipperException &e) {
                return foundError(x, grad, "clipper error");
            }

            auto [I, A, B] = _paths[V][ind];
            int J = (I+1) % N;
            Vec2 a, v;
            double l;
            bool isPoint = vd_c.contains_point();
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
                size_t n = P0.size()+1;
                std::vector<Vec2> P;
                P.reserve(n);
                for(const ClipperLib::IntPoint &p : P0) P.emplace_back(p.X / _scale + _mid.x, p.Y / _scale + _mid.y);
                P.push_back(P[0]);
                if(isPoint) {
                    if(A == 1.) vecF += vectorfieldEnergy(x, vecGrad, vecDiv, I, (J+1)%N, P);
                    else if(A == 0.) vecF += vectorfieldEnergy(x, vecGrad, vecDiv, (I+N-1)%N, J, P);
                    else vecF += vectorfieldEnergy(x, vecGrad, vecDiv, I, J, P);
                    double M0 = 0.;
                    Vec2 M1(0., 0.), M2(0., 0.);
                    // Iteration over every edges of this polygon
                    for(int i = 1; i < n; ++i) getMomentsSeg(P[i], P[i-1], M0, M1, M2);
                    // update f and grad
                    Vec2 add = .5 * (M2 - 2.*M1*a + M0*a*a);
                    f += add.x + add.y;
                    double gx = M0*a.x - M1.x;
                    double gy = M0*a.y - M1.y;
                    grad(2*I) += (1. - A) * gx;
                    grad(2*I+1) += (1. - A) * gy;
                    grad(2*J) += A * gx;
                    grad(2*J+1) += A * gy;

                } else {
                    vecF += vectorfieldEnergy(x, vecGrad, vecDiv, I, J, P);
                    for(Vec2 &u : P) {
                        u -= a;
                        u = Vec2(v.x*u.x + v.y*u.y, -v.y*u.x + v.x*u.y);
                    }
                    double M1a = 0., M1b = 0., M2 = 0.;
                    for(int i = 1; i < n; ++i) getYMoments2(P[i], P[i-1], l, M1a, M1b, M2);
                    // update f and grad
                    f += .5 * M2;
                    double coeffI = ((1. - A) * M1a + (1. - B) * M1b);
                    double coeffJ = (A * M1a + B * M1b);
                    grad(2*I) += v.y * coeffI;
                    grad(2*I+1) -= v.x * coeffI;
                    grad(2*J) += v.y * coeffJ;
                    grad(2*J+1) -= v.x * coeffJ;
                }
            }
        }
    }
    double cvtF = f;

    // Laplacian Regularization
    double gamma = laplacian_coeff * _boundary->_area / N;
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
    double L = length(x);
    double dif = L - _L0;
    double grad_len_coeff = 2. * length_coeff * _boundary->_area * dif / (N*N);
    double lenF = .5 * dif * grad_len_coeff;
    f += lenF;
    for(int i = 0; i < N; ++i) {
        int j = (i+1)%N;
        double dx = x(2*j)-x(2*i), dy = x(2*j+1)-x(2*i+1);
        double l = std::sqrt(dx*dx + dy*dy);
        dx *= grad_len_coeff / l;
        dy *= grad_len_coeff / l;
        grad(2*i) -= dx;
        grad(2*i+1) -= dy;
        grad(2*j) += dx;
        grad(2*j+1) += dy;
    }

    // Objective
    double h = _boundary->_area / _L0;
    double cvtApprox = .1 * h*h;
    double coeff = obj_coeff * cvtApprox;
    if(vecDiv > 1e-5) {
        double vecCoeff = 2. * coeff * _vecArea;
        vecF *= vecCoeff / vecDiv;
        for(int i = 0; i < 2*N; ++i) grad(i) += vecCoeff * vecGrad[i] / vecDiv;
        f += vecF;
    }
    double isoF = isotropyEnergy(x, grad, coeff);
    f += isoF;

    std::cerr << "Energy: " << f << "   (cvt: " << cvtF << "  Lap: " << lapF << "  len: " << lenF << "  vec: " << vecF << "  iso: " << isoF << ")" << std::endl;
    if(f < _prevF) {
        _prevF = f;
        _prevX = x;
		saveCycle(x);
    }
    return f;
}

void Smoother::optimize(Eigen::VectorXd &x) {
    _prevX.resize(x.size());
	saveCycle(x);

    // smooth
    const auto check = [&](const glm::vec2 &a, int i)->bool {
        int jb = (i + x.size() - 2) % x.size(), jc = (i + 2) % x.size();
        const glm::vec2 b(_prevX(jb), _prevX(jb+1)), c(_prevX(jc), _prevX(jc+1));
        for(int k = 1; k < _boundary->_points.size(); ++k)
            if(Globals::intersect(a, _boundary->_points[k-1], b, _boundary->_points[k])
               || Globals::intersect(a, _boundary->_points[k-1], c, _boundary->_points[k])) return false;
        for(const std::vector<glm::vec2> &in : _boundary->_holes)
            for(int k = 1; k < in.size(); ++k)
                if(Globals::intersect(a, in[k-1], b, in[k]) || Globals::intersect(a, in[k-1], c, in[k]))
                    return false;
        for(int k = 0; k < x.size(); k += 2) if(k != i) {
                int k2 = (k+2) % x.size();
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
	saveCycle(x);

    _prevX = x;
    _prevF = std::numeric_limits<double>::max();

    LBFGSpp::LBFGSParam<double> param;
    param.m = 7;
    param.max_linesearch = 42;
    param.epsilon = 1e-5f;
    param.max_iterations = 75 + .8 * std::sqrt(x.size());
    _L0 = length(x);
    double f;
    try {
        LBFGSpp::LBFGSSolver<double>(param).minimize(*this, x, f);
    } catch(const std::runtime_error &e) {
        if(printLineSearchError) std::cerr << "[Smoother Runtime Error]: " << e.what() << std::endl;
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

    Box box = _border_box;
    for(int i = 0; i < N; ++i) box.update(Vec2(x(2*i), x(2*i+1)));
    _mid = (box.min() + box.max()) / 2.;
    _scale = INT32_MAX / (2.5 * std::max(box.W(), box.H()));

    path.clear();
    path.reserve(N);
    for(int i = 0; i < N; ++i) {
        int j = (i+1)%N;
        path.emplace_back(x(2*i), x(2*i+1), 0.f);
        Vec2 v(x(2*j) - x(2*i), x(2*j+1) - x(2*i+1));
        double l = glm::length(v);
        double s = 0.1;
        int n = l / s;
        v /= (n+1);
        /*for(int k = 1; k <= n; ++k)
            path.emplace_back(x(2*i) + k*v.x, x(2*i+1) + k*v.y, 0.f);*/
    }
    N = path.size();

    // Compute VD
    std::vector<Segment> segments(N);
    for(int i = 0; i < N; ++i) {
        int j = (i+1) % N;
        segments[i].first.x = path[i].x;
        segments[i].first.y = path[i].y;
        segments[i].second.x = path[j].x;
        segments[i].second.y = path[j].y;
        box.update(segments[i].first);
        box.update(segments[i].second);
    }
    _mid = (box.min() + box.max()) / 2.;
    _scale = INT32_MAX / (2.5 * std::max(box.W(), box.H()));

    for(Segment &seg : segments) {
        seg.first = _scale * (seg.first - _mid);
        seg.second = _scale * (seg.second - _mid);
    }

    ClipperLib::Paths border(1);
    for(const glm::vec2 &v : _boundary->_points)
        border[0].emplace_back(_scale * (v.x - _mid.x), _scale * (v.y - _mid.y));
    border[0].pop_back();
    for(const std::vector<glm::vec2> &hole : _boundary->_holes) {
        border.emplace_back();
        for(const glm::vec2 &v : hole)
            border.back().emplace_back(_scale * (v.x - _mid.x), _scale * (v.y - _mid.y));
        border.back().pop_back();
    }
    VD vd;
    boost::polygon::construct_voronoi(points, points+4, segments.begin(), segments.end(), &vd);
    // Iterate over cells
    for(const VD::cell_type &vd_c : vd.cells()) {
        if(vd_c.is_degenerate()) continue;
        if(vd_c.source_category() == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) continue; // it is one of the four points added
        uint ind = vd_c.source_index();
        if(ind < 4) continue;
        ind -= 4;

        // Create polygon cell
        ClipperLib::Paths ps(1);
        ClipperLib::Path &p = ps[0];
        const VD::edge_type *e = vd_c.incident_edge();
        do {
            if (e->vertex0() != nullptr)
            {
                addCurvedEdge(ind, e, segments, p, 10);
                p.emplace_back(std::round(e->vertex0()->x()), std::round(e->vertex0()->y()));
            }
            e = e->prev();
        } while(e != vd_c.incident_edge());

        // We clip with the border
        ClipperLib::Paths clipped;
        ClipperLib::Clipper cl;
        try {
            cl.AddPaths(ps, ClipperLib::ptSubject, true);
            cl.AddPaths(border, ClipperLib::ptClip, true);
            cl.Execute(ClipperLib::ctIntersection, clipped, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        }
        catch(const ClipperLib::clipperException &e) {
            std::cerr << "clipper Error : " << e.what() << std::endl;
            continue;
        }
        if(vd_c.contains_point() && vd_c.source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) ind = (ind+1)%N;
        for(ClipperLib::Path &P : clipped) {

            ClipperLib::cInt area = (P[0].X - P.back().X) * (P[0].Y + P.back().Y);
            for(int i = 1; i < P.size(); ++i) area += (P[i].X - P[i-1].X) * (P[i].Y + P[i-1].Y);
            double A = - area / (2. * _scale * _scale);
            if(vd_c.contains_point()) {
                path[ind].z += 0.5 * A;
                path[(ind+1)%N].z += 0.5 * A;
            } else path[(ind+1)%N].z += A;
        }
    }

    std::cerr << "Radii computed" << std::endl;
}