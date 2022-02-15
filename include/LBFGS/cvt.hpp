//
//  Created by Yoann Coudert-osmont
//

#ifndef CVT_HPP
#define CVT_HPP

#include "graphics/Shape.h"
#include "LBFGS/clipper.hpp"

#include <boost/polygon/voronoi.hpp>
#include <Eigen/Core>

typedef glm::vec<2, double> Vec2;
typedef std::pair<Vec2, Vec2> Segment;
typedef boost::polygon::voronoi_diagram<double> VD;

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

struct self_intersection_error : public std::exception {
	const std::string m_msg;
	self_intersection_error(const char* msg):
		m_msg("An edge of the path intersects an other edge of the path (detected by: " + std::string(msg) + ")")
		{}
	const char* what() const throw() { return m_msg.c_str(); }
};

struct Voronoi {
public:
	Voronoi(const Shape *shape);
	Voronoi(const Shape *shape, const Box<double> &box);

protected:
	const Shape *_shape;
	Box<double> _shape_box;
	ClipperLib::Paths _clipBorder;
	double _scale;
	Vec2 _mid;

	void updateScale(const Box<double> &box);
	void updateClipBorder();
	void updatePoints(std::vector<Vec2> &points, const Eigen::VectorXd &x);

	ClipperLib::Paths clip(const ClipperLib::Paths &path);
};

struct CVT : public Voronoi {
public:
	CVT(const Shape *shape): Voronoi(shape) {}
	CVT(const Shape *shape, const Box<double> &box): Voronoi(shape, box) {}

protected:
	double _prevF;
	Eigen::VectorXd _prevX;

	double foundError(const Eigen::VectorXd &x, Eigen::VectorXd &grad, const char* msg);
	double cell2Clipper(const VD::cell_type &vd_c, int ni, ClipperLib::Paths &path,
						const std::vector<Vec2> points, const std::vector<Segment> &segments,
						const Eigen::VectorXd &x, Eigen::VectorXd &grad);
	
	static double point_fun_grad(const std::vector<Vec2> &P, const Vec2 &a, Vec2 &grad);
};

//////////////////
//
//   POINT CVT
//
//////////////////

struct PointCVT : public Voronoi {
public:
	PointCVT(const Shape *shape): Voronoi(shape) {}
	void updateLink(const Eigen::VectorXd &x, std::vector<glm::vec2> &points, std::vector<std::vector<int>> &oriLink, std::vector<std::vector<int>> &cLink);
private:
	std::vector<Vec2> _points;
};

//////////////////
//
//   SMOOTHER
//
//////////////////

class Smoother : public CVT {
public:
	double laplacian_coeff = .125;
	double length_coeff = .4;
	double obj_coeff = 0.013;

	Smoother(const Shape *shape, const std::vector<Shape> *objZones, const std::vector<Shape> *colorZones, double vecArea, int layerIndex);

	void optimize(Eigen::VectorXd &x);
	void computeRadii(const Eigen::VectorXd &x, std::vector<glm::vec3> &path);
	double operator()(Eigen::VectorXd &x, Eigen::VectorXd &grad);

private:
	const std::vector<Shape> *_objZones, *_colorZones;
	std::vector<ClipperLib::Paths> _clipZones;
	std::vector<std::vector<std::tuple<int, double, double>>> _paths;
	std::vector<std::vector<Segment>> _segments;
	int _layerIndex;
	double _vecArea, _L0;

	const static std::vector<Vec2> _points;

	void construct_voro(const Eigen::VectorXd &x);
	double vectorfieldEnergy(const Eigen::VectorXd &x, std::vector<double> &grad, double &div, int j, int k, std::vector<Vec2> &P);
	double isotropyEnergy(const Eigen::VectorXd &x, Eigen::VectorXd &grad, double coeff);
};

#endif //CVT_H