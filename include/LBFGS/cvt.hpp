//
//  Created by Yoann Coudert-osmont
//

#ifndef CVT_HPP
#define CVT_HPP

#include <boost/polygon/voronoi.hpp>
#include <Eigen/Core>

#include "graphics/Globals.h"
#include "LBFGS/clipper.hpp"

typedef glm::vec<2, double> Vec2;
typedef std::pair<Vec2, Vec2> Segment;
typedef boost::polygon::voronoi_diagram<double> VD;


struct self_intersection_error : public std::exception {
	const std::string m_msg;
	self_intersection_error(const char* msg):
		m_msg("An edge of the path intersects an other edge of the path (detected by: " + std::string(msg) + ")")
		{}
	const char* what() const throw() { return m_msg.c_str(); }
};


//////////////////
//
//   POINT CVT
//
//////////////////

class PointCVT
{
private:
    const std::vector<glm::vec2> *_outer;
    const std::vector<std::vector<glm::vec2>> *_inners;
	ClipperLib::Paths _border;
    Box<double> _border_box;
    double _scale;
	Vec2 _mid;
    std::vector<Vec2> _points;

    void construct_voro(const Eigen::VectorXd &x, VD *vd);

public:
    PointCVT(const std::vector<glm::vec2> *border, const std::vector<std::vector<glm::vec2>> *holes);
	void updateLink(const Eigen::VectorXd &x, std::vector<glm::vec2> &points, std::vector<std::vector<int>> &oriLink, std::vector<std::vector<int>> &cLink);
};

//////////////////
//
//  SEGMENT CVT
//
//////////////////

class SegmentCVT {
public:
    SegmentCVT(const Shape *boundary, const Box<double> &box, int layerIndex);
	Graph optimize(std::vector<glm::vec2> &points);

    double operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad);

private:
    const Shape *_boundary;
	ClipperLib::Paths _border;
    Box<double> _border_box;
    double _scale;
	Vec2 _mid;
    std::vector<Vec2> _points;
    std::vector<Segment> _segments;
	std::vector<uint> _points_ind, _segments_ind;
	double _wantedMaxSize;
	int _layerIndex;

	Eigen::VectorXd prevX;
	double prevF;

	double foundError(const Eigen::VectorXd &x, Eigen::VectorXd &grad, const char* msg);

	void updateBorder();
    void construct_voro(const Eigen::VectorXd &x, VD *vd);

    Graph getGraph(const Eigen::VectorXd &x);
    Graph getGraph(const VD &vd);
};

//////////////////
//
//   SMOOTHER
//
//////////////////

class Smoother
{
private:
	const Shape *_boundary;
    const std::vector<Shape> *_zones, *_strokeZones;
	std::vector<ClipperLib::Paths> _borders;
	std::vector<std::vector<std::tuple<int, double, double>>> _paths;
	Box<double> _border_box;
	std::vector<std::vector<Segment>> _segments;
	int _layerIndex;

	Vec2 _mid;
    double _scale, _vecArea, _L0;
    Eigen::VectorXd _prevX;
	double _prevF;

	const static Vec2 points[4];

	double foundError(const Eigen::VectorXd &x, Eigen::VectorXd &grad, const char* msg);

    void construct_voro(const Eigen::VectorXd &x);
	double vectorfieldEnergy(const Eigen::VectorXd &x, std::vector<double> &grad, double &div, int j, int k, std::vector<Vec2> &P);
	double isotropyEnergy(const Eigen::VectorXd &x, Eigen::VectorXd &grad, double coeff);

public:
	double laplacian_coeff = .125; // control smoothness
	double length_coeff = .4;
	double obj_coeff = 0.00625f;

    Smoother(const Shape *boundary, const std::vector<Shape> *zones, const std::vector<Shape> *strokeZones, double vecArea, int layerIndex);

	void optimize(Eigen::VectorXd &x);

	void computeRadii(const Eigen::VectorXd &x, std::vector<glm::vec3> &path);

    double operator()(Eigen::VectorXd &x, Eigen::VectorXd &grad);
};

#endif //CVT_H