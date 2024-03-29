
//
// Created by adrien_bedel on 25/09/19.
//

#include "graphics/Window.h"
#include "graphics/DirectionField.h"

#include <GL/freeglut.h>
#include <iostream>

const double zoom_step = 1.1f;

#define COLOR_INT(color) glColor3f( ((color) & 0xff) / 255., (((color) >> 8) & 0xff) / 255., (((color) >> 16) & 0xff) / 255. )
#define COLOR_INT2(color) glm::dvec3( ((color) & 0xff) / 255., (((color) >> 8) & 0xff) / 255., (((color) >> 16) & 0xff) / 255. )

////////////////////////
//
//      Window Functions
//
///////////////////////

// main
Window::Window(int argc, char **argv, GeneticAlgorithm *ga): _ga(ga) {
	currentInstance = this;
	glutInit(&argc, argv);

	// Initialize GLUT
	const int num_pix = 600000;
	int w = std::sqrt((num_pix * Globals::_SVGSize.x) / Globals::_SVGSize.y);
	int h = num_pix / w;
	glutSetOption(GLUT_MULTISAMPLE, 4);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutInitWindowSize(w, h);   // Set the window's initial width & height
	glutCreateWindow("Space Filling Curve"); // Create a window with the given title

	glutKeyboardFunc(keyCallback);
	glutMouseFunc(mouseCallback);
	glutMotionFunc(motionCallback);
	glutDisplayFunc(Window::drawCallback);
}

void Window::start() { glutMainLoop(); }

void Window::drawCallback() { currentInstance->display(); }
void Window::keyCallback(unsigned char key, int x, int y) { currentInstance->keyPressed(key, x, y); }
void Window::mouseCallback(int button, int state, int x, int y) { currentInstance->mouseClicked(button, state, x, y); }
void Window::motionCallback(int x, int y) { currentInstance->mouseMoved(x, y); }

void Window::printHelp() {
	std::cout << "=== Window HELP ===\n\n";
	std::cout << "- Use the mouse wheel to zoom in and out\n";
	std::cout << "- Use the mouse left button to move when you have zoomed in\n\n";

	std::cout << "- Use E/R to switch to the next/previous layer when the input contains several layers\n";
	std::cout << "- Use S/D to change graph mode -- Cycle / Graph / Nothing --\n";
	std::cout << "- Use X to enable/disable the vector field display\n";
	std::cout << "- Use C to switch between the print color mode and the objective color mode\n";
	std::cout << "   BLACK=NOTHING    GREY=ANISOTROPY    BLUE=ISOTROPY\n";
	std::cout << "   RED=BORDER_ALIGMENT    GREEN=ORTHOGONAL_BORDER_ALIGMENT\n";
	std::cout << "   YELLOW=BORDER_ALIGMENT    CYAN=ORTHOGONAL_BORDER_ALIGMENT  +++ Adding the border of the zone for aligment\n\n";
}

// key manager
void Window::keyPressed(unsigned char key, int x, int y) {
	switch (key) {
		case 'e': // go to next layer
			if(_ga->getNbReadyLayers() > 0) _layerIndex = (_layerIndex + 1) % _ga->getNbReadyLayers();
			glutPostRedisplay();
			break;
		case 'r': // go to previous layer
			if(_ga->getNbReadyLayers() > 0) _layerIndex = (_layerIndex + _ga->getNbReadyLayers() - 1) % _ga->getNbReadyLayers();
			glutPostRedisplay();
			break;
		case 's': // go to next graph mode
			_graphMode = (_graphMode+1)%3;
			glutPostRedisplay();
			break;
		case 'd': // go to previous graph mode
			_graphMode = (_graphMode+2)%3;
			glutPostRedisplay();
			break;
		case 'x':
			_showVecField = !_showVecField;
			glutPostRedisplay();
			break;
		case 'c':
			_showPrintColor = !_showPrintColor;
			glutPostRedisplay();
			break;
	}
}

void Window::replaceCenter() {
	double dist = glm::distance(_WinCenter, .5*Globals::_SVGSize);
	double max_allowed_dist = .5f*glm::length(Globals::_SVGSize) *  (1. - 1. / _zoom);
	if(dist > max_allowed_dist) _WinCenter = .5*Globals::_SVGSize + max_allowed_dist / dist * (_WinCenter - .5*Globals::_SVGSize);
}

glm::dvec2 Window::getCoord(int x0, int y0) {
	double W = (double) glutGet(GLUT_WINDOW_WIDTH);
	double H = (double) glutGet(GLUT_WINDOW_HEIGHT);
	double scale = std::min(W / Globals::_SVGSize.x, H / Globals::_SVGSize.y) * _zoom;
	glm::dvec2 p = _WinCenter;
	p.x += (x0 - .5f*W) / scale;
	p.y += (y0 - .5f*H) / scale;
	return p;
}

void Window::mouseMoved(int x0, int y0) {
	if(!_leftDown) return;
	_WinCenter += _lastClikedPos - getCoord(x0, y0);
	replaceCenter();
	glutPostRedisplay();
}

void Window::mouseClicked(int button, int state, int x0, int y0) {
	if(state == GLUT_UP) {
		if(button == GLUT_LEFT_BUTTON) {
			_leftDown = false;
		}
	} else {
		glm::dvec2 p = getCoord(x0, y0);
		if(button == GLUT_LEFT_BUTTON) {
			_lastClikedPos = p;
			_leftDown = true;
		} else if(button == GLUT_RIGHT_BUTTON) {
			std::cout << "x = " << p.x << ", y = " << p.y << std::endl;
		} else if(button == 3) {
			p.x = std::max(0., std::min(Globals::_SVGSize.x, p.x));
			p.y = std::max(0., std::min(Globals::_SVGSize.y, p.y));
			_zoom *= zoom_step;
			_WinCenter = p + (_WinCenter - p) / zoom_step;
		} else if(button == 4) {
			p.x = std::max(0., std::min(Globals::_SVGSize.x, p.x));
			p.y = std::max(0., std::min(Globals::_SVGSize.y, p.y));
			double div = std::min(zoom_step, _zoom);
			_zoom /= div;
			_WinCenter = p + (_WinCenter - p) * div;
			replaceCenter();
		}
	}
	glutPostRedisplay();
}

// Display
void Window::displayCycle() {
	const double W = (double) glutGet(GLUT_WINDOW_WIDTH);
	const double H = (double) glutGet(GLUT_WINDOW_HEIGHT);
	glLoadIdentity();
	double scale = std::min(W / Globals::_SVGSize.x, H / Globals::_SVGSize.y) * _zoom;
	if(_WinCenter.x < 0.) _WinCenter = .5 * Globals::_SVGSize;
	gluOrtho2D(_WinCenter.x - .5*W/scale, _WinCenter.x + .5*W/scale,
			   _WinCenter.y + .5*H/scale, _WinCenter.y - .5*H/scale);
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(std::min(10., 2.5 * std::pow(_zoom, 0.25)));

	const Layer &layer = _ga->_layers[_layerIndex];

	const auto drawVertex = [&](const glm::dvec2 &point){ glVertex2f(point.x, point.y); };

	// Show background
	glBegin(GL_TRIANGLES);
	for(const LocalOperator &op : layer._operators) {
		for(const Shape &zone : (_showPrintColor ? op.getColorZones() : op.getObjZones())) {
			if(_showPrintColor) COLOR_INT(zone._printColor);
			else glColor3f(COLORS[zone._objcetive].x, COLORS[zone._objcetive].y, COLORS[zone._objcetive].z);
			for(uint i : zone._triangles[0]) drawVertex(zone._points[i]);
			glColor3f(1., 1., 1.);
			for(uint i = 0; i < zone._holes.size(); ++i)
				for(uint j : zone._triangles[i+1]) drawVertex(zone._holes[i][j]);
		}
	}
	glEnd();

	// Show boundaries
	glBegin(GL_LINES);
	glColor3f(0.38f, 0.51f, 0.71f); // blue
	for(const LocalOperator &op : layer._operators) {
		for(const Shape &zone : (_showPrintColor ? op.getColorZones() : op.getObjZones())) {
			for(int i = 1; i < (int) zone._points.size(); ++i) {
				drawVertex(zone._points[i - 1]);
				drawVertex(zone._points[i]);
			}
			for(const std::vector<glm::dvec2> &hole : zone._holes) {
				for(int i = 1; i < (int) hole.size(); ++i) {
					drawVertex(hole[i - 1]);
					drawVertex(hole[i]);
				}
			}
		}
	}
	glEnd();

	// Show links
	if(_graphMode != 2) {
		for(const LocalOperator &op : layer._operators) {
			const auto [points, cLinks, oriLinks] = op.getGraph();
			const std::vector<std::vector<int>> &links = _graphMode ? oriLinks : cLinks;
			glBegin(GL_LINES);
			for(int i = 0; i < (int) links.size(); i++) for(int j : links[i]) if(i < j) for(int k : {i, j}) {
				const int z = op._zone[k];
				const glm::dvec3 background = _showPrintColor ? COLOR_INT2(op.getObjZones()[z]._printColor) : COLORS[op.getObjZones()[z]._objcetive];
				glColor3f(.5f * (20./255. + 1. - std::round(background.x)), .5f * (49./255. + 1. - std::round(background.x)), .5f * (70./255. + 1. - std::round(background.x)));
				drawVertex(points[k]);
			}
			glEnd();
		}
	}

	// Show vecfield
	if(_showVecField) {
		int nx = W / 13., ny = H / 13.;
		double dx = (W - 13. * (nx-1)) / 2. + scale*_WinCenter.x - .5f*W;
		double dy = (H - 13. * (ny-1)) / 2. + scale*_WinCenter.y - .5f*H;
		glLineWidth(1.8f);
		glBegin(GL_LINES);
		for(int i = 0; i < nx; ++i) {
			for(int j = 0; j < ny; ++j) {
				double cx = (dx + 13.*i) / scale;
				double cy = (dy + 13.*j) / scale;
				glm::dvec2 p(cx, cy);
				glm::dvec2 v = DirectionField::getVecAtPos(p, _layerIndex);
				double len = glm::length(v);
				double angle = std::atan2(v.y, v.x) / 2.;
				double c = 5.7f * len * std::cos(angle) / scale;
				double s = 5.7f * len * std::sin(angle) / scale;
				glColor3f(.9f, .9f, .9f);
				glVertex2f(cx - c, cy - s);
				glColor3f(.1f, .1f, .1f);
				glVertex2f(cx + c, cy + s);
			}
		}
		glEnd();
	}
}

void Window::display() {
	glClearColor(1., 1., 1., 1.);   // Set background color to black and opaque
	glClear(GL_COLOR_BUFFER_BIT);       // Clear the color buffer (background)
	if(_ga->getNbReadyLayers() > 0) displayCycle();
	glutSwapBuffers();
}

