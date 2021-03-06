//
// Created by adrien_bedel on 25/09/19.
//

#ifndef HAMILTON_WINDOW_H
#define HAMILTON_WINDOW_H

#include "managers/GeneticAlgorithm.h"

class Window {
public:
	Window(int argc, char **argv, GeneticAlgorithm* ga);

	void start();
	static void printHelp();

private:
	inline static Window*  currentInstance;

	GeneticAlgorithm*		_ga;
	int						_graphMode = 0;
	bool                    _showVecField = false;
	bool                    _showPrintColor = true;
	unsigned int            _layerIndex = 0;

	glm::vec2               _WinCenter = glm::vec2(-1.f, 0.f);
	float                   _zoom = 1.;
	glm::vec2               _lastClikedPos;
	bool                    _leftDown = false;

private:
	void                    display();
	void                    displayCycle();
	void                    keyPressed(unsigned char , int , int);
	void                    mouseClicked(int, int, int, int);
	void                    mouseMoved(int, int);
	static void             drawCallback();
	static void             keyCallback(unsigned char, int , int );
	static void             mouseCallback(int, int, int, int);
	static void             motionCallback(int, int);

	glm::vec2               getCoord(int, int);
	void                    replaceCenter();

};

#endif //HAMILTON_WINDOW_H
