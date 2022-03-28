//
// Created by adrien_bedel on 25/09/19.
//

#ifndef HAMILTON_WINDOW_H
#define HAMILTON_WINDOW_H

#include "managers/GeneticAlgorithm.h"
#include <mutex>
#include <queue>

class Window {
public:
	Window(int argc, char **argv, GeneticAlgorithm* ga);

	void start();
	inline static void add2Q(int layerIndex, const std::vector<glm::vec2> &points, const std::vector<std::vector<int>> &links) {
		currentInstance->_qLock.lock();
		currentInstance->_q.emplace(layerIndex, points, links);
		currentInstance->_qLock.unlock();
	}
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

	std::mutex _qLock;
	std::queue<std::tuple<int, std::vector<glm::vec2>, std::vector<std::vector<int>>>> _q;

private:
	void                    display();
	void                    displayCycle();
	void                    keyPressed(unsigned char , int , int);
	void                    mouseClicked(int, int, int, int);
	void                    mouseMoved(int, int);
	void					screenShot(int layerIndex, const std::vector<glm::vec2> &points, const std::vector<std::vector<int>> &links);
	static void             drawCallback();
	static void             keyCallback(unsigned char, int , int );
	static void             mouseCallback(int, int, int, int);
	static void             motionCallback(int, int);

	glm::vec2               getCoord(int, int);
	void                    replaceCenter();

};

#endif //HAMILTON_WINDOW_H
