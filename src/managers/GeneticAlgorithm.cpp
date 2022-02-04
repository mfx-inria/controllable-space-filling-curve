//
// Created by adrien_bedel on 18/11/19.
//

#include "managers/GeneticAlgorithm.h"
#include "managers/Printer.h"
#include "graphics/Window.h"
#include <algorithm>
#include <iostream>
#include <atomic>
#include <GL/freeglut.h>

GeneticAlgorithm::GeneticAlgorithm() {
	_nbIndividuals = _nbChampion * _sonPerChamp;
}

void GeneticAlgorithm::process(const std::string &fileName, int layerNb) try {
	// Algorithm
	initLayers(fileName, layerNb);
	std::cout << "===== Graph Construction ======" << std::endl;
	initCycles();
	std::cout << "=== Combinatorial Optimizer ===" << std::endl;
	shuffle();
	std::cout << "===== Geometric Optimizer =====" << std::endl;
	optimize();

	// Write GCODE
	Printer printer(Machine::CR10S_PRO);
	std::cout << "======= Writting GCODE ========" << std::endl;
	printer.printToGcode(_layers, "colorVary", true);
	printer.printToGcode(_layers, "colorNoVary", false);
	std::cout << "============ DONE =============" << std::endl;
} catch(const CSFCerror &e) {
	std::cerr << e.what() << std::endl;
	glutLeaveMainLoop();
}

template<typename F>
bool parallelize(int n, const F &fun) {
	std::atomic<int> K = 0;
	std::atomic<bool> error = false;
	const auto f = [&]() {
		try {
			int k;
			while((k = K++) < n) fun(k);
		} catch(const CSFCerror &e) {
			error = true;
			std::cerr << e.what() << std::endl;
		}
	};
	const int T = std::min((int) Globals::_nbThread, n) - 1;
	std::vector<std::thread> threads;
	threads.reserve(T);
	for(int t = 0; t < T; ++t) threads.emplace_back(f);
	f();
	for(std::thread &t : threads) t.join();
	return error;
}

void GeneticAlgorithm::initLayers(const std::string &fileName, int nbLayer) {
	_layers.resize(nbLayer);
	_nbReadyLayers = 0;
	std::vector<bool> done(nbLayer, false);
	if(parallelize(nbLayer, [&](int k) {
		_layers[k].initLayer(str_format(fileName, k), k);
		done[k] = true;
		while(_nbReadyLayers < nbLayer && done[_nbReadyLayers]) ++ _nbReadyLayers;
	})) THROW_ERROR("An error occured in a thread of shape construction!");
	_nbReadyLayers = nbLayer;
	Window::stopRefrech();
	glutPostRedisplay();
}

void GeneticAlgorithm::initCycles() {
	if(parallelize(_layers.size(), [&](int k) {
		_layers[k].initCycle(k);
		glutPostRedisplay();
	})) THROW_ERROR("An error occured in a thread of graph construction!");
}

inline std::vector<LocalOperator>::iterator bestPath(std::vector<LocalOperator>::iterator begin, std::vector<LocalOperator>::iterator end) {
	return std::min_element(begin, end, [](const LocalOperator &a, const LocalOperator &b) {
		return a.getScore() < b.getScore();
	});
}

inline void generateNewGeneration(std::vector<LocalOperator> &population) {
	if(parallelize(population.size(), [&](int k) {
		population[k].startShuffling(GeneticAlgorithm::_multiplier, k);		
		std::cout << "resulted score = " << population[k].getScore() << std::endl;
	})) THROW_ERROR("An error occured in a thread of combinatorial optimizer!");
}

void GeneticAlgorithm::shuffle() {
	for (Layer &layer : _layers) {
		for(LocalOperator &op : layer._operators) {
			if(op.getPoints().size() < 7) continue;

			_population.clear();
			_population.assign(_nbIndividuals, op);

			generateNewGeneration(_population);
			initChampions();
			for (int j = 0; j < _genNumber; j++) {
				upgradeGeneration();
				op = *bestPath(_champions.begin(), _champions.end());
				glutPostRedisplay();
			}
			finishUpGeneration();
			op = *bestPath(_champions.begin(), _champions.end());
			glutPostRedisplay();

			std::cout << "end--------- " << op.getScore() << std::endl;
		}
	}
}

// Return 2 times the number of common links
int getDiff(const LocalOperator &champ, const LocalOperator &indiv) {
	int score = 0;
	const std::vector<std::vector<int>> &champLinks = champ.getLinks();
	const std::vector<std::vector<int>> &indivLinks = indiv.getLinks();
	for (int i = 0; i < (int) champLinks.size(); i++) {
		const std::vector<int> &cLinks = champLinks[i];
		const std::vector<int> &iLinks = indivLinks[i];
		for(int j : iLinks)
			if(std::find(cLinks.begin(), cLinks.end(), j) != cLinks.end())
				++ score;
	}
	return score;
}

void GeneticAlgorithm::initChampions() {
	_champions.clear();
	_champions.resize(_nbChampion);

	std::vector<LocalOperator>::iterator it = bestPath(_population.begin(), _population.end());
	_champions[0] = std::move(*it);
	if(it+1 != _population.end()) *it = std::move(_population.back());
	_population.pop_back();

	std::vector<int> scores(_population.size(), 0);
	for(int i = 1; i < _nbChampion; ++i) {
		int idx = 0;
		for(int j = 0; j < (int) _population.size(); ++j) {
			scores[j] += getDiff(_champions[i-1], _population[j]);
			if(scores[j] < scores[idx]) idx = j;
		}
		_champions[i] = std::move(_population[idx]);
		if(idx+1 != (int) _population.size()) _population[idx] = std::move(_population.back());
		_population.pop_back();
		scores[idx] = scores.back();
		scores.pop_back();
	}
}

void GeneticAlgorithm::upgradeGeneration() {
	std::cout << "upgrade" << std::endl;
	_population.resize(_nbIndividuals);
	for (int i = 0; i < _nbIndividuals; i++)
		_population[i] = _champions[i / _sonPerChamp];

	generateNewGeneration(_population);

	for (int i = 0; i < _nbChampion; i++) {
		std::vector<LocalOperator>::iterator it = bestPath(_population.begin() + (i * _sonPerChamp), _population.begin() + ((i + 1) * _sonPerChamp));
		if(_champions[i].getScore() >= it->getScore()) _champions[i] = std::move(*it);
	}
}

void GeneticAlgorithm::finishUpGeneration() {
	std::cout << "finish" << std::endl;
	for (auto &champ : _champions)
		champ.updateState(true);
	generateNewGeneration(_champions);
}

void GeneticAlgorithm::optimize() {
	std::vector<std::pair<int, int>> path_indices;
	int S = 0;
	for(const Layer &layer : _layers) S += layer._operators.size();
	path_indices.reserve(S);
	for(int i = 0; i < (int) _layers.size(); ++i)
		for(int j = 0; j < (int) _layers[i]._operators.size(); ++j)
			path_indices.emplace_back(i, j);

	if(parallelize(S, [&](int k) {
		const auto &[i, j] = path_indices[k];
		_layers[i]._operators[j].optimize();
		glutPostRedisplay();
	})) THROW_ERROR("An error occured in a thread of geometric optimizer!");
}