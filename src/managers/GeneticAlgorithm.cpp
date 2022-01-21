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

void GeneticAlgorithm::process(const std::string &fileName, int layerNb) try {
	// Algorithm
	std::cout << "===== Graph Construction ======" << std::endl;
	initLayers(fileName, layerNb);
	Window::stopRefrech();
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

////////////////////////
//
//      INIT
//
////////////////////////

GeneticAlgorithm::GeneticAlgorithm() {
	_nbIndividuals = _nbChampion * _sonPerChamp;
}

void GeneticAlgorithm::initLayers(const std::string &fileName, int nbLayer) {
	_layers.resize(nbLayer);
	_nbReadyLayers = 0;

	std::atomic<int> K = 0;
	std::atomic<bool> error = false;
	std::vector<bool> done(nbLayer, false);
	const auto fun = [&]() {
		try {
			int k;
			while((k = K++) < nbLayer) {
				_layers[k].initLayer(str_format(fileName, k), k);
				done[k] = true;
				while(_nbReadyLayers < nbLayer && done[_nbReadyLayers]) ++ _nbReadyLayers;
				glutPostRedisplay();
			}
		} catch(const CSFCerror &e) {
			error = true;
			std::cerr << e.what() << std::endl;
		}
	};

	const int T = std::min((int) Globals::_nbThread, nbLayer) - 1;
	std::vector<std::thread> threads;
	threads.reserve(T);
	for(int t = 0; t < T; ++t) threads.emplace_back(fun);
	fun();
	for(std::thread &t : threads) t.join();
	if(error) THROW_ERROR("An error occured in a thread of graph construction!");
	_nbReadyLayers = nbLayer;
	glutPostRedisplay();
}

void GeneticAlgorithm::initLayers(const std::string &fileName)
{
	_layers.resize(1);
	_layers[0].initLayer(fileName, 0);
	_nbReadyLayers = 1;
	glutPostRedisplay();
}

inline static std::vector<LocalOperator>::iterator bestPath(std::vector<LocalOperator>::iterator begin, std::vector<LocalOperator>::iterator end) {
	return std::min_element(begin, end, [](const LocalOperator &a, const LocalOperator &b) {
		return a.getScore() < b.getScore();
	});
}

void GeneticAlgorithm::shuffle() {
	for (Layer &layer : _layers) {
		for(LocalOperator &op : layer._operators) {
			if(!op.isSucces() || op.getPoints().size() < 7) continue;

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

void GeneticAlgorithm::optimize()
{
	std::atomic<int> K = 0;
	std::vector<std::pair<int, int>> path_indices;
	int S = 0;
	for(const Layer &layer : _layers) S += layer._operators.size();
	path_indices.reserve(S);
	for(int i = 0; i < (int) _layers.size(); ++i)
		for(int j = 0; j < (int) _layers[i]._operators.size(); ++j)
			path_indices.emplace_back(i, j);


	const auto fun = [&](bool mainThread) {
		int k;
		while((k = K++) < S) {
			 const auto &[i, j] = path_indices[k];
			_layers[i]._operators[j].optimize();
			if(mainThread) glutPostRedisplay();
		}
	};

	const int T = std::min((int) Globals::_nbThread, S) - 1;
	std::vector<std::thread> threads;
	threads.reserve(T);
	for(int t = 0; t < T; ++t) threads.emplace_back(fun, false);
	fun(true);
	for(std::thread &t : threads) t.join();
	glutPostRedisplay();
}

void GeneticAlgorithm::generateNewGeneration(std::vector<LocalOperator> &population)
{
	const int N = population.size();
	std::atomic<int> K = 0;
	const auto fun = [&]() {
		int k;
		while((k = K++) < N) {
			population[k].startShuffling(_multiplier, k);		
			std::cout << "resulted score = " << population[k].getScore() << std::endl;
		}
	};

	const int T = std::min((int) Globals::_nbThread, N) - 1;
	std::vector<std::thread> threads;
	threads.reserve(T);
	for(int t = 0; t < T; ++t) threads.emplace_back(fun);
	fun();
	for(std::thread &t : threads) t.join();
}

void GeneticAlgorithm::upgradeGeneration()
{
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

void GeneticAlgorithm::finishUpGeneration()
{
	std::cout << "finish" << std::endl;
	for (auto &champ : _champions)
		champ.updateState(true);
	generateNewGeneration(_champions);
}

void GeneticAlgorithm::initChampions()
{
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

// Return 2 times the number of common links
int GeneticAlgorithm::getDiff(const LocalOperator &champ, const LocalOperator &indiv) const
{
	int score = 0;
	const std::vector<std::vector<int>> &champLinks = champ.getLinks();
	const std::vector<std::vector<int>> &indivLinks = indiv.getLinks();
	for (int i = 0; i < (int) champLinks.size(); i++)
	{
		const std::vector<int> &cLinks = champLinks[i];
		const std::vector<int> &iLinks = indivLinks[i];
		for(int j : iLinks)
			if(std::find(cLinks.begin(), cLinks.end(), j) != cLinks.end())
				++ score;
	}
	return score;
}

int GeneticAlgorithm::getGen() const
{
	return _genNumber;
}

int GeneticAlgorithm::getNbReadyLayers() const
{
	return _nbReadyLayers;
}