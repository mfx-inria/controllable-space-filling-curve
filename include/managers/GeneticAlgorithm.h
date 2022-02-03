//
// Created by adrien_bedel on 18/11/19.
//

#ifndef HAMILTON_GENETICALGORITHM_H
#define HAMILTON_GENETICALGORITHM_H

#include "tools/Layer.h"

class GeneticAlgorithm {
public:
    std::vector<Layer>  _layers;
    static inline int   _multiplier = 15;

public:
    GeneticAlgorithm();

	void	process(const std::string &fileName, int layerNb);
    
	inline int getNbReadyLayers() const { return _nbReadyLayers; }

private:
    int                 _nbIndividuals;
    int                 _nbReadyLayers = 0;
    int                 _nbChampion = 12;
    int                 _sonPerChamp = 4;
    int                 _genNumber = 7;
    std::vector<LocalOperator>  _population;
    std::vector<LocalOperator>  _champions;

private:
    void	initLayers(const std::string &fileName, int nbLayer);
    void	initCycles();
    void	shuffle();
    void	initChampions();
    void	upgradeGeneration();
    void	finishUpGeneration();
    void	optimize();
};

#endif //HAMILTON_GENETICALGORITHM_H
