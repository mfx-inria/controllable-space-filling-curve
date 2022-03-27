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
    int                 _genNumber = 6;

    GeneticAlgorithm();

	void process(const std::string &fileName, int layerNb);
    
	inline int getNbReadyLayers() const { return _nbReadyLayers; }

private:
    int                 _nbReadyLayers = 0;
    int                 _nbChampion = 10;
    int                 _sonPerChamp = 4;
    int                 _nbIndividuals;
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
