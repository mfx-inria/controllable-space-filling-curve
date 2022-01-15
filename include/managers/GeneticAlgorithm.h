//
// Created by adrien_bedel on 18/11/19.
//

#ifndef HAMILTON_GENETICALGORITHM_H
#define HAMILTON_GENETICALGORITHM_H

#include "tools/Layer.h"

class GeneticAlgorithm
{
public:
    std::vector<Layer>  _layers;
    static inline int   _multiplier = 15;

private:
    int                 _nbIndividuals;
    int                 _nbReadyLayers = 0;
    int                 _nbChampion = 12;
    int                 _sonPerChamp = 4;
    int                 _genNumber = 7;
    std::vector<LocalOperator>  _population;
    std::vector<LocalOperator>  _champions;

public:
    GeneticAlgorithm();
    void                initLayers(const std::string &, int );
    void                initLayers(const std::string &);
    void                shuffle();
    void                generateNewGeneration(std::vector<LocalOperator> &);
    void                upgradeGeneration();
    void                finishUpGeneration();
    void                initChampions();
    int                 getDiff(const LocalOperator &, const LocalOperator &) const;
    void                optimize();
    int                 getGen() const;
    int                 getNbReadyLayers() const;
	void				process(const std::string &fileName, int layerNb);
};

#endif //HAMILTON_GENETICALGORITHM_H
