//
// Created by adrien_bedel on 30/09/19.
//

#ifndef HAMILTON_CYCLEMANAGER_H
#define HAMILTON_CYCLEMANAGER_H

#include "GeneticAlgorithm.h"
#include "managers/Printer.h"

class CycleManager
{
private:
    std::string                 _fileName;
    int                         _layerNb;

protected:
    Printer                 _printer;
    GeneticAlgorithm        _ga;

public:
    CycleManager(int, char **, const Printer &);
    void        transposeThread();

};

#endif //HAMILTON_CYCLEMANAGER_H
