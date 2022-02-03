//
// Created by bedela on 26/05/2020.
//

#ifndef HAMILTON_LAYER_H
#define HAMILTON_LAYER_H

#include "localAlgo/LocalOperator.h"

class Layer {
public:
    std::vector<LocalOperator> _operators;

public:
    Layer() = default;

    void initLayer(const std::string &fileName, int layerIndex);
	void initCycle(int layerIndex);
};

#endif //HAMILTON_LAYER_H
