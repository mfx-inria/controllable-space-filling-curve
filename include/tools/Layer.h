//
// Created by bedela on 26/05/2020.
//

#ifndef HAMILTON_LAYER_H
#define HAMILTON_LAYER_H

#include "localAlgo/LocalOperator.h"
#include <string>

class Layer
{
private:
    float _initialScore;

public:
    std::vector<LocalOperator> _operators;

public:
    Layer() = default;

    void initLayer(const std::string &, int);

    float getScore() const;
    float getInitialScore() const;
};

#endif //HAMILTON_LAYER_H
