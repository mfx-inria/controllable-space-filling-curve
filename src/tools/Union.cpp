//
// Created by bedela on 14/05/2020.
//

#include "tools/Union.h"

Union::Union(int N) {
    _idx.resize(N);
    for(int i = 0; i < N; ++i) _idx[i] = i;
    _size.assign(N, 1);
    _maxSize = 1;
}

int Union::find(int i) {
    if(_idx[i] == i) return i;
    return (_idx[i] = find(_idx[i]));
}

void Union::merge(int i, int j){
    i = find(i);
    j = find(j);
    if(i == j) return;
    if(_size[i] < _size[j]) std::swap(i, j);
    _idx[j] = i;
    _size[i] += _size[j];
    _maxSize = std::max(_maxSize, _size[i]);
}

bool Union::fullyMerged(int nbPts) {
    return _idx.empty() || _maxSize >= nbPts;
}

bool Union::same(int i, int j) {
    return find(i) == find(j);
}
