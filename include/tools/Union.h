//
// Created by bedela on 14/05/2020.
//

#ifndef HAMILTON_UNION_H
#define HAMILTON_UNION_H

#include <vector>

class Union
{
private:
    std::vector<int>    _idx;
    std::vector<int>    _size;
    int                 _maxSize;

public:
    Union() = default;
    Union(int);
    void merge(int, int);
    bool fullyMerged(int);
    bool same(int, int);
    int find(int);
};

#endif //HAMILTON_UNION_H
