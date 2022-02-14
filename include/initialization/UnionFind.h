#ifndef HAMILTON_UNIONFIND_H
#define HAMILTON_UNIONFIND_H

#include <vector>

class UnionFind {
public:
    UnionFind() = default;
    UnionFind(int n): _idx(n, -1), _maxSize(1) {} 

    inline int find(int i) { return _idx[i]<0 ? i : _idx[i]=find(_idx[i]); }
    
	inline void merge(int i, int j) {
		i = find(i); j = find(j);
		if(i == j) return;
		if(_idx[i] > _idx[j]) std::swap(i, j);
		_idx[i] += _idx[j];
		_idx[j] = i;
		_maxSize = std::max(_maxSize, -_idx[i]);
	}

    inline bool fullyMerged(int nbPts) { return _idx.empty() || _maxSize >= nbPts; }
    inline bool same(int i, int j) { return find(i) == find(j); }

private:
    std::vector<int>    _idx;
    int                 _maxSize;
};

#endif //HAMILTON_UNIONFIND_H
