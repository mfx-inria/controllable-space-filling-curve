//
// Created by bedela on 01/07/2020.
//

#ifndef HAMILTON_MATCHING_H
#define HAMILTON_MATCHING_H

#include "graphics/Shape.h"
#include "tools/Union.h"

#include <memory>

class Tree {

public:
    int _nodeIdx;
    bool _needEdge;
    std::shared_ptr<Tree> _parent;
    std::vector<std::shared_ptr<Tree>> _childrens;
public:
    Tree() = delete;
    Tree(int, bool, std::shared_ptr<Tree>);
    bool    operator==(int) const;

};

class Matching {
public:
    int     _score = 0;
    int     _nbConnectedPoints;

    std::vector<std::vector<int>>   _cLinks;
    std::vector<std::vector<int>>   _originalLinks;
    std::vector<glm::vec2>          _points;
    Union                           _union;

private:
    void    removeUnused(const Shape &);
    void    checkInit();
    void    switchLink();
    void    augment();
    void    augmentPath(int);
    void    createLink(int, int);
    void    createOriLink(int, int);
    void    removeLink(int, int);
    void    removeOriLink(int, int);
    void    initUnion();
    bool    isLinked(const std::vector<int> &, int );
    void    match();
    void    fuseIslands();
    int     distInTree(std::shared_ptr<Tree>, int );
    int     getIdx(int, int);
    bool    splitFourConnected();
    std::vector<int>    getIdxs(int, int);
    std::vector<int>    getPath(int, int);

public:
    Matching(const Shape &, Graph &);
};

#endif //HAMILTON_MATCHING_H
