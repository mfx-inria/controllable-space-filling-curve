//
// Created by adrien_bedel on 25/09/19.
//

#ifndef HAMILTON_HAMCORE_H
#define HAMILTON_HAMCORE_H

#include "managers/CycleManager.h"

class HamCore : public CycleManager
{
private:
    inline static HamCore*  currentInstance;

    bool                    _isRefresh;
    bool                    _showAllLinks;
    bool                    _showAnyLinks;
    bool                    _showVecField;
    bool                    _showStrokeColor;
    unsigned int            _layerIndex = 0;

    glm::vec2               _WinCenter = glm::vec2(-1.f, 0.f);
    float                   _zoom = 1.;
    glm::vec2               _lastClikedPos;
    bool                    _leftDown = false;

    std::vector<int>        _stack;

public:
    HamCore(int , char **, const std::string &, Machine);

    static void addToStack(int);
    static void stopRefrech();

private:
    void                    display();
    void                    displayCycle();
    void                    keyPressed(unsigned char , int , int);
    void                    mouseClicked(int, int, int, int);
    void                    mouseMoved(int, int);
    static void             drawCallback();
    static void             keyCallback(unsigned char, int , int );
    static void             mouseCallback(int, int, int, int);
    static void             motionCallback(int, int);

    glm::vec2               getCoord(int, int);
    void                    replaceCenter();

};

#endif //HAMILTON_HAMCORE_H
