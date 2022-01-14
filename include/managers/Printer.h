//
// Created by adrien_bedel on 06/01/20.
//

#ifndef HAMILTON_PRINTER_H
#define HAMILTON_PRINTER_H

#include "tools/Layer.h"
#include <fstream>

enum Machine { COLOR, PRUSA, CR10S_PRO};

#define STARTPOS 110.5f

class Printer
{
private:
    float                       _wantedSpeed = 600.f;
    float                       _multiplier;
    float                       _size;
    float                       _printStartX;
    float                       _printStartY;
    std::vector<std::string>    _printerPaths;
    unsigned int                _machineIdx;
    float _extrusion;


private:
    void    initSpeedMultiplier(const LocalOperator &, bool);
    void    travel(std::fstream &, int, float, float);
    void    printLineSpeed(std::fstream &, int, int, int, double, int);
    void    printLine(std::fstream &, int, int, double, int, float, float);
    void    resetAtPos(std::fstream &, float, float, int);
    void    purge(std::fstream &, float, float, const glm::vec3 &);
    void    printLineCustom(std::fstream &, float, float, double, const glm::vec3 &);
    void    printLineCustom(std::fstream &, const glm::vec2 &, double, const glm::vec3 &);
    void    printLineCustom(std::fstream &, const glm::vec2 & , double);

    float   printBoundingSquare(std::fstream &, float , float , float , float , float, const glm::vec3 &);
    void    printLayers(std::fstream &, const std::vector<Layer> &, int, bool);
    float   getExtrusionV(float , const std::vector<glm::vec3> &);
    void    retract(std::fstream &, int);
    std::pair<double, double>    calculateExtrusion(const std::vector<glm::vec3> &, float, int , int, bool);

public:
    Printer(const Machine &);
    void    printToGcode(const std::vector<Layer> &, const std::string &, bool);
    void    writeToFile(const std::vector<Layer> &, const std::string &, bool);
    void    writeBorders(const std::vector<Layer> &, const std::string &);
    void    writeLayersToFile(const std::vector<Layer> &);
};

#endif //HAMILTON_PRINTER_H
