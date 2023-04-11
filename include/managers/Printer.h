//
// Created by adrien_bedel on 06/01/20.
//

#ifndef HAMILTON_PRINTER_H
#define HAMILTON_PRINTER_H

#include "tools/Layer.h"
#include <fstream>

enum Machine { COLOR, PRUSA, CR10S_PRO};

#define STARTPOS 110.5

class Printer
{
private:
    double                       _wantedSpeed = 600.;
    double                       _multiplier;
    double                       _size;
    double                       _printStartX;
    double                       _printStartY;
    std::vector<std::string>    _printerPaths;
    unsigned int                _machineIdx;
    double _extrusion;


private:
    void    initSpeedMultiplier(const LocalOperator &, bool);
    void    travel(std::fstream &, int, double, double);
    void    printLineSpeed(std::fstream &, int, int, int, double, int);
    void    printLine(std::fstream &, int, int, double, int, double, double);
    void    resetAtPos(std::fstream &, double, double, int);
    void    purge(std::fstream &, double, double, const glm::dvec3 &);
    void    printLineCustom(std::fstream &, double, double, double, const glm::dvec3 &);
    void    printLineCustom(std::fstream &, const glm::dvec2 &, double, const glm::dvec3 &);
    void    printLineCustom(std::fstream &, const glm::dvec2 & , double);

    double   printBoundingSquare(std::fstream &, double , double , double , double , double, const glm::dvec3 &);
    void    printLayers(std::fstream &, const std::vector<Layer> &, int, bool);
    double   getExtrusionV(double , const std::vector<glm::dvec3> &);
    void    retract(std::fstream &, int);
    std::pair<double, double>    calculateExtrusion(const std::vector<glm::dvec3> &, double, int , int, bool);

public:
    Printer(const Machine &);
    void    printToGcode(const std::vector<Layer> &, const std::string &, bool);
    void    writeToFile(const std::vector<Layer> &, const std::string &, bool);
    void    writeBorders(const std::vector<Layer> &, const std::string &);
    void    writeLayersToFile(const std::vector<Layer> &);
};

#endif //HAMILTON_PRINTER_H
