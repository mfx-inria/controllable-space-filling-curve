//
// Created by adrien_bedel on 06/01/20.
//

#include "managers/Printer.h"
#include <iomanip>
#include <iostream>

Printer::Printer(const Machine &machine)
{
    _machineIdx = machine;
    _printStartX = std::max(STARTPOS, 10.f);
    _printStartY = std::max(STARTPOS, 5.f);
    _printerPaths = {"../resources/Printers/ColorPrinter/", "../resources/Printers/prusa/", "../resources/Printers/CR10S_Pro/"};
}

void Printer::printToGcode(const std::vector<Layer> &layers, const std::string &name, bool doVary)
{
    _size = std::max(Globals::_SVGSize.x, Globals::_SVGSize.y) + 10.f;
    std::fstream myfi;
    myfi.open("core.gcode", std::ios::out);
    myfi << std::fixed;
    myfi << std::setprecision(6);

    printLayers(myfi, layers, 1, doVary);

    myfi.close();

    std::ifstream header(_printerPaths[_machineIdx] + "header.gcode", std::ios::in);
    std::ifstream footer(_printerPaths[_machineIdx] + "footer.gcode", std::ios::in);
    std::ifstream core("core.gcode", std::ios::in);
    std::fstream output(name + ".gcode", std::ios::out);
    output << header.rdbuf() << core.rdbuf() << footer.rdbuf();

    core.close();
    header.close();
    footer.close();
    output.close();
}

float pointDist(const glm::vec3 &a, const glm::vec3 &b) {
	return glm::distance(glm::vec2(a), glm::vec2(b));
}

void Printer::printLayers(std::fstream &myfi, const std::vector<Layer> &layers, int layerMult, bool doVary)
{
    initSpeedMultiplier(layers.front()._operators.front(), doVary);
    if (layerMult == 1)
    {
        auto [p, c] = layers[0]._operators[0].getFinal();
        purge(myfi, _printStartX - 5.f, _printStartY - 5.f, c[0]);
    }

    double extrusion = 0.0;
    double futurExtrusion = 0.0;

    int layerNb = 1;
    // foreach layer
    for (const auto &layer : layers)
    {
        // foreach shape in layer
        for (int i = 0; i < (int) layer._operators.size(); i++)
        {
            if(!layer._operators[i].isSucces()) continue;
            auto [points, zones] = layer._operators[i].getFinal();
            int nb = points.size();
            std::cout << "nb = " << nb << std::endl;
            float extrusionValue = getExtrusionV(layer._operators[i].getArea(), points);

            resetAtPos(myfi, points[0].x + _printStartX, points[0].y + _printStartY, 2000);
            myfi << "G1 Z" << Globals::_outRPrismeHeight * (float)layerNb * (float)layerMult << std::endl;
            if (layerNb * layerMult == 1)
                myfi << "G1 600" << std::endl;
            else
                myfi << "G1 F1200" << std::endl;
            extrusion = 0.0;
            futurExtrusion = 0.0;

            int k = 0;

            // follow path of the shapes
            for (int j = 0; j < nb; j++)
            {
                while (futurExtrusion - extrusion < Globals::_lengthToClear)
                {
                    auto [F, E] = calculateExtrusion(points, extrusionValue, k % nb, nb, doVary);
                    futurExtrusion += E;
                    k++;
                }

                auto [F, E] = calculateExtrusion(points, extrusionValue, j, nb, doVary);

                if (doVary)
                    myfi << "G1 F" << (int)(F * _multiplier) << std::endl;
                extrusion += E;

                if (_machineIdx == Machine::COLOR)
                    printLineCustom(myfi, points[(j + 1) % nb], extrusion, zones[(k + 1) % nb]);
                else
                    printLineCustom(myfi, points[(j + 1) % nb], extrusion);
            }
            retract(myfi, 2000);
        }
        if (layerNb == 1)
            _multiplier *= 2.f;
        std::cout << "extrusion = " << extrusion << std::endl;
        layerNb++;
    }
    _extrusion = extrusion;
    myfi << "G1 Z" << Globals::_outRPrismeHeight * ((float)layerNb * (float)layerMult + 1.f) << std::endl;
}

std::pair<double, double> Printer::calculateExtrusion(const std::vector<glm::vec3> &points, float extrusionValue, int j, int nb, bool doVary)
{
    double F = 0.f, E = 0.f;
    if (doVary)
    {
        double X = 0.1f;
        double fnom = 200.0;
        E = (points[(j + 1) % nb].z * Globals::_outRPrismeHeight) / Globals::_frameInArea;
        F = 60.0 * pointDist(points[j], points[(j + 1) % nb]) / (E / X);
        F = fnom * pow(F / fnom, 3.0);
        F *= 5.0;
        F = std::min(500.0, std::max(10.0, F));
    }
    else
    {
        E = pointDist(points[j], points[(j + 1) % nb]) * extrusionValue;
    }
    return {F, E};
}

float Printer::getExtrusionV(float area, const std::vector<glm::vec3> &points)
{
    float volume = area * Globals::_outRPrismeHeight;
    float totalDist = 0;
    int nb = points.size();
    for (int i = 0; i < nb; i++)
        totalDist += pointDist(points[i], points[(i + 1) % nb]);
    std::cout << "total Dist = " << totalDist << std::endl;
    float lengthFrameIn = volume / Globals::_frameInArea;
    return (lengthFrameIn / totalDist);
}

void Printer::initSpeedMultiplier(const LocalOperator &op, bool doVary)
{
    if (!doVary)
    {
        _multiplier = 1.f;
        return ;
    }
    const auto &[points, zone] = op.getFinal();
    int nb = points.size();
    float extrusionValue = getExtrusionV(op.getArea(), points);
    double fullSpeed = 0.f;

    for (int j = 0; j < nb; j++)
    {
        auto [F, E] = calculateExtrusion(points, extrusionValue, j, nb, doVary);
        fullSpeed += F;
    }
    fullSpeed /= (double)nb;
    _multiplier = _wantedSpeed / fullSpeed;
}

float Printer::printBoundingSquare(std::fstream &myfi, float cubeStartX, float cubeStartY, float cubeSize, float finalExtrud, float extrud, const glm::vec3 &color)
{
    printLineCustom(myfi, cubeStartX, cubeStartY + cubeSize, finalExtrud, color);
    finalExtrud += extrud;
    printLineCustom(myfi, cubeStartX + cubeSize, cubeStartY + cubeSize, finalExtrud, color);
    finalExtrud += extrud;
    printLineCustom(myfi, cubeStartX + cubeSize, cubeStartY, finalExtrud, color);
    finalExtrud += extrud;
    printLineCustom(myfi, cubeStartX, cubeStartY, finalExtrud, color);
    finalExtrud += extrud;
    return finalExtrud;
}

void Printer::purge(std::fstream &myfi, float startX, float startY, const glm::vec3 &color)
{
    myfi << "G1 Z" << Globals::_outRPrismeHeight << std::endl;
    float cubeStartX = startX;
    float cubeStartY = startY;
    float cubeSize = _size;
    float extrud = cubeSize * Globals::_extrusionHeight * 1.3f;
    float finalExtrud = extrud;
    resetAtPos(myfi, cubeStartX, cubeStartY, 700);
    finalExtrud = printBoundingSquare(myfi, cubeStartX, cubeStartY, cubeSize, finalExtrud, extrud, glm::vec3(1, 0, 0));
    cubeStartX++;
    cubeStartY++;
    cubeSize -= 2;
    travel(myfi, 700, cubeStartX, cubeStartY);
    finalExtrud = printBoundingSquare(myfi, cubeStartX, cubeStartY, cubeSize, finalExtrud, extrud, glm::vec3(0, 1, 0));
    cubeStartX++;
    cubeStartY++;
    cubeSize -= 2;
    travel(myfi, 700, cubeStartX, cubeStartY);
    printBoundingSquare(myfi, cubeStartX, cubeStartY, cubeSize, finalExtrud, extrud, color);
    retract(myfi, 300);
}

void Printer::retract(std::fstream &myfi, int speed)
{
    myfi << "G92 E0.0" << std::endl;
    if (_machineIdx == Machine::CR10S_PRO)
        return;
    myfi << ";retract" << std::endl;
    if (_machineIdx == Machine::COLOR)
        myfi << "G0 F" << speed << " E-9.000000" << std::endl;
    else
        myfi << "G0 F" << 300 << " E-0.500000" << std::endl;
}

void Printer::resetAtPos(std::fstream &myfi, float x, float y, int speed)
{
    travel(myfi, 4800, x, y);
    myfi << ";prime" << std::endl;
    myfi << "G0 F" << speed << " E0.000000" << std::endl;
    myfi << "G92 E0.0" << std::endl;
}

void Printer::travel(std::fstream &myfi, int speed, float x, float y)
{
    myfi << ";travel" << std::endl;
    myfi << "G0"
         << " F" << speed
         << " X" << x
         << " Y" << y << std::endl;
}

void Printer::printLineCustom(std::fstream &myfi, const glm::vec2 &pos, double extrusion, const glm::vec3 &color) {
    myfi << "G1"
         << " X" << pos.x + _printStartX
         << " Y" << pos.y + _printStartY
         << " E" << extrusion
         << " A" << color.x
         << " B" << color.y
         << " C" << color.z << std::endl;
}

void Printer::printLineCustom(std::fstream &myfi, float x, float y, double extrusion, const glm::vec3 &color) {
    myfi << "G1"
         << " X" << x
         << " Y" << y
         << " E" << extrusion;
    if (_machineIdx == Machine::COLOR)
    {
        myfi << " A" << color.x
             << " B" << color.y
             << " C" << color.z;
    }
    myfi << std::endl;
}

void Printer::printLineCustom(std::fstream &myfi, const glm::vec2 &pos, double extrusion)
{
    myfi << "G1"
         << " X" << pos.x + _printStartX
         << " Y" << pos.y + _printStartY
         << " E" << extrusion << std::endl;
}