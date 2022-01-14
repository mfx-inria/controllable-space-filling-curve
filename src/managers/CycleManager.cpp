//
// Created by adrien_bedel on 30/09/19.
//

#include "managers/CycleManager.h"

#include "graphics/HamCore.h"
#include "tools/nanosvg.h"

#include <iostream>

CycleManager::CycleManager(int argc, char ** argv, const Printer &printer) : _printer(printer) {
    _layerNb = 1;
    if (argc > 1)
    {
        std::vector<std::string> args;
        argv++;
        while (*argv != NULL)
        {
            args.emplace_back(*argv);
            auto a = args.back();
            if (auto idx = a.find("fileName="); idx != std::string::npos)
            {
                a.erase(a.begin() + idx, a.begin() + idx + 9);
                _fileName = a;
            }
            else if (auto idx = a.find("layerNb="); idx != std::string::npos)
            {
                a.erase(a.begin() + idx, a.begin() + idx + 8);
                _layerNb = stoi(a);
            }
            else if (auto idx = a.find("seed="); idx != std::string::npos)
            {
                a.erase(a.begin() + idx, a.begin() + idx + 5);
                Globals::_seed = stoi(a);
            }
            argv++;
        }
        _ga.changeParameters(args);
    }
    if (_fileName.empty()) _fileName = "../results/brain/brainCut";
    std::cerr << "seed = " << Globals::_seed << std::endl;

    // Get imageWidth and height
    struct NSVGimage* image;
    //image = nsvgParseFromFile(("../resources/SVGmodels/" + _fileName + "_slices_SVG/" + _fileName + "_slice_0.svg").c_str(), "mm", 96);
    image = nsvgParseFromFile((_fileName + ".svg").c_str(), "mm", 96);

    if(image == nullptr) throw ("file parser problem");
    Globals::_SVGSize.x = image->width * .26458; // pixel to mm (pixel is 1/96 inch = .26458mm)
    Globals::_SVGSize.y = image->height * .26458; // pixel to mm (pixel is 1/96 inch = .26458mm)
    Image::_imgWidth = std::sqrt((Image::_pixelsWanted * Globals::_SVGSize.x) / Globals::_SVGSize.y);
    Image::_imgHeight = Image::_pixelsWanted / Image::_imgWidth;
    nsvgDelete(image);

    Image::resize(_layerNb);
}

void CycleManager::transposeThread() {

    //_ga.initLayers("../resources/SVGmodels/" + _fileName + "_slices_SVG/" + _fileName + "_slice_", _layerNb);
    _ga.initLayers(_fileName + ".svg");

    HamCore::stopRefrech();

    std::cout << "shuffle" << std::endl;
    // _ga.shuffle();
    std::cout << "opti" << std::endl;
    _ga.optimize();
    std::cout << "end opti" << std::endl;


    _printer.writeBorders(_ga._layers, _fileName + "_vary");
    _printer.writeToFile(_ga._layers, _fileName + "_vary", true);

    _printer.writeLayersToFile(_ga._layers);
    _printer.printToGcode(_ga._layers, "colorVary", true);
    _printer.printToGcode(_ga._layers, "colorNoVary", false);

    //_printer.writeResults(_ga, _fileName);
    std::cout << "done" << std::endl;
}
