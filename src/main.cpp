//
// Created by adrien_bedel on 23/09/19.
//

#include "graphics/Window.h"
#include "tools/nanosvg.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <future>

void getImageSize(const std::string &fileName, float &width, float &heihgt) {
    struct NSVGimage* image;
    image = nsvgParseFromFile(fileName.c_str(), "mm", 96);
    if(image == nullptr) throw std::runtime_error("can't parse input svg file");
	width = image->width * .26458; // pixel to mm (pixel is 1/96 inch = .26458mm)
	heihgt = image->height * .26458;
    nsvgDelete(image);
}

int main(int argc, char** argv) {
	// Parse arguments
	if(argc < 2) {
		std::cerr << "Error: this programm needs at least one argument: the path to a txt file describing the input and the parameters used. Other arguments will be sent to glut." << std::endl;
		exit(1);
	}
	std::string path(argv[1]);
	std::ifstream ifs(path);
	std::string fileName = "../results/brain/brainCut.svg";
	int layerNb = 1;
	while(ifs) {
		std::string line, var, val;
		std::getline(ifs, line);
		std::istringstream iss(line);
		if(iss >> var && iss >> val) {
			if(var == "path") fileName = val;
			else if(var == "d") ;
			else if(var == "layerNb") layerNb = std::stoi(val);
			else if(var == "seed") Globals::_seed = std::stoul(val);
		}
	}

	// Init variables
	getImageSize(str_format(fileName, 0), Globals::_SVGSize.x, Globals::_SVGSize.y);
    Globals::initVariables(layerNb);

	// Start windows and process
	GeneticAlgorithm ga;
    Window hm(argc, argv, &ga);
	auto t = std::async(&GeneticAlgorithm::process, &ga, fileName, layerNb);
	hm.start();
    return 0;
}
