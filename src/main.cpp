//
// Created by adrien_bedel on 23/09/19.
//

#include "graphics/HamCore.h"
#include <memory>
#include <iostream>

int main(int argc, char** argv)
{
    Globals::initVariables();

    try
    {
    std::unique_ptr<HamCore> core = std::make_unique<HamCore>(argc, argv, "Ham", Machine::CR10S_PRO);
    }
    catch (char *msg)
    {
        std::cout << msg << std::endl;
    }
    catch (const std::invalid_argument &error)
    {
        std::cout << "invalid arguments" << std::endl;
    }
    return 0;
}
