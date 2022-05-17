#!/bin/bash

echo "Installing dependencies..."
echo "libboost-dev, libgl-dev, freeglut3-dev, libglm-dev, libeigen3-dev"
sudo apt install libboost-dev libgl-dev freeglut3-dev libglm-dev libeigen3-dev

if [ ! -d build ]; then
	echo "Generating build directory"
	mkdir build
fi
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make