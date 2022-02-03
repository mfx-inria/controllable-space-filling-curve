# controllable-space-filling-curve

Before compiling the code you need some libraries:
* Boost
* OpenGL
* FreeGLUT
* GLM
* Eigen3

If you are using a Debian distribution you can install these libraries using
```bash
sudo apt install libboost-dev libgl-dev freeglut3-dev libglm-dev libeigen3-dev
```

Once these libraries are installed, you can compile the code using
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

You can reproduce the right brain of Figure 12. using the following command
```bash
./build/Hamilton input/brain.txt
```
A window will appear. You will need to wait a bit before the model is displayed in the window. Some help to use the window will be printed in the terminal at the beginning 