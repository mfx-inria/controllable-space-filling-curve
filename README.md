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
or using the `compile.sh`script.

You can reproduce the right brain of Figure 15 and our result in Figure 25 using the following commands
```bash
./build/Hamilton input/brain.txt
./build/Hamilton input/people.txt
```
or using the `figure15.sh` and `figure25.sh` scripts.
A window will appear. You will need to wait a bit before the model is displayed in the window. Some help to use the window will be printed in the terminal at the beginning.
Moving your mouth in the Window will force the update of the window if you still see nothing after several seconds.