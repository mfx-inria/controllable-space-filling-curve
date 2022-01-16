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

**Warning:** The code is currently being refactored. Instructions to use the code will be given in few days or week. We are working to make its use simpler.