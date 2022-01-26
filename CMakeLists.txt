cmake_minimum_required(VERSION 3.12)

project(Hamilton)

find_package(Boost REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include/)
include_directories(include/LBFGS)
include_directories(${Boost_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-std=c++20 -pthread -O3 -Wall")

add_executable(${PROJECT_NAME}
	src/main.cpp
	src/graphics/Globals.cpp
	src/graphics/Window.cpp
	src/graphics/triangulate.cpp
	src/graphics/DirectionField.cpp
	src/LBFGS/clipper.cpp
	src/LBFGS/cvt.cpp
	src/localAlgo/LocalOperator.cpp
	src/localAlgo/ObjectiveFunctions.cpp
	src/managers/GeneticAlgorithm.cpp
	src/managers/GraphCreator.cpp
	src/managers/Printer.cpp
	src/tools/Layer.cpp
	src/tools/Matching.cpp
	src/tools/Union.cpp
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
	${Boost_LIBRARIES}
	${GLUT_LIBRARY}
	${OPENGL_LIBRARIES}
	Eigen3::Eigen
)