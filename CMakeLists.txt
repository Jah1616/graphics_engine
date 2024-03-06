cmake_minimum_required(VERSION 3.26)
project(engine)

set(CMAKE_CXX_STANDARD 17)

add_executable(engine easy_image.cc
        ini_configuration.cc
        engine.cc
        2D_L-System/l_parser/l_parser.cc
        3D/vector3D.cc
        3D/Figure3D.cpp
        utils.hpp)
