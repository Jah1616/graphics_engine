cmake_minimum_required(VERSION 3.26)
project(2D_L_Systems)

set(CMAKE_CXX_STANDARD 17)

add_executable(2D_L_Systems easy_image.cc
        ini_configuration.cc
        Lines2D.cpp
        engine.cc
        l_parser/l_parser.cc)
