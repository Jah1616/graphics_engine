cmake_minimum_required(VERSION 3.26)
project(engine)

set(CMAKE_CXX_STANDARD 17)

add_executable(engine
        engine.cc
        easy_image.cc
        ini/ini_configuration.cc
        utils/l-parser.cc
        utils/vector3d.cc
        utils/3d_transformations.cpp
        utils/l-systems.cpp
        utils/platonic_bodies.cpp
        generate_image.cpp
        utils/utils.cpp
)
