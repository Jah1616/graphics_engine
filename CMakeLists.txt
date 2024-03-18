cmake_minimum_required(VERSION 3.26)
project(engine)

set(CMAKE_CXX_STANDARD 17)

add_executable(engine easy_image.cc
        ini/ini_configuration.cc
        engine.cc
        utils/l_parser/l_parser.cc
        utils/Vector3D.cc)
