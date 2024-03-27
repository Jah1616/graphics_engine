#pragma once
#include <vector>
#include <limits>


enum ZBUF_MODE{
    ZBUF_NONE,
    ZBUF_LINE,
    ZBUF_TRIANGLE
};

constexpr double posInf = std::numeric_limits<double>::infinity();
constexpr double negInf = -std::numeric_limits<double>::infinity();

class ZBuffer: public std::vector<std::vector<double>>{
public:
    ZBuffer(const unsigned int width, const unsigned int height):image_width(width), image_height(height){
        std::vector<std::vector<double>>::resize(image_width, std::vector<double>(image_height, posInf));
    }
private:
    unsigned int image_width;
    unsigned int image_height;
};
