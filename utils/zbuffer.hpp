#pragma once
#include <vector>
#include <limits>
#include <iostream>


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
        resize(image_width, std::vector<double>(image_height, posInf));
    }
private:
    unsigned int image_width;
    unsigned int image_height;
};


static void draw_zbuf_line(img::EasyImage& image, ZBuffer &zbuf, int x0, int y0, double z0,
                                    int x1, int y1, double z1, const img::Color &color){
    if (x0 >= image.get_width() || y0 >= image.get_height() || x1 >= image.get_width() || y1 > image.get_height()) {
        std::cerr << "Drawing zbuf line from (" << x0 << "," << y0 << ") to (" << x1 << "," << y1 << ") in image of width "
           << image.get_width() << " and height " << image.get_height();
        exit(-1);
    }
    assert(x0 >= 0); assert(y0 >= 0);
    assert(x1 >= 0); assert(y1 >= 0);
    const double delta = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
    if (x0 == x1){
//        special case for x0 == x1
        if (y0 > y1){
            std::swap(y0, y1);
            std::swap(z0, z1);
        }
        for (int i = y0; i <= y1; i++){
            const double p = abs((i - y1))/delta;
            const double z_inv = p/z0 + (1-p)/z1;
            if (z_inv < zbuf[x0][i]){
                image(x0, i) = color;
                zbuf[x0][i] = z_inv;
            }
        }
    }
    else if (y0 == y1){
//        special case for y0 == y1
        if (x0 > x1){
            std::swap(x0, x1);
            std::swap(z0, z1);
        }
        for (int i = x0; i <= x1; i++){
            const double p = abs((i - x1))/delta;
            const double z_inv = p/z0 + (1-p)/z1;
            if (z_inv < zbuf[i][y0]){
                image(i, y0) = color;
                zbuf[i][y0] = z_inv;
            }
        }
    }
    else {
        if (x0 > x1){
            //flip points if x1>x0: we want x0 to have the lowest value
            std::swap(x0, x1);
            std::swap(y0, y1);
            std::swap(z0, z1);
        }
        double m = (double)(y1- y0) / (x1-x0);
        if (-1.0 <= m && m <= 1.0){
            for (int i = 0; i <= (x1 - x0); i++){
                const int x = x0+i;
                const double y = y0 + m * i;
                const int y_round = lround(y);

                const double p = abs(sqrt((x1-x)*(x1-x) + (y1-y)*(y1-y))) / delta;
                const double z_inv = p/z0 + (1-p)/z1;
                if (z_inv < zbuf[x][y_round]){
                    image(x, y_round) = color;
                    zbuf[x][y_round] = z_inv;
                }
            }
        }
        else if (m > 1.0){
            for (int i = 0; i <= (y1 - y0); i++){
                const double x = x0 + (i / m);
                const int x_round = lround(x);
                const int y = y0 + i;

                const double p = abs(sqrt((x1-x)*(x1-x) + (y1-y)*(y1-y))) / delta;
                const double z_inv = p/z0 + (1-p)/z1;
                if (z_inv < zbuf[x_round][y]){
                    image(x_round, y) = color;
                    zbuf[x_round][y] = z_inv;
                }
            }
        }
        else if (m < -1.0){
            for (int i = 0; i <= (y0 - y1); i++){
                const double x = x0 - (i / m);
                const int x_round = lround(x);
                const int y = y0 - i;

                const double p = abs(sqrt((x1-x)*(x1-x) + (y1-y)*(y1-y))) / delta;
                const double z_inv = p/z0 + (1-p)/z1;
                if (z_inv < zbuf[x_round][y]){
                    image(x_round, y) = color;
                    zbuf[x_round][y] = z_inv;
                }
            }
        }
    }
}