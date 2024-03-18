#pragma once
#include "../ini_configuration.h"
#include "../easy_image.h"
#include "../utils/utils.h"
#include "../L-Systems/2D-L-System.h"


img::EasyImage draw2DLines (const Lines2D &lines, const unsigned int size, const img::Color &bgColor){
    // set min and max coords
    Line2D firstline = lines.front();
    double xmin{fmin(firstline.p1.x, firstline.p2.x)};
    double xmax{fmax(firstline.p1.x, firstline.p2.x)};
    double ymin{fmin(firstline.p1.y, firstline.p2.y)};
    double ymax{fmax(firstline.p1.y, firstline.p2.y)};
    for (auto line : lines){
        xmin = (line.p1.x < xmin) ? line.p1.x : xmin;
        xmin = (line.p2.x < xmin) ? line.p2.x : xmin;
        xmax = (line.p1.x > xmax) ? line.p1.x : xmax;
        xmax = (line.p2.x > xmax) ? line.p2.x : xmax;
        ymin = (line.p1.y < ymin) ? line.p1.y : ymin;
        ymin = (line.p2.y < ymin) ? line.p2.y : ymin;
        ymax = (line.p1.y > ymax) ? line.p1.y : ymax;
        ymax = (line.p2.y > ymax) ? line.p2.y : ymax;
    }

    double xrange = xmax - xmin;
    double yrange = ymax - ymin;
    double imagex = size * xrange / fmax(xrange, yrange);
    double imagey = size * yrange / fmax(xrange, yrange);

    img::EasyImage image(lround(imagex), lround(imagey));
    image.clear(bgColor);

    double scale = 0.95 * imagex / xrange;
    double dx = (imagex - scale * (xmin + xmax)) / 2;
    double dy = (imagey - scale * (ymin + ymax)) / 2;

    for (auto line : lines){
        line.p1.x *= scale;
        line.p1.y *= scale;
        line.p2.x *= scale;
        line.p2.y *= scale;

        line.p1.x += dx;
        line.p1.y += dy;
        line.p2.x += dx;
        line.p2.y += dy;
        image.draw_line(lround(line.p1.x), lround(line.p1.y), lround(line.p2.x), lround(line.p2.y),
                        img::Color(lround(line.color.red*255), lround(line.color.green*255), lround(line.color.blue*255)));
    }
    return image;
}

img::EasyImage generate2DLSystemImage(const ini::Configuration &conf){
    unsigned int size = conf["General"]["size"].as_int_or_die();
    std::vector<double> bgColor = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    std::string inputFile = conf["2DLSystem"]["inputfile"].as_string_or_die();
    std::vector<double> color = conf["2DLSystem"]["color"].as_double_tuple_or_die();

    return draw2DLines(L2D_to_Lines2D(inputFile, Color(color[0], color[1], color[2])),
                       size, img::Color(lround(bgColor[0] * 255), lround(bgColor[1] * 255), lround(bgColor[2] * 255)));
}
