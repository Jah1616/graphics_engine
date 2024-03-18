#pragma once
#include "../ini_configuration.h"
#include "../easy_image.h"
#include "../utils/utils.h"
#include "../L-Systems/2D-L-System.h"
#include <cmath>


img::EasyImage draw2DLines (const Lines2D &lines, const unsigned int size, const img::Color &bgColor){
    // set min and max coords
    auto firstline = lines.front();
    auto xmin{fmin(firstline.p1.x, firstline.p2.x)};
    auto xmax{fmax(firstline.p1.x, firstline.p2.x)};
    auto ymin{fmin(firstline.p1.y, firstline.p2.y)};
    auto ymax{fmax(firstline.p1.y, firstline.p2.y)};
    for (const auto &line : lines){
        if (line.p1.x < xmin) xmin = line.p1.x;
        if (line.p2.x < xmin) xmin = line.p2.x;
        if (line.p1.x > xmax) xmax = line.p1.x;
        if (line.p2.x > xmax) xmax = line.p2.x;
        if (line.p1.y < ymin) ymin = line.p1.y;
        if (line.p2.y < ymin) ymin = line.p2.y;
        if (line.p1.y > ymax) ymax = line.p1.y;
        if (line.p2.y > ymax) ymax = line.p2.y;
    }

    const auto xrange = xmax - xmin;
    const auto yrange = ymax - ymin;
    const auto imagex = size * xrange / fmax(xrange, yrange);
    const auto imagey = size * yrange / fmax(xrange, yrange);

    img::EasyImage image(lround(imagex), lround(imagey));
    image.clear(bgColor);

    const auto scale = 0.95 * imagex / xrange;
    const auto dx = (imagex - scale * (xmin + xmax)) / 2;
    const auto dy = (imagey - scale * (ymin + ymax)) / 2;

    for (auto line : lines){
        line.p1.x *= scale; line.p1.y *= scale;
        line.p2.x *= scale; line.p2.y *= scale;

        line.p1.x += dx; line.p1.y += dy;
        line.p2.x += dx; line.p2.y += dy;
        image.draw_line(lround(line.p1.x), lround(line.p1.y), lround(line.p2.x), lround(line.p2.y),
                        img::Color(lround(line.color.red*255), lround(line.color.green*255), lround(line.color.blue*255)));
    }
    return image;
}

img::EasyImage generate2DLSystemImage(const ini::Configuration &conf){
    const auto size = conf["General"]["size"].as_int_or_die();
    const auto bgColor = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    const auto inputFile = conf["2DLSystem"]["inputfile"].as_string_or_die();
    const auto lineColor = conf["2DLSystem"]["color"].as_double_tuple_or_die();

    return draw2DLines(L2D_to_Lines2D(inputFile, Color(lineColor[0], lineColor[1], lineColor[2])),
                       size, img::Color(lround(bgColor[0] * 255), lround(bgColor[1] * 255), lround(bgColor[2] * 255)));
}
