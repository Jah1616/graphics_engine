#include "Lines2D.cpp"
#include "../ini_configuration.h"
#include "../easy_image.h"
#include "l_parser/l_parser.h"

#include <string>
#include <cmath>
#include <fstream>


img::EasyImage draw2DLines (const Lines2D &lines, const unsigned int size, const img::Color &bgColor){
    double xmin{0};
    double xmax{0};
    double ymin{0};
    double ymax{0};
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
    double scale = 0.95 * imagex / xrange;

    for (auto line : lines){
        line.p1.x *= scale;
        line.p1.y *= scale;
        line.p2.x *= scale;
        line.p2.y *= scale;
    }

    img::EasyImage image(50, 50);
    image.clear(bgColor);

    return image;
}

Lines2D LSystem(const std::string &input, const Color &lineColor){
    LParser::LSystem2D l_system;
    std::ifstream input_stream(input);
    input_stream >> l_system;input_stream.close();

    std::set<char> alphabet = l_system.get_alphabet();
    std::string initiator = l_system.get_initiator();
    double angle = l_system.get_starting_angle();
    double currentAngle = l_system.get_starting_angle();
    unsigned int iterations = l_system.get_nr_iterations();

    for (int i=0 ; i<iterations ; i++){
        std::string newString{};
        for (auto c : initiator){
            newString.append((alphabet.find(c) == alphabet.end()) ?
                             std::to_string(c) : l_system.get_replacement(c));
        }
        initiator = newString;
    }

    Lines2D out;
    Point2D p1(0, 0);
    Point2D p2(0, 0);
    for (auto c : initiator){
        if (alphabet.find(c) == alphabet.end()){
            if (c == '-') currentAngle -= angle;
            if (c == '+') currentAngle += angle;
        } else {
            p2.x += cos(currentAngle);
            p2.y += sin(currentAngle);
            if (l_system.draw(c)) out.emplace_back(p1, p2, lineColor);
            p1.x = p2.x;
            p1.y = p2.y;
        }
    }
    return out;
}

img::EasyImage generate2DLinesImage(const ini::Configuration &conf){
    unsigned int size = conf["General"]["size"].as_int_or_die();
    std::vector<double> bgColor = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    std::string inputFile = conf["2DLSystem"]["inputfile"].as_string_or_die();
    std::vector<double> color = conf["2DLSystem"]["color"].as_double_tuple_or_die();
    img::EasyImage image;

    image = draw2DLines(LSystem(inputFile, Color(color[0],color[1],color[2])),
                        size,
                        img::Color(lround(bgColor[0]*255),lround(bgColor[1]*255),lround(bgColor[2]*255)));

    return image;
}
