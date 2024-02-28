#include "Lines2D.cpp"
#include "../ini_configuration.h"
#include "../easy_image.h"
#include "l_parser/l_parser.h"

#include <string>
#include <cmath>
#include <fstream>
#include <stack>
#include <random>


// SOURCE: ChatGPT
// Create a random number generator engine
std::random_device randomDouble;
std::mt19937 gen(randomDouble());

// Create a uniform real distribution between 0 and 1
std::uniform_real_distribution<double> dis(0.0, 1.0);


Lines2D file_to_lines(const std::string &input, const Color &lineColor, bool random){
    LParser::LSystem2D l_system;
    std::ifstream input_stream(input);
    input_stream >> l_system;input_stream.close();

    // get constants
    std::set<char> alphabet = l_system.get_alphabet();
    std::string initiator = l_system.get_initiator();
    double angle = l_system.get_angle();
    double currentAngle = l_system.get_starting_angle();
    unsigned int iterations = l_system.get_nr_iterations();

    // replacement
    for (int i=0 ; i<iterations ; i++){
        std::string newString{};
        for (auto c : initiator){
            // do not replace char if random nr between 0 and 1 is smaller than the chance of replacement
            if (alphabet.find(c) == alphabet.end() or (random and dis(gen) > l_system.get_chance(c))){
                newString.push_back(c);
            }
            else{
                newString.append(l_system.get_replacement(c));
            }
        }
        initiator = newString;
    }

    // generate Lines2D
    Lines2D out;
    Point2D p1(0, 0);
    Point2D p2(0, 0);
    std::stack<std::pair<Point2D, double>> pointsStack;
    for (char c : initiator){
        if (alphabet.find(c) == alphabet.end()){
            if (c == '-') currentAngle -= angle;
            if (c == '+') currentAngle += angle;
            if (c == '('){
                pointsStack.emplace(p2, currentAngle);
            }
            if (c == ')' and !pointsStack.empty()) {
                p1 = pointsStack.top().first;
                p2 = pointsStack.top().first;
                currentAngle = pointsStack.top().second;
                pointsStack.pop();
            }
        } else {
            p2.x += cos(currentAngle * M_PI / 180);
            p2.y += sin(currentAngle * M_PI / 180);
            if (l_system.draw(c)) out.emplace_back(p1, p2, lineColor);
            p1.x = p2.x;
            p1.y = p2.y;
        }
    }
    return out;
}

img::EasyImage lines_to_img (const Lines2D &lines, const unsigned int size, const img::Color &bgColor){
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
