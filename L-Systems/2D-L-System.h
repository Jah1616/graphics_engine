#pragma once
#include "l_parser/l_parser.h"
#include "../utils/utils.h"
#include <string>
#include <fstream>
#include <stack>
#include <random>


// SOURCE: ChatGPT
std::random_device randomDouble;
std::mt19937 gen(randomDouble());
std::uniform_real_distribution<double> dis(0.0, 1.0);

Lines2D L2D_to_Lines2D(const std::string &input, const Color &lineColor){
    LParser::LSystem2D l_system;
    std::ifstream input_stream(input);
    input_stream >> l_system;
    input_stream.close();

    // get constants
    const auto alphabet = l_system.get_alphabet();
    auto initiator = l_system.get_initiator();
    const auto iterations = l_system.get_nr_iterations();

    // replacement
    for (auto i=0 ; i<iterations ; i++){
        std::string newString;
        for (auto c : initiator){
            if (alphabet.find(c) == alphabet.end()) newString.push_back(c);
            else {
                if (l_system.chances.find(c) == l_system.chances.end())
                    newString.append(l_system.get_replacement(c));
                else {
                    auto rand = dis(gen);
                    for (const auto &replacement : l_system.get_chances(c)){
                        rand -= replacement.second;
                        if (rand <= 0){
                            newString.append(replacement.first);
                            break;
                        }
                    }
                }
            }
        }
        initiator = newString;
    }

    // parse string
    const auto angle = l_system.get_angle() * M_PI/180;
    auto currentAngle = l_system.get_starting_angle() * M_PI/180;
    Lines2D out;
    Point2D p1(0, 0);
    Point2D p2(0, 0);
    std::stack<Point2D> pointsStack;
    std::stack<double> angleStack;

    for (auto c : initiator){
        if (alphabet.find(c) == alphabet.end()){
            if (c == '-') currentAngle -= angle;
            if (c == '+') currentAngle += angle;
            if (c == '('){
                pointsStack.push(p2);
                angleStack.push(currentAngle);
            }
            if (c == ')') {
                p1 = pointsStack.top();
                p2 = pointsStack.top();
                currentAngle = angleStack.top();
                pointsStack.pop();
                angleStack.pop();
            }
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
