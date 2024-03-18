#pragma once
#include "l_parser/l_parser.h"
#include "../utils/utils.h"
#include <string>
#include <cmath>
#include <fstream>
#include <stack>


void L3D_to_Figure3D(const std::string& input, Figure3D& figure){
    LParser::LSystem3D l_system;
    std::ifstream input_stream(input);
    input_stream >> l_system;input_stream.close();

    // get constants
    const std::set<char> alphabet = l_system.get_alphabet();
    std::string initiator = l_system.get_initiator();
    const unsigned int iterations = l_system.get_nr_iterations();

    // replacement
    for (unsigned int i=0 ; i<iterations ; i++){
        std::string newString;
        for (char c : initiator){
            if (alphabet.find(c) == alphabet.end()) newString.push_back(c);
            else newString.append(l_system.get_replacement(c));
        }
        initiator = newString;
    }

    // parse string
    const double angle = l_system.get_angle() * M_PI / 180;
    Vector3D H = Vector3D::vector(1, 0, 0);
    Vector3D L = Vector3D::vector(0, 1, 0);
    Vector3D U = Vector3D::vector(0, 0, 1);
    Vector3D p1 = Vector3D::point(0, 0, 0);
    Vector3D p2 = Vector3D::point(0, 0, 0);

    std::stack<Vector3D> vectorStack;
    std::stack<Vector3D> pointsStack;
    std::stack<unsigned int> indexStack;
    unsigned int index = 0;

    for (char c : initiator){
        if (alphabet.find(c) == alphabet.end()){
            if (c == '+'){ // turn left
                Vector3D Hnew = H*cos(angle) + L*sin(angle);
                Vector3D Lnew = -H*sin(angle) + L*cos(angle);
                H=Hnew; L=Lnew;
            }
            else if (c == '-'){ // turn right
                Vector3D Hnew = H*cos(-angle) + L*sin(-angle);
                Vector3D Lnew = -H*sin(-angle) + L*cos(-angle);
                H=Hnew; L=Lnew;
            }
            else if (c == '^'){ // pitch up
                Vector3D Hnew = H*cos(angle) + U*sin(angle);
                Vector3D Unew = -H*sin(angle) + U*cos(angle);
                H=Hnew; U=Unew;
            }
            else if (c == '&'){ // pitch down
                Vector3D Hnew = H*cos(-angle) + U*sin(-angle);
                Vector3D Unew = -H*sin(-angle) + U*cos(-angle);
                H=Hnew; U=Unew;
            }
            else if (c == '\\'){ // roll left
                Vector3D Lnew = L*cos(angle) - U*sin(angle);
                Vector3D Unew = L*sin(angle) + U*cos(angle);
                L=Lnew; U=Unew;
            }
            else if (c == '/'){ // roll right
                Vector3D Lnew = L*cos(-angle) - U*sin(-angle);
                Vector3D Unew = L*sin(-angle) + U*cos(-angle);
                L=Lnew; U=Unew;
            }
            else if (c == '|'){ // turn back
                H *= -1;
                L *= -1;
            }
            else if (c == '('){
                vectorStack.push(H);
                vectorStack.push(L);
                vectorStack.push(U);
                pointsStack.push(p1);
            }
            else if (c == ')'){
                U = vectorStack.top(); vectorStack.pop();
                L = vectorStack.top(); vectorStack.pop();
                H = vectorStack.top(); vectorStack.pop();
                p1 = pointsStack.top(); pointsStack.pop();
                p2 = p1;
            }
        } else {
            p2 += H;
            if (l_system.draw(c)){
                if (!figure.points.empty() and p1.x == figure.points.back().x and p1.y == figure.points.back().y and p1.z == figure.points.back().z){
                    figure.points.push_back(p2);
                    figure.faces.push_back({index-1, index});
                    ++index;
                } else {
                    figure.points.push_back(p1);
                    figure.points.push_back(p2);
                    figure.faces.push_back({index, index+1});
                    index += 2;
                }
            }
            p1 = p2;
        }
    }
}
