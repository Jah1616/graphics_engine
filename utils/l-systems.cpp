#include <string>
#include <fstream>
#include <stack>
#include <random>
#include <array>
#include "l-parser.h"
#include "utils.h"

// SOURCE: ChatGPT
static std::random_device randomDouble;
static std::mt19937 gen(randomDouble());
static std::uniform_real_distribution<double> dis(0.0, 1.0);

void LSystem2D(Lines2D& lines, const std::string& input, const Color& lineColor){
//    Timer timer("2D-L-System");
    LParser::LSystem2D l_system;
    std::ifstream input_stream(input);
    input_stream >> l_system;
    input_stream.close();

    // get constants
    const std::set<char>& alphabet = l_system.get_alphabet();
    std::string currentString = l_system.get_initiator();
    const auto& iterations = l_system.get_nr_iterations();

    // replacement
    for (auto i=0 ; i<iterations ; i++){
        std::string newString;
        for (char c : currentString){
            if (alphabet.find(c) == alphabet.end()) newString += c;
            else {
                if (l_system.chances.find(c) == l_system.chances.end())
                    newString += l_system.get_replacement(c);
                else {
                    double rand = dis(gen);
                    for (const auto& [replacement, chance] : l_system.get_chances(c)){
                        rand -= chance;
                        if (rand <= 0){
                            newString += replacement;
                            break;
                        }
                    }
                }
            }
        }
        currentString = newString;
    }

    // parse string
    const double angle = toRadian(l_system.get_angle());
    double currentAngle = toRadian(l_system.get_starting_angle());
    Point2D p1(0, 0);
    std::stack<std::pair<Point2D, double>> bracketStack;

    for (char c : currentString){
        if (alphabet.find(c) != alphabet.end()){
            Point2D p2(p1.x + cos(currentAngle), p1.y + sin(currentAngle));
            if (l_system.draw(c)) lines.emplace_back(p1, p2, lineColor);
            p1 = p2;
        }
        else if (c == '-') currentAngle -= angle;
        else if (c == '+') currentAngle += angle;
        else if (c == '(') bracketStack.emplace(p1, currentAngle);
        else if (c == ')') {
            p1 = bracketStack.top().first;
            currentAngle = bracketStack.top().second;
            bracketStack.pop();
        }
    }
}

void LSystem3D(Figure3D& figure, const std::string& input){
//    Timer timer("3D-L-System");
    LParser::LSystem3D l_system;
    std::ifstream input_stream(input);
    input_stream >> l_system;input_stream.close();

    // get constants
    const std::set<char>& alphabet = l_system.get_alphabet();
    std::string initiator = l_system.get_initiator();
    const auto& iterations = l_system.get_nr_iterations();

    // replacement
    for (int i=0 ; i<iterations ; i++){
        std::string newString{};
        for (char c : initiator){
            if (alphabet.find(c) == alphabet.end()) newString.push_back(c);
            else newString.append(l_system.get_replacement(c));
        }
        initiator = newString;
    }

    // parse string
    const double angle = toRadian(l_system.get_angle());
    const double cosAngle = cos(angle);
    const double sinAngle = sin(angle);
    const double sinAngleNeg = -sinAngle;
    Vector3D H = Vector3D::vector(1, 0, 0);
    Vector3D L = Vector3D::vector(0, 1, 0);
    Vector3D U = Vector3D::vector(0, 0, 1);
    Vector3D p1 = Vector3D::point(0, 0, 0);
    Vector3D p2 = Vector3D::point(0, 0, 0);

    std::stack<std::array<Vector3D, 4>> bracketStack;

    for (char c : initiator){
        if (alphabet.find(c) != alphabet.end()){
            p2 += H;
            if (l_system.draw(c)){
                auto p1_it = std::find(figure.points.begin(), figure.points.end(), p1);
                auto p1_index = (int)(p1_it - figure.points.begin());
                if (p1_it == figure.points.end()){
                    figure.points.push_back(p1);
                    p1_index = (int)figure.points.size() - 1;
                }

                auto p2_it = std::find(figure.points.begin(), figure.points.end(), p2);
                auto p2_index = (int)(p2_it - figure.points.begin());
                if (p2_it == figure.points.end()){
                    figure.points.push_back(p2);
                    p2_index = (int)figure.points.size() - 1;
                }

                figure.faces.push_back({p1_index, p2_index});
            }
            p1 = p2;
        }
        else if (c == '+'){ // turn left
            Vector3D Hnew = H*cosAngle + L*sinAngle;
            Vector3D Lnew = -H*sinAngle + L*cosAngle;
            H=Hnew; L=Lnew;
        }
        else if (c == '-'){ // turn right
            Vector3D Hnew = H*cosAngle + L*sinAngleNeg;
            Vector3D Lnew = -H*sinAngleNeg + L*cosAngle;
            H=Hnew; L=Lnew;
        }
        else if (c == '^'){ // pitch up
            Vector3D Hnew = H*cosAngle + U*sinAngle;
            Vector3D Unew = -H*sinAngle + U*cosAngle;
            H=Hnew; U=Unew;
        }
        else if (c == '&'){ // pitch down
            Vector3D Hnew = H*cosAngle + U*sinAngleNeg;
            Vector3D Unew = -H*sinAngleNeg + U*cosAngle;
            H=Hnew; U=Unew;
        }
        else if (c == '\\'){ // roll left
            Vector3D Lnew = L*cosAngle - U*sinAngle;
            Vector3D Unew = L*sinAngle + U*cosAngle;
            L=Lnew; U=Unew;
        }
        else if (c == '/'){ // roll right
            Vector3D Lnew = L*cosAngle - U*sinAngleNeg;
            Vector3D Unew = L*sinAngleNeg + U*cosAngle;
            L=Lnew; U=Unew;
        }
        else if (c == '|'){ // turn back
            H *= -1;
            L *= -1;
        }
        else if (c == '(') bracketStack.push({H, L, U, p1});
        else if (c == ')'){
            const auto& [h, l, u, p] = bracketStack.top();
            H = h;
            L = l;
            U = u;
            p1 = p2 = p;
            bracketStack.pop();
        }
    }
}
