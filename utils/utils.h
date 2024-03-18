#pragma once
#include "vector3D.h"
#include <list>


struct Color{
    Color(double red, double green, double blue) :red(red) ,green(green) ,blue(blue) {}
    double red;
    double green;
    double blue;
};


struct Point2D{
    Point2D(double x, double y) :x(x), y(y) {}
    double x;
    double y;
};
struct Line2D{
    Line2D(Point2D p1, Point2D p2, Color color) :p1(p1), p2(p2), color(color) {}
    Point2D p1;
    Point2D p2;
    Color color;
};
typedef std::list<Line2D> Lines2D;


typedef std::vector<unsigned int> Face;
struct Figure3D{
    Figure3D(const std::vector<Vector3D>& points, const std::vector<Face>& faces, const Color& color)
            :points(points)
            ,faces(faces)
            ,color(color){}
    Figure3D(const Color& color): points{}, faces{}, color(color) {};
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Color color;
};
typedef std::list<Figure3D> Figures3D;
