#pragma once
#include <list>
#include <limits>
#include <vector>
#include "vector3d.h"
#include "../easy_image.h"


double toRadian(double x) {return x * M_PI / 180;}
double toDegrees(double x) {return x / M_PI * 180;}


struct Color{
    Color(const double red, const double green, const double blue) :red(red) ,green(green) ,blue(blue) {}
    double red;
    double green;
    double blue;
};


struct Point2D{
    Point2D(const double x, const double y) :x(x), y(y) {}
    double x;
    double y;
};
struct Line2D{
    Line2D(const Point2D& p1, const Point2D& p2, const Color& color) :p1(p1), p2(p2), color(color) {}
    Line2D(const Point2D& p1, const Point2D& p2, const double z1, const double z2, const Color& color)
    :p1(p1), p2(p2), color(color), z1(z1), z2(z2){}
    Point2D p1;
    Point2D p2;
    Color color;
    double z1;
    double z2;
};
typedef std::list<Line2D> Lines2D;


typedef std::vector<unsigned int> Face;
struct Figure3D{
    Figure3D(const std::vector<Vector3D>& points, const std::vector<Face>& faces, const Color& color)
    :points(points) ,faces(faces) ,color(color){}
    Figure3D(const Color& color): points{}, faces{}, color(color) {};
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Color color;
};
typedef std::list<Figure3D> Figures3D;
