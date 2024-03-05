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



using Lines2D = std::list<Line2D>;
