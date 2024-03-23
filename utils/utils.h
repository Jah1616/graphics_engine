#pragma once
#include <list>
#include <limits>
#include <vector>
#include <chrono>
#include "vector3d.h"
#include "../easy_image.h"


constexpr double toRadian(double x) {return x * M_PI / 180;}
constexpr double toDegrees(double x) {return x / M_PI * 180;}


class Timer{
private:
    const std::string _target;
    const std::chrono::time_point<std::chrono::high_resolution_clock> _startTime;
    std::ostream& _outputStream;
protected:
public:
    Timer(const std::string& target, std::ostream& out = std::cout)
    :_target(target)
    ,_startTime(std::chrono::high_resolution_clock::now())
    ,_outputStream(out) {}
    ~Timer(){Stop();}

    void Stop(){
        auto endTime = std::chrono::high_resolution_clock::now();

        auto start = std::chrono::time_point_cast<std::chrono::milliseconds>(_startTime).time_since_epoch();
        auto end = std::chrono::time_point_cast<std::chrono::milliseconds>(endTime).time_since_epoch();

        auto duration = end - start;
        _outputStream << _target << " duration: " << duration.count() << "ms\n";
    }
};


struct Color{double red; double green; double blue;
    Color(const double red, const double green, const double blue) :red(red) ,green(green) ,blue(blue) {}
    Color(const std::vector<double>& color) :red(color[0]), green(color[1]), blue(color[2]) {}
};
img::Color imgColor(const Color& color){
    return img::Color(img::Color(lround(color.red*255), lround(color.green*255), lround(color.blue*255)));
}
img::Color imgColor(const std::vector<double>& color){
    return img::Color(img::Color(lround(color[0]*255), lround(color[1]*255), lround(color[2]*255)));
}


struct Point2D{double x; double y;
    Point2D(const double x, const double y) :x(x), y(y) {}
};
struct Line2D{Point2D p1; Point2D p2; Color color; double z1; double z2;
    Line2D(const Point2D& p1, const Point2D& p2, const Color& color) :p1(p1), p2(p2), color(color), z1(), z2() {}
    Line2D(const Point2D& p1, const Point2D& p2, const double z1, const double z2, const Color& color)
    :p1(p1), p2(p2), color(color), z1(z1), z2(z2){}
    void invert(){
        std::swap(p1, p2);
        std::swap(z1, z2);
    }
};
typedef std::list<Line2D> Lines2D;


struct PointPolar{
    double r;
    double phi;
    double theta;
};
constexpr PointPolar toPolar(const Vector3D& point){
    double r = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    double theta = std::atan2(point.y, point.x);
    double phi = std::acos(point.z / r);
    return {r, phi, theta};
}


typedef std::vector<unsigned int> Face;
struct Figure3D{std::vector<Vector3D> points; std::vector<Face> faces; Color color;
    Figure3D(const std::vector<Vector3D>& points, const std::vector<Face>& faces, const Color& color)
    :points(points) ,faces(faces) ,color(color){}
    Figure3D(const Color& color): points{}, faces{}, color(color) {};
};
typedef std::list<Figure3D> Figures3D;


// Point2D operators
void operator *= (Point2D& lhs, const double rhs){
    lhs.x *= rhs;
    lhs.y *= rhs;
}
Point2D operator * (Point2D lhs, const double rhs){
    lhs *= rhs;
    return lhs;
}
void operator /= (Point2D& lhs, const double rhs){
    lhs.x /= rhs;
    lhs.y /= rhs;
}
Point2D operator / (Point2D lhs, const double rhs){
    lhs /= rhs;
    return lhs;
}
void operator += (Point2D& lhs, const Point2D& rhs){
    lhs.x += rhs.x;
    lhs.y += rhs.y;
}
Point2D operator + (Point2D lhs, const Point2D& rhs){
    lhs += rhs;
    return lhs;
}
void operator -= (Point2D& lhs, const Point2D& rhs){
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
}
Point2D operator - (Point2D lhs, const Point2D& rhs){
    lhs -= rhs;
    return lhs;
}

bool operator == (const Vector3D& lhs, const Vector3D& rhs){
    return ((lhs.is_point() and rhs.is_point()) or (lhs.is_vector() and rhs.is_vector()))
    and lhs.x == rhs.x and lhs.y == rhs.y and lhs.z == rhs.z;
}
bool operator == (const Point2D& lhs, const Point2D& rhs){
    return lhs.x == rhs.x and lhs.y == rhs.y;
}
