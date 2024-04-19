#pragma once
#include <list>
#include <utility>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cassert>
#include <cmath>
#include "vector3d.h"
#include "../easy_image.h"


constexpr double toRadian(double x) {return x * M_PI / 180;}
constexpr double toDegrees(double x) {return x / M_PI * 180;}

// Timer
class Timer{
private:
    std::string _target;
    std::chrono::time_point<std::chrono::high_resolution_clock> _start;
    std::ostream& _outputStream;
protected:
public:
    explicit Timer(std::string  target, std::ostream& out = std::cout)
    : _target(std::move(target))
    , _start(std::chrono::high_resolution_clock::now())
    , _outputStream(out) {}
    ~Timer(){stop();}

    void stop(){
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - _start);
        _outputStream << _target << " duration: " << duration.count() << "ms\n";
    }
};

// Color
struct Color{double red; double green; double blue;
    Color(const double red, const double green, const double blue) :red(red) ,green(green) ,blue(blue) {}
    explicit Color(const std::vector<double>& color) :red(color[0]), green(color[1]), blue(color[2]) {
        assert(color.size() == 3);
    }
};
img::Color imgColor(const Color& color);
img::Color imgColor(const std::vector<double>& color);

// 2D utils
struct Point2D{double x; double y;
    Point2D(const double x, const double y) :x(x), y(y) {}
};
struct Line2D{Point2D p1; Point2D p2; Color color; double z1; double z2;
    Line2D(const Point2D& p1, const Point2D& p2, const Color& color, const double z1 = 0, const double z2 = 0)
    :p1(p1), p2(p2), color(color), z1(z1), z2(z2){}
    void invert(){
        std::swap(p1, p2);
        std::swap(z1, z2);
    }
};
using Lines2D = std::vector<Line2D>;

// 3D utils
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

using Face = std::vector<unsigned int>;
std::vector<Face> triangulate(const Face& face);

struct Figure3D{std::vector<Vector3D> points; std::vector<Face> faces; Color color;
    Figure3D(const std::vector<Vector3D>& points, const std::vector<Face>& faces, const Color& color)
    :points(points) ,faces(faces) ,color(color){}
    explicit Figure3D(const Color& color): points{}, faces{}, color(color) {};
};
typedef std::vector<Figure3D> Figures3D;

struct ImgVars{double scale; double dx; double dy; double imagex; double imagey;};
ImgVars getImgVars(const Lines2D& lines, unsigned int size);

// Point2D operators
void operator *= (Point2D& lhs, double rhs);
Point2D operator * (Point2D lhs, double rhs);
void operator /= (Point2D& lhs, double rhs);
Point2D operator / (Point2D lhs, double rhs);
void operator += (Point2D& lhs, const Point2D& rhs);
Point2D operator + (Point2D lhs, const Point2D& rhs);
void operator -= (Point2D& lhs, const Point2D& rhs);
Point2D operator - (Point2D lhs, const Point2D& rhs);

bool operator == (const Vector3D& lhs, const Vector3D& rhs);
bool operator == (const Point2D& lhs, const Point2D& rhs);


// ===================== DECLARATIONS =====================
// 3d_transformations
void applyTransform(Figure3D& f, const Matrix& m);
void applyTransform(Figures3D& f, const Matrix& m);
Matrix scale(double factor);
Matrix rotateX(double angle);
Matrix rotateY(double angle);
Matrix rotateZ(double angle);
Matrix translate(const Vector3D& vector);
Matrix eyePointTrans(const Vector3D& eyepoint);
Point2D doProjection(const Vector3D& point, double d = 1, double dx = 0, double dy = 0);
Lines2D doProjection(const Figures3D& figures);

// l-systems
void LSystem_2D(Lines2D& lines, const std::string &input, const Color &lineColor);
void LSystem_3D(Figure3D& figure, const std::string& input);

// platonic_bodies
void createCube(Figure3D& figure);
void createTetrahedron(Figure3D& figure);
void createOctahedron(Figure3D& figure);
void createIcosahedron(Figure3D& figure);
void createDodecahedron(Figure3D& figure);
void createCylinder(Figure3D& figure, unsigned int n, double h);
void createCone(Figure3D& figure, unsigned int n, double h);
void createSphere(Figure3D& figure, unsigned int n);
void createTorus(Figure3D& figure, double r, double R, unsigned int n, unsigned int m);
