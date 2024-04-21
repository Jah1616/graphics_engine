#pragma once
#include <list>
#include <utility>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <set>
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
    explicit Timer(std::string target, std::ostream& out = std::cout)
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
    Color(double red, double green, double blue) :red(red) ,green(green) ,blue(blue) { checkRange(); }
    explicit Color(const std::vector<double>& color) :red(color[0]), green(color[1]), blue(color[2]) {
        assert(color.size() == 3);
        checkRange();
    }
private:
    void checkRange() const {
        assert(0 <= red and red <= 1);
        assert(0 <= green and green <= 1);
        assert(0 <= blue and blue <= 1);
    }
};
img::Color imgColor(const Color&);
img::Color imgColor(const std::vector<double>&);

// 2D utils
struct Point2D{double x; double y;
    Point2D(double x, double y) :x(x), y(y) {}
};
struct Line2D{Point2D p1; Point2D p2; Color color; double z1; double z2;
    Line2D(const Point2D& p1, const Point2D& p2, const Color& color, double z1 = 0, double z2 = 0)
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

using Face = std::vector<int>;
std::vector<Face> triangulate(const Face&);

struct Figure3D{std::vector<Vector3D> points; std::vector<Face> faces; Color color;
    Figure3D(const std::vector<Vector3D>& points, const std::vector<Face>& faces, const Color& color)
    :points(points) ,faces(faces) ,color(color){}
    explicit Figure3D(const Color& color): points{}, faces{}, color(color) {};
};
typedef std::vector<Figure3D> Figures3D;

struct ImgVars{double scale; double dx; double dy; double imagex; double imagey;};
ImgVars getImgVars(const Lines2D&, int);

// Point2D operators
void operator *= (Point2D&, double);
Point2D operator * (Point2D, double);
void operator /= (Point2D&, double);
Point2D operator / (Point2D, double);
void operator += (Point2D&, const Point2D&);
Point2D operator + (Point2D, const Point2D&);
void operator -= (Point2D&, const Point2D&);
Point2D operator - (Point2D, const Point2D&);

bool operator == (const Vector3D&, const Vector3D&);
bool operator == (const Point2D&, const Point2D&);


// ===================== DECLARATIONS =====================
// 3d_transformations
void applyTransform(Figure3D&, const Matrix&);
void applyTransform(Figures3D&, const Matrix&);
Matrix scale(double);
Matrix rotateX(double);
Matrix rotateY(double);
Matrix rotateZ(double);
Matrix translate(const Vector3D&);
Matrix eyePointTrans(const Vector3D&);
Point2D doProjection(const Vector3D&, double = 1, double = 0, double = 0);
Lines2D doProjection(const Figures3D&);

// l-systems
void LSystem2D(Lines2D&, const std::string&, const Color&);
void LSystem3D(Figure3D&, const std::string&);

// platonic_bodies
void createCube(Figure3D&);
void createTetrahedron(Figure3D&);
void createOctahedron(Figure3D&);
void createIcosahedron(Figure3D&);
void createDodecahedron(Figure3D&);
void createCylinder(Figure3D&, int, double);
void createCone(Figure3D&, int, double);
void createSphere(Figure3D&, int);
void createTorus(Figure3D&, double, double, int, int);
