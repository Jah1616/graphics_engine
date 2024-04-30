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

static bool contains(const std::string& str, const std::string& part){
    return str.find(part) != std::string::npos;
}

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
    Color(double red, double green, double blue) :red(red) ,green(green) ,blue(blue) { limit(); }
    explicit Color(const std::vector<double>& color) :red(color[0]), green(color[1]), blue(color[2]) {
        assert(color.size() == 3);
        limit();
    }
    void operator += (const Color& rhs){
        red += rhs.red;
        green += rhs.green;
        blue += rhs.blue;
        limit();
    }
    void operator *= (const Color& rhs){
        red *= rhs.red;
        green *= rhs.green;
        blue *= rhs.blue;
        limit();
    }
private:
    void limit(){
        assert(0 <= red);
        assert(0 <= green);
        assert(0 <= blue);
        if (red > 1) red = 1;
        if (green > 1) green = 1;
        if (blue > 1) blue = 1;
    }
};
Color operator + (Color, const Color& rhs);
Color operator * (Color, const Color& rhs);

struct Light{ Color ambient; Color diffuse; };
struct InfLight : public Light { Vector3D direction; };
struct PointLight : public Light { Vector3D location; double angle; };
using Lights = std::list<Light>;

img::Color imgColor(const Color&);
img::Color imgColor(const std::vector<double>&);


// 2D utils
struct Point2D{double x; double y;
    void operator *= (double rhs){
        x *= rhs;
        y *= rhs;
    }
    void operator /= (double rhs){
        x /= rhs;
        y /= rhs;
    }
    void operator += (const Point2D& rhs){
        x += rhs.x;
        y += rhs.y;
    }
    void operator -= (const Point2D& rhs){
        x -= rhs.x;
        y -= rhs.y;
    }
    bool operator == (const Point2D& rhs) const {
        return x == rhs.x and y == rhs.y;
    }
};
Point2D operator * (Point2D, double);
Point2D operator / (Point2D, double);
Point2D operator + (Point2D, const Point2D&);
Point2D operator - (Point2D, const Point2D&);

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
struct PointPolar{ double r; double phi; double theta; };
constexpr PointPolar toPolar(const Vector3D& point){
    double r = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    double theta = std::atan2(point.y, point.x);
    double phi = std::acos(point.z / r);
    return {r, phi, theta};
}

using Face = std::vector<int>;
std::vector<Face> triangulate(const Face&);

struct Figure3D{
    std::vector<Vector3D> points; std::vector<Face> faces;
    Light reflection;

    Figure3D(const std::vector<Vector3D>& points, const std::vector<Face>& faces, const Color& color)
    :points(points), faces(faces), reflection{color}{}
    explicit Figure3D(const Light& reflection): points{}, faces{}, reflection(reflection) {};
};
bool operator == (const Vector3D&, const Vector3D&);
using Figures3D = std::vector<Figure3D>;
Figure3D compress(const Figures3D&, const Light&);

struct ImgVars{double scale; double dx; double dy; double imagex; double imagey;};
ImgVars getImgVars(const Lines2D&, int);


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
Lines2D LSystem2D(const Color&, const std::string&);
Figure3D LSystem3D(const Light&, const std::string&);

// platonic_bodies
Figure3D createCube(const Light&);
Figure3D createTetrahedron(const Light&);
Figure3D createOctahedron(const Light&);
Figure3D createIcosahedron(const Light&);
Figure3D createDodecahedron(const Light&);
Figure3D createCylinder(const Light&, int, double);
Figure3D createCone(const Light&, int, double);
Figure3D createSphere(const Light&, int);
Figure3D createTorus(const Light&, double, double, int, int);
