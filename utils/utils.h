#pragma once
#include <list>
#include <limits>
#include <utility>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cassert>
#include "vector3d.h"
#include "../easy_image.h"


constexpr double toRadian(double x) {return x * M_PI / 180;}
constexpr double toDegrees(double x) {return x / M_PI * 180;}

// Timer
class Timer{
private:
    const std::string _target;
    const std::chrono::time_point<std::chrono::high_resolution_clock> _start;
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
inline img::Color imgColor(const Color& color){
    return img::Color(img::Color(lround(color.red*255), lround(color.green*255), lround(color.blue*255)));
}
inline img::Color imgColor(const std::vector<double>& color){
    return img::Color(img::Color(lround(color[0]*255), lround(color[1]*255), lround(color[2]*255)));
}

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
typedef std::vector<Line2D> Lines2D;

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

typedef std::vector<unsigned int> Face;
static std::vector<Face> triangulate(const Face& face){
    std::vector<Face> out;
    unsigned int first = face[0];
    for (unsigned int i=1 ; i<=face.size()-2 ; i++) out.push_back({first, face[i], face[i+1]});
    return out;
}

struct Figure3D{std::vector<Vector3D> points; std::vector<Face> faces; Color color;
    Figure3D(const std::vector<Vector3D>& points, const std::vector<Face>& faces, const Color& color)
    :points(points) ,faces(faces) ,color(color){}
    explicit Figure3D(const Color& color): points{}, faces{}, color(color) {};
};
typedef std::vector<Figure3D> Figures3D;

struct ImgVars{double scale; double dx; double dy; double imagex; double imagey;};
static ImgVars getImgVars(const Lines2D& lines, const unsigned int size){
    double xmin = std::numeric_limits<double>::infinity();
    double ymin = xmin;
    double xmax = -std::numeric_limits<double>::infinity();
    double ymax = xmax;

    for (const auto& line : lines){
        xmin = std::min({xmin, line.p1.x, line.p2.x});
        xmax = std::max({xmax, line.p1.x, line.p2.x});
        ymin = std::min({ymin, line.p1.y, line.p2.y});
        ymax = std::max({ymax, line.p1.y, line.p2.y});
    }

    const double xrange = xmax - xmin;
    const double yrange = ymax - ymin;
    const double imagex = size * xrange / fmax(xrange, yrange);
    const double imagey = size * yrange / fmax(xrange, yrange);

    const double scale = 0.95 * imagex / xrange;
    const double dx = (imagex - scale * (xmin + xmax)) / 2;
    const double dy = (imagey - scale * (ymin + ymax)) / 2;

    return {scale, dx, dy, imagex, imagey};
}

// Point2D operators
static void operator *= (Point2D& lhs, const double rhs){
    lhs.x *= rhs;
    lhs.y *= rhs;
}
static Point2D operator * (Point2D lhs, const double rhs){
    lhs *= rhs;
    return lhs;
}
static void operator /= (Point2D& lhs, const double rhs){
    lhs.x /= rhs;
    lhs.y /= rhs;
}
static Point2D operator / (Point2D lhs, const double rhs){
    lhs /= rhs;
    return lhs;
}
static void operator += (Point2D& lhs, const Point2D& rhs){
    lhs.x += rhs.x;
    lhs.y += rhs.y;
}
static Point2D operator + (Point2D lhs, const Point2D& rhs){
    lhs += rhs;
    return lhs;
}
static void operator -= (Point2D& lhs, const Point2D& rhs){
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
}
static Point2D operator - (Point2D lhs, const Point2D& rhs){
    lhs -= rhs;
    return lhs;
}

static bool operator == (const Vector3D& lhs, const Vector3D& rhs){
    return ((lhs.is_point() and rhs.is_point()) or (lhs.is_vector() and rhs.is_vector()))
    and lhs.x == rhs.x and lhs.y == rhs.y and lhs.z == rhs.z;
}
static bool operator == (const Point2D& lhs, const Point2D& rhs){
    return lhs.x == rhs.x and lhs.y == rhs.y;
}


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
