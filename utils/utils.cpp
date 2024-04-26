#include "utils.h"

img::Color imgColor(const Color& color){
    return img::Color(lround(color.red*255), lround(color.green*255), lround(color.blue*255));
}
img::Color imgColor(const std::vector<double>& color){
    assert(color.size() == 3);
    return img::Color(lround(color[0]*255), lround(color[1]*255), lround(color[2]*255));
}

std::vector<Face> triangulate(const Face& face){
    std::vector<Face> out;
    int first = face[0];
    for (int i=1 ; i<=face.size()-2 ; i++) out.push_back({first, face[i], face[i+1]});
    return out;
}

Figure3D compress(const Figures3D& figures, const Light& color){
    Figure3D out(color);

    for (const Figure3D& figure : figures){
        int points_amt = (int) out.points.size();
        for (Face face : figure.faces){
            for (int& index : face) index += points_amt;
            out.faces.push_back(face);
        }
        out.points.insert(out.points.end(), figure.points.begin(), figure.points.end());
    }

    return out;
}

ImgVars getImgVars(const Lines2D& lines, int size){
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

void operator *= (Point2D& lhs, double rhs){
    lhs.x *= rhs;
    lhs.y *= rhs;
}
Point2D operator * (Point2D lhs, double rhs){
    lhs *= rhs;
    return lhs;
}
void operator /= (Point2D& lhs, double rhs){
    lhs.x /= rhs;
    lhs.y /= rhs;
}
Point2D operator / (Point2D lhs, double rhs){
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