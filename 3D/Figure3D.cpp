#include "vector3D.h"
//#include "../utils.hpp"
#include "../2D/Lines2D.cpp"

#include <vector>
#include <list>
#include <cmath>


using Face = std::vector<unsigned int>;

struct Figure3D{
    std::vector<Vector3D*> points;
    std::vector<Face*> faces;
    Color color;
};

using Figures3D = std::list<Figure3D>;


void applyTransform(Figure3D& f, const Matrix& m){
    for (Vector3D* point : f.points) *point *= m;
}

void applyTransform(Figures3D& f, const Matrix& m){
    for (Figure3D& figure : f) applyTransform(figure, m);
}

Matrix scale(const double factor){
    Matrix m;
    m(1, 1) = factor;
    m(2, 2) = factor;
    m(3, 3) = factor;
    m(4, 4) = 1;
    return m;
}

Matrix rotateX(double angle){
    angle *= M_PI/180;
    Matrix m;
    m(1, 1) = 1;
    m(2, 2) = cos(angle);
    m(2, 3) = sin(angle);
    m(3, 2) = -sin(angle);
    m(3, 3) = cos(angle);
    m(4, 4) = 1;
    return m;
}

Matrix rotateY(double angle){
    angle *= M_PI/180;
    Matrix m;
    m(1, 1) = cos(angle);
    m(1, 3) = -sin(angle);
    m(2, 2) = 1;
    m(3, 1) = sin(angle);
    m(3, 3) = cos(angle);
    m(4, 4) = 1;
    return m;
}

Matrix rotateZ(double angle){
    angle *= M_PI/180;
    Matrix m;
    m(1, 1) = cos(angle);
    m(1, 2) = sin(angle);
    m(2, 1) = -sin(angle);
    m(2, 2) = cos(angle);
    m(3, 3) = 1;
    m(4, 4) = 1;
    return m;
}

Matrix translate(const double x, const double y, const double z){
    Matrix m;
    m(1, 1) = 1;
    m(2, 2) = 1;
    m(3, 3) = 1;
    m(4, 4) = 1;
    m(4, 1) = x;
    m(4, 2) = y;
    m(4, 3) = z;
    return m;
}

void toPolar(const Vector3D& point, double &theta, double &phi, double &r){
    r = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    theta = atan2(point.y, point.x);
    phi = acos(point.z / r);
}

Matrix eyePointTrans(const Vector3D& eyepoint){
    Matrix m;
    double theta, phi, r;
    toPolar(eyepoint, theta, phi, r);
    m(1, 1) = -sin(theta);
    m(1, 2) = -cos(theta)*cos(phi);
    m(1, 3) = cos(theta)*sin(phi);
    m(2, 1) = cos(theta);
    m(2, 2) = -sin(theta)*cos(phi);
    m(2, 3) = sin(theta)*sin(phi);
    m(3, 2) = sin(phi);
    m(3, 3) = cos(phi);
    m(4, 3) = -r;
    m(4, 4) = 1;
    return m;
}

Point2D doProjection(const Vector3D& point, const double d = 1){
    return {d*point.x/-point.z, d*point.y/-point.z};
}

Lines2D doProjection(const Figures3D& figs){
    Lines2D lines;
    for (const Figure3D& figure : figs){
        for (Face* face : figure.faces){
            for (unsigned int i=0 ; i<face->size()-1 ; i++){
                lines.emplace_back(doProjection(*figure.points[(*face)[i]]),
                                   doProjection(*figure.points[(*face)[i+1]]),
                                   figure.color);
            }
            lines.emplace_back(doProjection(*figure.points[face->back()]),
                               doProjection(*figure.points[face->front()]),
                               figure.color);
        }
    }
    return lines;
}


