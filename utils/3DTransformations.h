#pragma once
#include "utils.h"
#include <cmath>


void applyTransform(Figure3D& f, const Matrix& m){
    for (auto& point : f.points) point *= m;
}

void applyTransform(Figures3D& f, const Matrix& m){
    for (auto& figure : f) applyTransform(figure, m);
}

Matrix scale(const double factor){
    Matrix m;
    m(1, 1) = factor;
    m(2, 2) = factor;
    m(3, 3) = factor;
    m(4, 4) = 1;
    return m;
}

Matrix rotateX(const double angle){
    Matrix m;
    m(1, 1) = 1;
    m(2, 2) = cos(angle);
    m(2, 3) = sin(angle);
    m(3, 2) = -sin(angle);
    m(3, 3) = cos(angle);
    m(4, 4) = 1;
    return m;
}

Matrix rotateY(const double angle){
    Matrix m;
    m(1, 1) = cos(angle);
    m(1, 3) = -sin(angle);
    m(2, 2) = 1;
    m(3, 1) = sin(angle);
    m(3, 3) = cos(angle);
    m(4, 4) = 1;
    return m;
}

Matrix rotateZ(const double angle){
    Matrix m;
    m(1, 1) = cos(angle);
    m(1, 2) = sin(angle);
    m(2, 1) = -sin(angle);
    m(2, 2) = cos(angle);
    m(3, 3) = 1;
    m(4, 4) = 1;
    return m;
}

Matrix translate(const Vector3D& vector){
    Matrix m;
    m(1, 1) = 1;
    m(2, 2) = 1;
    m(3, 3) = 1;
    m(4, 4) = 1;
    m(4, 1) = vector.x;
    m(4, 2) = vector.y;
    m(4, 3) = vector.z;
    return m;
}

void toPolar(const Vector3D& point, double &theta, double &phi, double &r){
    r = point.length();
    theta = std::atan2(point.y, point.x);
    phi = std::acos(point.z / r);
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

Lines2D doProjection(const Figures3D& figures){
    Lines2D lines;
    for (const auto& figure : figures){
        for (const auto& face : figure.faces){
            auto nrPoints = face.size();
            for (auto i=0 ; i<nrPoints ; i++){
                lines.emplace_back(doProjection(figure.points[face[i]]),
                                   doProjection(figure.points[face[(i+1) % nrPoints]]),
                                   figure.color);
            }
        }
    }
    return lines;
}
