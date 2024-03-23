#pragma once
#include <cmath>
#include "utils.h"


void applyTransform(Figure3D& f, const Matrix& m){
    for (Vector3D& point : f.points) point *= m;
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
    if (!vector.is_vector()) {std::cerr << "Can not call translate() with a non-vector object.\n";}
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

Matrix eyePointTrans(const Vector3D& eyepoint){
    Matrix m;
    auto [r, phi, theta] = toPolar(eyepoint);
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
    for (const auto& [points, faces, color] : figures){
        for (const Face& face : faces){
            unsigned int nrPoints = face.size();
            for (unsigned int i=0 ; i<nrPoints ; i++){
                Vector3D p1 = points[face[i]];
                Vector3D p2 = points[face[(i+1) % nrPoints]];
                lines.emplace_back(doProjection(p1), doProjection(p2), p1.z, p2.z, color);
            }
        }
    }
    return lines;
}
