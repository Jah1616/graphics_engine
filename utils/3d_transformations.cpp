#include <cmath>
#include "utils.h"

void applyTransform(Figure3D& f, const Matrix& m){
    for (auto& point : f.points) point *= m;
}

void applyTransform(Figures3D& f, const Matrix& m){
    for (auto& figure : f) applyTransform(figure, m);
}

Matrix scale(double factor){
    Matrix m;
    m(1, 1) = factor;
    m(2, 2) = factor;
    m(3, 3) = factor;
    m(4, 4) = 1;
    return m;
}

Matrix rotateX(double angle){
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
    assert(vector.is_vector());
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
    assert(eyepoint.is_point());
    Matrix m;
    const auto [r, phi, theta] = toPolar(eyepoint);
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

Point2D doProjection(const Vector3D& point, double d, double dx, double dy){
    return {d*point.x/-point.z + dx, d*point.y/-point.z + dy};
}

Lines2D doProjection(const Figures3D& figures){
    Lines2D lines;
    for (const auto& [points, faces, reflection] : figures){
        for (const auto& face : faces){
            const auto nrPoints = face.size();
            for (int i=0 ; i<nrPoints ; i++){
                const Vector3D& p1 = points[face[i]];
                const Vector3D& p2 = points[face[(i+1) % nrPoints]];
                lines.emplace_back(doProjection(p1), doProjection(p2), reflection.ambient, p1.z, p2.z);
            }
        }
    }
    return lines;
}
