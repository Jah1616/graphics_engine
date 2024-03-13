#pragma once

#include "vector3D.h"
//#include "../utils.hpp"
#include "../2D/2D_LSystem.hpp"
#include "../ini_configuration.h"
#include "../easy_image.h"

#include <vector>
#include <list>
#include <cmath>


using Face = std::vector<unsigned int>;

struct Figure3D{
    Figure3D(const Color& color): points{}, faces{}, color(color) {};
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Color color;
};

using Figures3D = std::list<Figure3D>;

void applyTransform(Figure3D& f, const Matrix& m){
    for (Vector3D point : f.points) point *= m;
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
        for (const Face& face : figure.faces){
            for (unsigned int i=0 ; i<face.size()-1 ; i++){
                lines.emplace_back(doProjection(figure.points[face[i]]),
                                   doProjection(figure.points[face[i+1]]),
                                   figure.color);
            }
            if (face.size() > 2) lines.emplace_back(doProjection(figure.points[face.back()]),
                               doProjection(figure.points[face.front()]),
                               figure.color);
        }
    }
    return lines;
}

img::EasyImage generate3DLinesImage(const ini::Configuration &conf){
    const unsigned int size = conf["General"]["size"].as_int_or_die();
    const std::vector<double> bgColor = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    const unsigned int nrFigs = conf["General"]["nrFigures"].as_int_or_die();
    const std::vector<double> eye = conf["General"]["eye"].as_double_tuple_or_die();
    const Vector3D eyePoint = Vector3D::vector(eye[0], eye[1], eye[2]);

    Figures3D figures;
    for (unsigned int i=0 ; i < nrFigs ; i++){
        std::string fig_string = "Figure" + std::to_string(i);

        const double fig_scale = conf[fig_string]["scale"].as_double_or_die();
        const double fig_rotateX = conf[fig_string]["rotateX"].as_double_or_die();
        const double fig_rotateY = conf[fig_string]["rotateY"].as_double_or_die();
        const double fig_rotateZ = conf[fig_string]["rotateZ"].as_double_or_die();
        const std::vector<double> fig_center = conf[fig_string]["center"].as_double_tuple_or_die();

        const std::vector<double> fig_color = conf[fig_string]["color"].as_double_tuple_or_die();
        Figure3D newFigure(Color(fig_color[0], fig_color[1], fig_color[2]));

        unsigned int nrPoints = conf[fig_string]["nrPoints"].as_int_or_die();
        for (unsigned int p=0 ; p < nrPoints ; p++){
            std::vector<double> coords = conf[fig_string]["point" + std::to_string(p)].as_double_tuple_or_die();
            Vector3D point = Vector3D::vector(coords[0], coords[1], coords[2]);
            newFigure.points.push_back(point);
        }

        unsigned int nrLines = conf[fig_string]["nrLines"].as_int_or_die();
        for (unsigned int l=0; l < nrLines ; l++){
            std::string lineString = "line" + std::to_string(l);
            std::vector<int> linePoints = conf[fig_string]["line" + std::to_string(l)].as_int_tuple_or_die();
            Face newFace;
            for (int p : linePoints) newFace.push_back(p);
            newFigure.faces.push_back(newFace);
        }

//        applyTransform(newFigure, scale(fig_scale)
//        *rotateX(fig_rotateX)*rotateY(fig_rotateY)*rotateZ(fig_rotateZ)
//        *translate(fig_center[0], fig_center[1], fig_center[2])
//        *eyePointTrans(eyePoint));

        applyTransform(newFigure, scale(fig_scale));
        applyTransform(newFigure, rotateX(fig_rotateX));
        applyTransform(newFigure, rotateY(fig_rotateY));
        applyTransform(newFigure, rotateZ(fig_rotateZ));
        applyTransform(newFigure, translate(Vector3D::vector(fig_center[0], fig_center[1], fig_center[2])));
        applyTransform(newFigure, eyePointTrans(eyePoint));

        figures.push_back(newFigure);
    }
    return draw2DLines(doProjection(figures), size, img::Color(lround(bgColor[0] * 255),
                                                               lround(bgColor[1] * 255),
                                                               lround(bgColor[2] * 255)));
}
