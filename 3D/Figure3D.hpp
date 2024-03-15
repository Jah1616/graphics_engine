#pragma once

//#include "../utils.hpp"
#include "vector3D.h"
#include "../2D/2D_LSystem.hpp"
#include "../ini_configuration.h"
#include "../easy_image.h"

#include <vector>
#include <list>
#include <cmath>


using Face = std::vector<unsigned int>;

struct Figure3D{
    Figure3D(const std::vector<Vector3D>& points, const std::vector<Face>& faces, const Color& color)
    :points(points)
    ,faces(faces)
    ,color(color){}
    Figure3D(const Color& color): points{}, faces{}, color(color) {};
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Color color;
};

using Figures3D = std::vector<Figure3D>;

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
    for (const Figure3D& figure : figures){
        for (const Face& face : figure.faces){
            unsigned int nrPoints = face.size();
            for (unsigned int i=0 ; i<nrPoints ; i++){
                lines.emplace_back(doProjection(figure.points[face[i]]),
                                   doProjection(figure.points[face[(i+1) % nrPoints]]),
                                   figure.color);
            }
        }
    }
    return lines;
}

Figure3D createCube(const Color& color){
    return {{Vector3D::point(1,-1,-1), Vector3D::point(-1,1,-1),
             Vector3D::point(1,1,1), Vector3D::point(-1,-1,1),
             Vector3D::point(1,1,-1), Vector3D::point(-1,-1,-1),
             Vector3D::point(1,-1,1), Vector3D::point(-1,1,1)},
            {{0,4,2,6}, {4,1,7,2}, {1,5,3,7}, {5,0,6,3}, {6,2,7,3}, {0,5,1,4}},
            color};
}

Figure3D createTetrahedron(const Color& color){
    return {{Vector3D::point(1,-1,-1), Vector3D::point(-1,1,-1),
             Vector3D::point(1,1,1), Vector3D::point(-1,-1,1)},
            {{0,1,2}, {1,3,2}, {0,3,1}, {0,2,3}},
            color};
}

Figure3D createOctahedron(const Color& color){
    return {{Vector3D::point(1,0,0), Vector3D::point(0,1,0),
             Vector3D::point(-1,0,0), Vector3D::point(0,-1,0),
             Vector3D::point(0,0,-1), Vector3D::point(0,0,1)},
            {{0,1,5}, {1,2,5}, {2,3,5}, {3,0,5}, {1,0,4}, {2,1,4}, {3,2,4}, {0,3,4}},
            color};
}

Figure3D createIcosahedron(const Color& color){
    std::vector<Vector3D> points;
    points.push_back(Vector3D::point(0, 0, sqrt(5)/2));
    for (unsigned int i=2 ; i<=6 ; ++i){
        points.push_back(Vector3D::point(cos((i-2)*2*M_PI/5), sin((i-2)*2*M_PI/5), 0.5));
    }
    for (unsigned int i=7 ; i<=11 ; ++i){
        points.push_back(Vector3D::point(cos(M_PI/5 + (i-7)*2*M_PI/5),
                                         sin(M_PI/5 + (i-7)*2*M_PI/5),
                                         -0.5));
    }
    points.push_back(Vector3D::point(0, 0, -sqrt(5)/2));

    return {points,
            {{0,1,2}, {0,2,3}, {0,3,4}, {0,4,5}, {0,5,1}, {1,6,2}, {2,6,7}, {2,7,3}, {3,7,8}, {3,8,4}, {4,8,9},
             {4,9,5}, {5,9,10}, {5,10,1}, {1,10,6}, {11,7,6}, {11,8,7}, {11,9,8}, {11,10,9}, {11,6,10}},
            color};
}

Figure3D createDodecahedron(const Color& color){
    Figure3D ico = createIcosahedron({0, 0, 0});
    std::vector<Vector3D> points;
    for (Face face : ico.faces){
        points.push_back(Vector3D::point((ico.points[face[0]].x + ico.points[face[1]].x + ico.points[face[2]].x)/3,
                                         (ico.points[face[0]].y + ico.points[face[1]].y + ico.points[face[2]].y)/3,
                                         (ico.points[face[0]].z + ico.points[face[1]].z + ico.points[face[2]].z)/3));
    }

    return {points,
            {{0,1,2,3,4}, {0,5,6,7,1}, {1,7,8,9,2}, {2,9,10,11,3}, {3,11,12,13,4}, {4,13,14,5,0},
             {19,18,17,16,15}, {19,14,13,12,18}, {18,12,11,10,17}, {17,10,9,8,16}, {16,8,7,6,15}, {15,6,5,14,19}},
            color};
}

Figure3D createCylinder(const unsigned int n, const double h, const Color& color){
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Face base;
    for (unsigned int i=0 ; i<n ; ++i){
        points.push_back(Vector3D::point(cos(2*i*M_PI/n), sin(2*i*M_PI/n), 0));
        base.push_back(n-i-1);
    }

    Face top;
    for (unsigned int i=0 ; i<n ; ++i){
        points.push_back(Vector3D::point(cos(2*i*M_PI/n), sin(2*i*M_PI/n), h));
        top.push_back(n+i);
        faces.push_back({i, (i+1)%n, (i+1)%n + n, i+n});
    }
    faces.push_back(base);
    faces.push_back(top);

    return {points, faces, color};
}

Figure3D createCone(const unsigned int n, const double h, const Color& color){
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Face base;

    for (unsigned int i=0 ; i<n ; ++i){
        points.push_back(Vector3D::point(cos(2*i*M_PI/n), sin(2*i*M_PI/n), 0));
        faces.push_back({i, (i + 1) % n, n});
        base.push_back(n-i-1);
    }
    points.push_back(Vector3D::point(0, 0, h));
    faces.push_back(base);

    return {points, faces, color};
}

Figure3D createSphere(const unsigned int n, const Color& color){
    Figure3D ico = createIcosahedron(color);
    Figure3D sphere{color};
    for (unsigned int i=0 ; i<n ; ++i){
        sphere = {{}, {}, color};
        for (unsigned int idx=0 ; idx<ico.faces.size() ; ++idx){
            Face face = ico.faces[idx];
            Vector3D a = ico.points[face[0]];
            Vector3D b = ico.points[face[1]];
            Vector3D c = ico.points[face[2]];
            Vector3D d = Vector3D::point((a.x + b.x)/2, (a.y + b.y)/2, (a.z + b.z)/2);
            Vector3D e = Vector3D::point((a.x + c.x)/2, (a.y + c.y)/2, (a.z + c.z)/2);
            Vector3D f = Vector3D::point((c.x + b.x)/2, (c.y + b.y)/2, (c.z + b.z)/2);
            a.normalise();
            b.normalise();
            c.normalise();
            d.normalise();
            e.normalise();
            f.normalise();

            sphere.points.push_back(a);
            sphere.points.push_back(b);
            sphere.points.push_back(c);
            sphere.points.push_back(d);
            sphere.points.push_back(e);
            sphere.points.push_back(f);

            sphere.faces.push_back({6*idx+0, 6*idx+3, 6*idx+4}); //ADE
            sphere.faces.push_back({6*idx+1, 6*idx+5, 6*idx+3}); //BFD
            sphere.faces.push_back({6*idx+2, 6*idx+4, 6*idx+5}); //CEF
            sphere.faces.push_back({6*idx+3, 6*idx+5, 6*idx+4}); //DFE
        }
        ico = sphere;
    }
    return sphere;
}

img::EasyImage generate3DLinesImage(const ini::Configuration &conf){
    const unsigned int size = conf["General"]["size"].as_int_or_die();
    const std::vector<double> bgColor = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    const unsigned int nrFigs = conf["General"]["nrFigures"].as_int_or_die();
    const std::vector<double> eyeCoords = conf["General"]["eye"].as_double_tuple_or_die();
    const Vector3D eyePoint = Vector3D::point(eyeCoords[0], eyeCoords[1], eyeCoords[2]);

    Figures3D figures;
    for (unsigned int i=0 ; i < nrFigs ; i++){
        const std::string fig_string = "Figure" + std::to_string(i);
        const std::string fig_type = conf[fig_string]["type"].as_string_or_die();

        const double fig_scale = conf[fig_string]["scale"].as_double_or_die();
        const double fig_rotateX = conf[fig_string]["rotateX"].as_double_or_die() * M_PI/180;
        const double fig_rotateY = conf[fig_string]["rotateY"].as_double_or_die() * M_PI/180;
        const double fig_rotateZ = conf[fig_string]["rotateZ"].as_double_or_die() * M_PI/180;
        const std::vector<double> fig_centerCoords = conf[fig_string]["center"].as_double_tuple_or_die();
        const Vector3D fig_centerPoint = Vector3D::point(fig_centerCoords[0], fig_centerCoords[1], fig_centerCoords[2]);
        const std::vector<double> fig_color = conf[fig_string]["color"].as_double_tuple_or_die();

        Color fig_color_obj = Color(fig_color[0], fig_color[1], fig_color[2]);
        Figure3D newFigure(fig_color_obj);

        if (fig_type == "LineDrawing"){
            const unsigned int nrPoints = conf[fig_string]["nrPoints"].as_int_or_die();
            for (unsigned int p=0 ; p < nrPoints ; p++){
                std::vector<double> coords = conf[fig_string]["point" + std::to_string(p)].as_double_tuple_or_die();
                newFigure.points.push_back(Vector3D::point(coords[0], coords[1], coords[2]));
            }

            const unsigned int nrLines = conf[fig_string]["nrLines"].as_int_or_die();
            for (unsigned int l=0; l < nrLines ; l++){
                std::string lineString = "line" + std::to_string(l);
                std::vector<int> linePoints = conf[fig_string]["line" + std::to_string(l)].as_int_tuple_or_die();
                Face newFace;
                for (unsigned int p : linePoints) newFace.push_back(p);
                newFigure.faces.push_back(newFace);
            }
        }

        else if (fig_type == "Cube") newFigure = createCube(fig_color_obj);
        else if (fig_type == "Tetrahedron") newFigure = createTetrahedron(fig_color_obj);
        else if (fig_type == "Octahedron") newFigure = createOctahedron(fig_color_obj);
        else if (fig_type == "Icosahedron") newFigure = createIcosahedron(fig_color_obj);
        else if (fig_type == "Dodecahedron") newFigure = createDodecahedron(fig_color_obj);
        else if (fig_type == "Cylinder"){
            newFigure = createCylinder(conf[fig_string]["n"].as_int_or_die(),
                                       conf[fig_string]["height"].as_double_or_die(),
                                       fig_color_obj);
        }
        else if (fig_type == "Cone"){
            newFigure = createCone(conf[fig_string]["n"].as_int_or_die(),
                                       conf[fig_string]["height"].as_double_or_die(),
                                       fig_color_obj);
        }
        else if (fig_type == "Sphere"){
            newFigure = createSphere(conf[fig_string]["n"].as_int_or_die(), fig_color_obj);
        }
        else if (fig_type == "Torus"){

        }

        applyTransform(newFigure, scale(fig_scale)
        *rotateX(fig_rotateX)*rotateY(fig_rotateY)*rotateZ(fig_rotateZ)
        *translate(fig_centerPoint)
        *eyePointTrans(eyePoint));

        figures.push_back(newFigure);
    }

    return draw2DLines(doProjection(figures), size, img::Color(lround(bgColor[0] * 255),
                                                               lround(bgColor[1] * 255),
                                                               lround(bgColor[2] * 255)));
}
