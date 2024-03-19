#pragma once
#include <cmath>
#include <vector>
#include <limits>
#include "ini/ini_configuration.h"
#include "easy_image.h"
#include "utils/utils.h"
#include "utils/l-systems.h"
#include "utils/platonic_bodies.h"
#include "utils/3d_transformations.h"


void draw2DLines (img::EasyImage& image, const unsigned int size, const Lines2D &lines,
                  const double bgRed, const double bgGreen, const double bgBlue){
    // set min and max coords
    double xmin = std::numeric_limits<double>::max(); double ymin = xmin;
    double xmax = std::numeric_limits<double>::min(); double ymax = xmax;
    for (const Line2D &line : lines){
        xmin = std::min(xmin, std::min(line.p1.x, line.p2.x));
        xmax = std::max(xmax, std::max(line.p1.x, line.p2.x));
        ymin = std::min(ymin, std::min(line.p1.y, line.p2.y));
        ymax = std::max(ymax, std::max(line.p1.y, line.p2.y));
    }


    const double xrange = xmax - xmin;
    const double yrange = ymax - ymin;
    const double imagex = size * xrange / fmax(xrange, yrange);
    const double imagey = size * yrange / fmax(xrange, yrange);

    image = img::EasyImage(lround(imagex), lround(imagey));
    image.clear(img::Color(lround(bgRed*255), lround(bgGreen*255), lround(bgBlue*255)));

    const double scale = 0.95 * imagex / xrange;
    const double dx = (imagex - scale * (xmin + xmax)) / 2;
    const double dy = (imagey - scale * (ymin + ymax)) / 2;

    for (Line2D line : lines){
        line.p1.x *= scale; line.p1.y *= scale;
        line.p2.x *= scale; line.p2.y *= scale;

        line.p1.x += dx; line.p1.y += dy;
        line.p2.x += dx; line.p2.y += dy;
        image.draw_line(lround(line.p1.x), lround(line.p1.y), lround(line.p2.x), lround(line.p2.y),
                        img::Color(lround(line.color.red*255), lround(line.color.green*255), lround(line.color.blue*255)));
    }
}

void generate2DLSystemImage(img::EasyImage& image, const ini::Configuration &conf){
    const unsigned int size = conf["General"]["size"].as_int_or_die();
    const std::vector<double> bgColor = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    const std::string inputFile = conf["2DLSystem"]["inputfile"].as_string_or_die();
    const std::vector<double> lineColor = conf["2DLSystem"]["color"].as_double_tuple_or_die();

    draw2DLines(image, size,
                LSystem_2D(inputFile, Color(lineColor[0], lineColor[1], lineColor[2])),
                bgColor[0], bgColor[1], bgColor[2]);
}

void generateWireframeImage(img::EasyImage& image, const ini::Configuration &conf){
    const unsigned int size = conf["General"]["size"].as_int_or_die();
    const std::vector<double> bgColor = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    const unsigned int nrFigs = conf["General"]["nrFigures"].as_int_or_die();
    const std::vector<double> eyeCoords = conf["General"]["eye"].as_double_tuple_or_die();
    const auto eyePoint = Vector3D::point(eyeCoords[0], eyeCoords[1], eyeCoords[2]);

    Figures3D figures;
    for (unsigned int i=0 ; i < nrFigs ; i++){
        const std::string fig_string = "Figure" + std::to_string(i);
        const std::string fig_type = conf[fig_string]["type"].as_string_or_die();

        const double fig_scale = conf[fig_string]["scale"].as_double_or_die();
        const double fig_rotateX = toRadian(conf[fig_string]["rotateX"].as_double_or_die());
        const double fig_rotateY = toRadian(conf[fig_string]["rotateY"].as_double_or_die());
        const double fig_rotateZ = toRadian(conf[fig_string]["rotateZ"].as_double_or_die());
        const std::vector<double> fig_centerCoords = conf[fig_string]["center"].as_double_tuple_or_die();
        const Vector3D fig_centerPoint = Vector3D::point(fig_centerCoords[0], fig_centerCoords[1], fig_centerCoords[2]);
        const std::vector<double> fig_color = conf[fig_string]["color"].as_double_tuple_or_die();

        Color fig_color_obj(fig_color[0], fig_color[1], fig_color[2]);
        Figure3D newFigure(fig_color_obj);

        if (fig_type == "LineDrawing"){
            const unsigned int nrPoints = conf[fig_string]["nrPoints"].as_int_or_die();
            for (unsigned int p=0 ; p < nrPoints ; p++){
                const std::vector<double> coords = conf[fig_string]["point" + std::to_string(p)].as_double_tuple_or_die();
                newFigure.points.push_back(Vector3D::point(coords[0], coords[1], coords[2]));
            }

            const unsigned int nrLines = conf[fig_string]["nrLines"].as_int_or_die();
            for (unsigned int l=0; l < nrLines ; l++){
                const std::vector<int> linePoints = conf[fig_string]["line" + std::to_string(l)].as_int_tuple_or_die();
                Face newFace;
                for (unsigned int p : linePoints) newFace.push_back(p);
                newFigure.faces.push_back(newFace);
            }
        }
        else if (fig_type == "Cube") createCube(newFigure);
        else if (fig_type == "Tetrahedron") createTetrahedron(newFigure);
        else if (fig_type == "Octahedron") createOctahedron(newFigure);
        else if (fig_type == "Icosahedron") createIcosahedron(newFigure);
        else if (fig_type == "Dodecahedron") createDodecahedron(newFigure);
        else if (fig_type == "Cylinder") createCylinder(newFigure, conf[fig_string]["n"].as_int_or_die(),
                                                        conf[fig_string]["height"].as_double_or_die());
        else if (fig_type == "Cone") createCone(newFigure, conf[fig_string]["n"].as_int_or_die(),
                                                conf[fig_string]["height"].as_double_or_die());
        else if (fig_type == "Sphere") createSphere(newFigure, conf[fig_string]["n"].as_int_or_die());
        else if (fig_type == "Torus") createTorus(newFigure, conf[fig_string]["r"].as_double_or_die(),
                                                  conf[fig_string]["R"].as_double_or_die(),
                                                  conf[fig_string]["n"].as_int_or_die(),
                                                  conf[fig_string]["m"].as_int_or_die());
        else if (fig_type == "3DLSystem") LSystem_3D(newFigure, conf[fig_string]["inputfile"]);

        Matrix transformMatrix = scale(fig_scale);
        transformMatrix *= rotateX(fig_rotateX);
        transformMatrix *= rotateY(fig_rotateY);
        transformMatrix *= rotateZ(fig_rotateZ);
        transformMatrix *= translate(fig_centerPoint);
        transformMatrix *= eyePointTrans(eyePoint);

        applyTransform(newFigure, transformMatrix);
        figures.push_back(newFigure);
    }

    draw2DLines(image, size, doProjection(figures), bgColor[0], bgColor[1], bgColor[2]);
}