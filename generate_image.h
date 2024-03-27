#pragma once
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include "ini/ini_configuration.h"
#include "easy_image.h"
#include "utils/utils.h"
#include "utils/l-systems.h"
#include "utils/platonic_bodies.h"
#include "utils/3d_transformations.h"


void draw2DLines (img::EasyImage& image, const unsigned int size, const Lines2D& lines,
                  const std::vector<double>& bgColor, const bool zbuffed = false){
//    if (zbuffed) Timer timer("draw2DLines (zbuffed)");
//    else Timer timer("draw2DLines");

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

    image = img::EasyImage(lround(imagex), lround(imagey));
    image.clear(imgColor(bgColor));

    const double scale = 0.95 * imagex / xrange;
    const double dx = (imagex - scale * (xmin + xmax)) / 2;
    const double dy = (imagey - scale * (ymin + ymax)) / 2;

    if (zbuffed){
        ZBuffer zbuf(lround(imagex), lround(imagey));
        for (auto [p1, p2, lineColor, z1, z2]: lines) {
            p1 *= scale;
            p2 *= scale;

            p1.x += dx;
            p1.y += dy;
            p2.x += dx;
            p2.y += dy;

            image.draw_zbuf_line(zbuf, lround(p1.x), lround(p1.y), z1,
                                 lround(p2.x), lround(p2.y), z2,
                                 imgColor(lineColor));
        }
    } else {
        for (auto [p1, p2, lineColor, z1, z2]: lines) {
            p1 *= scale;
            p2 *= scale;

            p1.x += dx;
            p1.y += dy;
            p2.x += dx;
            p2.y += dy;

            image.draw_line(lround(p1.x), lround(p1.y), lround(p2.x), lround(p2.y),
                            imgColor(lineColor));
        }
    }
}

void generate2DLSystemImage(img::EasyImage& image, const ini::Configuration& conf){
    const unsigned int size = conf["General"]["size"].as_int_or_die();
    const std::vector<double> bgColor = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    const std::string inputFile = conf["2DLSystem"]["inputfile"].as_string_or_die();
    const std::vector<double> lineColor = conf["2DLSystem"]["color"].as_double_tuple_or_die();

    Lines2D lines;
    LSystem_2D(lines, inputFile, Color(lineColor));
    draw2DLines(image, size, lines, bgColor);
}

void generateWireframeImage(img::EasyImage& image, const ini::Configuration& conf, const bool zbuffed = false){
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
        const Vector3D fig_translate = Vector3D::vector(fig_centerCoords[0], fig_centerCoords[1], fig_centerCoords[2]);
        const std::vector<double> fig_color = conf[fig_string]["color"].as_double_tuple_or_die();

        Figure3D newFigure(fig_color);

        if (fig_type == "LineDrawing"){
//            Timer timer("Linedrawing");
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

        Matrix transformMatrix = scale(fig_scale)
                * rotateX(fig_rotateX) * rotateY(fig_rotateY) * rotateZ(fig_rotateZ)
                * translate(fig_translate)
                * eyePointTrans(eyePoint);

        applyTransform(newFigure, transformMatrix);
        figures.push_back(newFigure);
    }

    draw2DLines(image, size, doProjection(figures), bgColor, zbuffed);
}
