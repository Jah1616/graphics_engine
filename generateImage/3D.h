#pragma once
#include "../utils/utils.h"
#include "../ini_configuration.h"
#include "../easy_image.h"
#include "../3D/3DBodies.h"
#include "../3D/3DTransformations.h"
#include "../L-Systems/3D-L-System.h"


img::EasyImage generateWireframeImage(const ini::Configuration &conf){
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
            newFigure = createTorus(conf[fig_string]["r"].as_double_or_die(),
                                    conf[fig_string]["R"].as_double_or_die(),
                                    conf[fig_string]["n"].as_int_or_die(),
                                    conf[fig_string]["m"].as_int_or_die(),
                                    fig_color_obj);
        }
        else if (fig_type == "3DLSystem") L3D_to_Figure3D(conf[fig_string]["inputfile"], newFigure);

        Matrix transformMatrix = scale(fig_scale);
        transformMatrix *= rotateX(fig_rotateX);
        transformMatrix *= rotateY(fig_rotateY);
        transformMatrix *= rotateZ(fig_rotateZ);
        transformMatrix *= translate(fig_centerPoint);
        transformMatrix *= eyePointTrans(eyePoint);

        applyTransform(newFigure, transformMatrix);
        figures.push_back(newFigure);
    }

    return draw2DLines(doProjection(figures), size, img::Color(lround(bgColor[0] * 255),
                                                               lround(bgColor[1] * 255),
                                                               lround(bgColor[2] * 255)));
}
