#include <cmath>
#include "generate_image.h"

void draw2DLines(img::EasyImage& image, const ImgVars& imgVars, const Lines2D& lines, ZBUF_MODE zbufMode){
//    Timer timer("draw2DLines");
    const auto& [scale, dx, dy, imagex, imagey] = imgVars;
    auto zbuf = (zbufMode == ZBUF_NONE) ? ZBuffer(0, 0) : ZBuffer(lround(imagex), lround(imagey));

    for (auto [p1, p2, lineColor, z1, z2] : lines) {
        p1 *= scale;
        p2 *= scale;

        p1.x += dx;
        p1.y += dy;
        p2.x += dx;
        p2.y += dy;

        if (zbufMode == ZBUF_NONE) image.draw_line(lround(p1.x), lround(p1.y),
                                                   lround(p2.x), lround(p2.y),
                                                   imgColor(lineColor));
        else if (zbufMode == ZBUF_LINE)
            drawZBufLine(image, zbuf, lround(p1.x), lround(p1.y), z1,
                         lround(p2.x), lround(p2.y), z2,
                         imgColor(lineColor));
    }
}

void drawZBufTriangles(img::EasyImage& image, const ImgVars& imgVars, const Figures3D& figures, const Lights& lights){
    const auto& [scale, dx, dy, imagex, imagey] = imgVars;
    ZBuffer zbuf(lround(imagex), lround(imagey));

    for (const auto& [points, faces, reflection] : figures){
        Color face_color{0, 0, 0};
        for (const auto& [ambient] : lights){
            face_color += (reflection.ambient * ambient);
        }
        for (const Face& face : faces){
            for (const Face& tri : triangulate(face)){
                drawZBufTriangle(image, zbuf, points[tri[0]], points[tri[1]], points[tri[2]],
                                 scale, dx, dy, face_color);
            }
        }
    }
}

Figure3D lineDrawing(const Light& reflection, const ini::Section& conf){
    Figure3D figure(reflection);
    auto nrPoints = conf["nrPoints"].as_int_or_die();
    for (auto p=0 ; p < nrPoints ; p++){
        const std::vector<double> coords = conf["point" + std::to_string(p)].as_double_tuple_or_die();
        figure.points.push_back(Vector3D::point(coords[0], coords[1], coords[2]));
    }

    auto nrLines = conf["nrLines"].as_int_or_die();
    for (auto l=0; l < nrLines ; l++){
        const Face& newFace = conf["line" + std::to_string(l)].as_int_tuple_or_die();
        figure.faces.push_back(newFace);
    }
    return figure;
}

Figure3D platonicBody(const Light& reflection, const std::string& type, const ini::Section& conf){
    if (contains(type, "Cube")) return createCube(reflection);
    else if (contains(type, "Tetrahedron")) return createTetrahedron(reflection);
    else if (contains(type, "Octahedron")) return createOctahedron(reflection);
    else if (contains(type, "Icosahedron")) return createIcosahedron(reflection);
    else if (contains(type, "Dodecahedron")) return createDodecahedron(reflection);
    else if (contains(type, "Cylinder")) return createCylinder(reflection, conf["n"].as_int_or_die(), conf["height"].as_double_or_die());
    else if (contains(type, "Cone")) return createCone(reflection, conf["n"].as_int_or_die(), conf["height"].as_double_or_die());
    else if (contains(type, "Sphere")) return createSphere(reflection, conf["n"].as_int_or_die());
    else if (contains(type, "Torus")) return createTorus(reflection, conf["r"].as_double_or_die(), conf["R"].as_double_or_die(),
                                                         conf["n"].as_int_or_die(), conf["m"].as_int_or_die());
    return Figure3D(reflection);
}

Figure3D generateFractal(const Figure3D& figure, const ini::Section& conf){
    int nr_iterations = conf["nrIterations"].as_int_or_die();
    double fractalScale = conf["fractalScale"].as_double_or_die();
    Matrix scaleMatrix = scale(1/fractalScale);

    Figures3D fractal{figure};
    for (int iteration = 0; iteration < nr_iterations; ++iteration){
        Figures3D temp;
        for (const Figure3D& frac : fractal){
            for (auto i = 0; i < frac.points.size(); ++i){
                Figure3D newFig = frac;
                applyTransform(newFig, scaleMatrix);
                auto p = Vector3D::vector(frac.points[i] - newFig.points[i]);
                applyTransform(newFig, translate(p));
                temp.push_back(newFig);
            }
        }
        fractal = temp;
    }

    return compress(fractal, figure.reflection);
}

img::EasyImage generate2DLSystemImage(const ini::Configuration& conf){
    const int& size = conf["General"]["size"].as_int_or_die();
    const std::vector<double>& bgColor = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    const std::string& inputFile = conf["2DLSystem"]["inputfile"].as_string_or_die();
    const std::vector<double>& lineColor = conf["2DLSystem"]["color"].as_double_tuple_or_die();

    Lines2D lines = LSystem2D(Color(lineColor), inputFile);
    auto imgVars = getImgVars(lines, size);
    img::EasyImage image(lround(imgVars.imagex), lround(imgVars.imagey));
    image.clear(imgColor(bgColor));
    draw2DLines(image, imgVars, lines, ZBUF_NONE);
    return image;
}

img::EasyImage generateWireframeImage(const ini::Configuration& conf, ZBUF_MODE zbufMode){
    const auto& size = conf["General"]["size"].as_int_or_die();
    const std::vector<double>& bgColor = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    const auto& nrFigs = conf["General"]["nrFigures"].as_int_or_die();

    const std::vector<double>& eyeCoords = conf["General"]["eye"].as_double_tuple_or_die();
    const auto eyePoint = Vector3D::point(eyeCoords[0], eyeCoords[1], eyeCoords[2]);

    const auto& nrLights = conf["General"]["nrLights"].as_int_or_default(0);
    Lights lights;
    if (nrLights == 0) lights.push_back(Light{{1,1,1}});
    else {
        for (int i = 0; i < nrLights; ++i) {
            const auto& light_conf = conf["Light" + std::to_string(i)];
            const std::vector<double>& ambient = light_conf["ambient"].as_double_tuple_or_die();

            lights.push_back({Color(ambient)});
        }
    }

    Figures3D figures;
    for (auto i=0 ; i < nrFigs ; i++){
        const ini::Section& fig_conf = conf["Figure" + std::to_string(i)];
        const std::string fig_type = fig_conf["type"].as_string_or_die();

        const double fig_scale = fig_conf["scale"].as_double_or_die();
        const double fig_rotateX = toRadian(fig_conf["rotateX"].as_double_or_die());
        const double fig_rotateY = toRadian(fig_conf["rotateY"].as_double_or_die());
        const double fig_rotateZ = toRadian(fig_conf["rotateZ"].as_double_or_die());
        const std::vector<double> fig_centerCoords = fig_conf["center"].as_double_tuple_or_die();
        const Vector3D fig_translate = Vector3D::vector(fig_centerCoords[0], fig_centerCoords[1], fig_centerCoords[2]);
        const std::vector<double> color = fig_conf["color"].as_double_tuple_or_default({});
        const std::vector<double> ambient = fig_conf["ambientReflection"].as_double_tuple_or_default({});
        Color fig_ambient = ambient.empty() ? Color(color) : Color(ambient);

        Light reflection{fig_ambient};

        Figure3D newFigure(reflection);
        if (fig_type == "LineDrawing") newFigure = lineDrawing(reflection, fig_conf);
        else if (fig_type == "3DLSystem") newFigure = LSystem3D(reflection, fig_conf["inputfile"].as_string_or_die());
        else newFigure = platonicBody(reflection, fig_type, fig_conf);

        if (contains(fig_type, "Fractal")){
//            Figure3D base = newFigure;
//            newFigure = generateFractal(base, fig_conf);
            newFigure = generateFractal(newFigure, fig_conf);
        }

        Matrix transformMatrix = scale(fig_scale)
                * rotateX(fig_rotateX) * rotateY(fig_rotateY) * rotateZ(fig_rotateZ)
                * translate(fig_translate);

        applyTransform(newFigure, transformMatrix);
        figures.push_back(newFigure);
    }

    applyTransform(figures, eyePointTrans(eyePoint));
    const Lines2D lines = doProjection(figures);

    if (lines.empty()) return {};

    auto imgVars = getImgVars(lines, size);

    img::EasyImage image(lround(imgVars.imagex), lround(imgVars.imagey));
    image.clear(imgColor(bgColor));

    if (zbufMode == ZBUF_TRIANGLE) drawZBufTriangles(image, imgVars, figures, lights);
    else draw2DLines(image, imgVars, lines, zbufMode);

    return image;
}
