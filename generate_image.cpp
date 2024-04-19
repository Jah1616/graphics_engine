#include <cmath>
#include <algorithm>
#include "generate_image.h"

void draw2DLines(img::EasyImage& image, const ImgVars& imgVars, const Lines2D& lines,
                  const std::vector<double>& bgColor, ZBUF_MODE zbufMode){
//    if (zbuffed) Timer timer("draw2DLines (zbuffed)");
//    else Timer timer("draw2DLines");

    const auto& [scale, dx, dy, imagex, imagey] = imgVars;
    image = img::EasyImage(lround(imagex), lround(imagey));
    image.clear(imgColor(bgColor));
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
        else if (zbufMode == ZBUF_LINE) draw_zbuf_line(image, zbuf, lround(p1.x), lround(p1.y), z1,
                                                             lround(p2.x), lround(p2.y), z2,
                                                             imgColor(lineColor));
    }
}

void drawZBufTriangle(ZBuffer& zbuf, img::EasyImage& image,
                       const Vector3D& A, const Vector3D& B, const Vector3D& C,
                       const double d, const double dx, const double dy, const Color& color){
    const Point2D a = doProjection(A, d, dx, dy);
    const Point2D b = doProjection(B, d, dx, dy);
    const Point2D c = doProjection(C, d, dx, dy);

    Lines2D lines{
            {a, b, color, A.z, B.z},
            {b, c, color, B.z, C.z},
            {a, c, color, B.z, C.z}
    };

    const int ymin = lround(std::min({a.y, b.y, c.y}) + 0.5);
    const int ymax = lround(std::max({a.y, b.y, c.y}) - 0.5);

//    const Point2D g((a + b + c)/3);
//    double zg_inv = (1/A.z + 1/B.z + 1/C.z)/3;
//    zg_inv *= 1.0001;

    for (int i=ymin ; i<=ymax ; i++){
        double xl = posInf;
        double xr = negInf;
        double zl, zr;
        for (const auto& [p, q, lineColor, z1, z2] : lines){
            const bool b1 = (i - p.y)*(i - q.y) <= 0;
            const bool b2 = p.y != q.y;
            if (b1 and b2){
                double t = (i - q.y)/(p.y - q.y);
                double xi = t*p.x + (1-t)*q.x;
                if (xi < xl){
                    xl = xi;
                    zl = t*z1 + (1-t)*z2;
                }
                if (xi > xr){
                    xr = xi;
                    zr = t*z1 + (1-t)*z2;
                }
            }
        }
        draw_zbuf_line(image, zbuf, lround(xl + 0.5), i, zl, lround(xr - 0.5), i, zr, imgColor(color));
    }
}

void drawZBufTriangles(img::EasyImage& image, const ImgVars& imgVars, const Figures3D& figures,
                       const std::vector<double>& bgColor){
    const auto& [scale, dx, dy, imagex, imagey] = imgVars;
    image = img::EasyImage(lround(imagex), lround(imagey));
    image.clear(imgColor(bgColor));
    ZBuffer zbuf(lround(imagex), lround(imagey));

    for (auto const& [points, faces, color] : figures){
        for (const Face& face : faces){
            for (const Face& tri : triangulate(face)){
                drawZBufTriangle(zbuf, image, points[tri[0]], points[tri[1]], points[tri[2]],
                                 scale, dx, dy, color);
            }
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
    auto imgVars = getImgVars(lines, size);
    draw2DLines(image, imgVars, lines, bgColor, ZBUF_NONE);
}

void generateWireframeImage(img::EasyImage& image, const ini::Configuration& conf, ZBUF_MODE zbufMode){
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

        Figure3D newFigure(Color{fig_color});

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

    const Lines2D lines = doProjection(figures);
    auto imgVars = getImgVars(lines, size);

    if (zbufMode == ZBUF_TRIANGLE) drawZBufTriangles(image, imgVars, figures, bgColor);
    else draw2DLines(image, imgVars, lines, bgColor, zbufMode);
}
