#pragma once
#include "ini/ini_configuration.h"
#include "easy_image.h"
#include "utils/utils.h"
#include "utils/zbuffer.hpp"

void draw2DLines(img::EasyImage& image, const ImgVars& imgVars, const Lines2D& lines,
                  const std::vector<double>& bgColor, ZBUF_MODE zbufMode);
void drawZBufTriangle(ZBuffer& zbuf, img::EasyImage& image,
                       const Vector3D& A, const Vector3D& B, const Vector3D& C,
                       double d, double dx, double dy, const Color& color);
void drawZBufTriangles(img::EasyImage& image, const ImgVars& imgVars, const Figures3D& figures,
                       const std::vector<double>& bgColor);
void generate2DLSystemImage(img::EasyImage& image, const ini::Configuration& conf);
void generateWireframeImage(img::EasyImage& image, const ini::Configuration& conf, ZBUF_MODE zbufMode);
