#pragma once
#include "ini/ini_configuration.h"
#include "easy_image.h"
#include "utils/utils.h"
#include "utils/zbuffer.hpp"

void draw2DLines(img::EasyImage& image, const ImgVars& imgVars, const Lines2D& lines,
                 const std::vector<double>& bgColor, ZBUF_MODE zbufMode);
void generate2DLSystemImage(img::EasyImage& image, const ini::Configuration& conf);

void drawZBufTriangles(img::EasyImage& image, const ImgVars& imgVars, const Figures3D& figures,
                       const std::vector<double>& bgColor);
void lineDrawing(Figure3D& figure, const ini::Section& conf);
void platonicBody(Figure3D& figure, const std::string& type, const ini::Section& conf);
void generateWireframeImage(img::EasyImage& image, const ini::Configuration& conf, ZBUF_MODE zbufMode);
