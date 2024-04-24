#pragma once
#include "ini/ini_configuration.h"
#include "easy_image.h"
#include "utils/utils.h"
#include "utils/zbuffer.hpp"

img::EasyImage generate2DLSystemImage(const ini::Configuration&);
img::EasyImage generateWireframeImage(const ini::Configuration&, ZBUF_MODE = ZBUF_NONE, bool = false);
