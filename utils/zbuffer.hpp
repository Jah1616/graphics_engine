#pragma once
#include <vector>
#include <limits>
#include <iostream>
#include <array>


enum ZBUF_MODE{
    ZBUF_NONE,
    ZBUF_LINE,
    ZBUF_TRIANGLE
};

constexpr double posInf = std::numeric_limits<double>::infinity();
constexpr double negInf = -std::numeric_limits<double>::infinity();

struct ZBuffer: public std::vector<std::vector<double>>{
    ZBuffer(const int width, const int height) {
        resize(width, std::vector<double>(height, posInf));
    }
};

static void drawZBufLine(img::EasyImage& image, ZBuffer& zbuf, int x0, int y0, double z0,
                         int x1, int y1, double z1, const img::Color& color){
    if (x0 >= image.get_width() || y0 >= image.get_height() || x1 >= image.get_width() || y1 > image.get_height()){
        std::cerr << "Drawing zbuf line from (" << x0 << "," << y0 << ") to (" << x1 << "," << y1 << ") in image of width "
        << image.get_width() << " and height " << image.get_height();
        exit(-1);
    }

    if (x0 == x1){
//        special case for x0 == x1
        if (y0 > y1){
            std::swap(y0, y1);
            std::swap(z0, z1);
        }
        for (int i = y0; i <= y1; i++){
            double p = (y0 != y1) ? (double)(y1-i)/(y1-y0) : 0;
            double z_inv = p/z0 + (1-p)/z1;
            if (z_inv < zbuf[x0][i]){
                image(x0, i) = color;
                zbuf[x0][i] = z_inv;
            }
        }
    }
    else if (y0 == y1){
//        special case for y0 == y1
        if (x0 > x1){
            std::swap(x0, x1);
            std::swap(z0, z1);
        }
        for (int i = x0; i <= x1; i++){
            double p = (double)(x1-i)/(x1-x0);
            double z_inv = p/z0 + (1-p)/z1;
            if (z_inv < zbuf[i][y0]){
                image(i, y0) = color;
                zbuf[i][y0] = z_inv;
            }
        }
    }
    else {
        if (x0 > x1){
            //flip points if x1>x0: we want x0 to have the lowest value
            std::swap(x0, x1);
            std::swap(y0, y1);
            std::swap(z0, z1);
        }
        double m = (double)(y1-y0) / (x1-x0);
        if (-1.0 <= m && m <= 1.0){
            for (int i = 0; i <= (x1 - x0); i++){
                int x = x0 + i;
                double y = y0 + m * i;
                int y_round = lround(y);

                double p = 1 - (double)i / (x1-x0);
                double z_inv = p/z0 + (1-p)/z1;
                if (z_inv < zbuf[x][y_round]){
                    image(x, y_round) = color;
                    zbuf[x][y_round] = z_inv;
                }
            }
        }
        else if (m > 1.0){
            for (int i = 0; i <= (y1 - y0); i++){
                double x = x0 + (i / m);
                int x_round = lround(x);
                int y = y0 + i;

                double p = 1 - (double)i / (y1-y0);
                double z_inv = p/z0 + (1-p)/z1;
                if (z_inv < zbuf[x_round][y]){
                    image(x_round, y) = color;
                    zbuf[x_round][y] = z_inv;
                }
            }
        }
        else if (m < -1.0){
            for (int i = 0; i <= (y0 - y1); i++){
                double x = x0 - (i / m);
                int x_round = lround(x);
                int y = y0 - i;

                double p = 1 - (double)i / (y0-y1);
                double z_inv = p/z0 + (1-p)/z1;
                if (z_inv < zbuf[x_round][y]){
                    image(x_round, y) = color;
                    zbuf[x_round][y] = z_inv;
                }
            }
        }
    }
}

static void drawZBufTriangle(img::EasyImage& image, ZBuffer& zbuf, const Vector3D& A, const Vector3D& B, const Vector3D& C,
                      double d, double dx, double dy, const Color& color){
    Point2D a = doProjection(A, d, dx, dy);
    Point2D b = doProjection(B, d, dx, dy);
    Point2D c = doProjection(C, d, dx, dy);
    auto imageColor = imgColor(color);
    int ymin = lround(std::min({a.y, b.y, c.y}) + 0.5);
    int ymax = lround(std::max({a.y, b.y, c.y}) - 0.5);
    Lines2D edges{
            {a, b, color, A.z, B.z},
            {a, c, color, A.z, C.z},
            {b, c, color, B.z, C.z}
    };

    Point2D g = (a+b+c)/3;
    double zg_inv = 1/(3*A.z) + 1/(3*B.z) + 1/(3*C.z);
    zg_inv *= 1.0001;
    Vector3D w = Vector3D::cross(B-A, C-A);
    double k = w.x*A.x + w.y*A.y + w.z*A.z;
    double dzdx = w.x/(-d*k);
    double dzdy = w.y/(-d*k);

    for (int y=ymin ; y <= ymax ; y++){
        double xl = posInf;
        double xr = negInf;
        for (const auto& [p, q, lineColor, zp, zq] : edges){
            bool b1 = ((y - p.y) * (y - q.y) <= 0);
            bool b2 = (p.y != q.y);
            if (b1 and b2){
                double xi = q.x + (p.x - q.x) * (y - q.y) / (p.y - q.y);
                xl = std::min(xl, xi);
                xr = std::max(xr, xi);
            }
        }
        int xmin = lround(xl + 0.5);
        int xmax = lround(xr - 0.5);
        for (int x=xmin ; x <= xmax ; x++){
            double zi_inv = zg_inv + (x - g.x)*dzdx + (y - g.y)*dzdy;
            if (zi_inv < zbuf[x][y]){
                zbuf[x][y] = zi_inv;
                image(x, y) = imageColor;
            }
        }
    }
}
