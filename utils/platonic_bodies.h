#pragma once
#include <cmath>
#include "utils.h"


void createCube(Figure3D& figure){
    figure.points.clear(); figure.faces.clear();
    figure.points = {Vector3D::point(1,-1,-1), Vector3D::point(-1,1,-1),
                     Vector3D::point(1,1,1), Vector3D::point(-1,-1,1),
                     Vector3D::point(1,1,-1), Vector3D::point(-1,-1,-1),
                     Vector3D::point(1,-1,1), Vector3D::point(-1,1,1)};
    figure.faces = {{0,4,2,6}, {4,1,7,2}, {1,5,3,7}, {5,0,6,3}, {6,2,7,3}, {0,5,1,4}};
}

void createTetrahedron(Figure3D& figure){
    figure.points.clear(); figure.faces.clear();
    figure.points = {Vector3D::point(1,-1,-1), Vector3D::point(-1,1,-1),
                     Vector3D::point(1,1,1), Vector3D::point(-1,-1,1)};
    figure.faces = {{0,1,2}, {1,3,2}, {0,3,1}, {0,2,3}};
}

void createOctahedron(Figure3D& figure){
    figure.points.clear(); figure.faces.clear();
    figure.points = {Vector3D::point(1,0,0), Vector3D::point(0,1,0),
                     Vector3D::point(-1,0,0), Vector3D::point(0,-1,0),
                     Vector3D::point(0,0,-1), Vector3D::point(0,0,1)};
    figure.faces = {{0,1,5}, {1,2,5}, {2,3,5}, {3,0,5}, {1,0,4}, {2,1,4}, {3,2,4}, {0,3,4}};
}

void createIcosahedron(Figure3D& figure){
    static std::vector<Vector3D> points;
    if (points.empty()){
        points.reserve(12);
        points.push_back(Vector3D::point(0, 0, sqrt(5)/2));
        for (unsigned int i=2 ; i<=6 ; ++i)
            points.push_back(Vector3D::point(cos((i-2)*2*M_PI/5), sin((i-2)*2*M_PI/5), 0.5));
        for (unsigned int i=7 ; i<=11 ; ++i)
            points.push_back(Vector3D::point(cos(M_PI/5 + (i-7)*2*M_PI/5),
                                                sin(M_PI/5 + (i-7)*2*M_PI/5),
                                                -0.5));
        points.push_back(Vector3D::point(0, 0, -sqrt(5)/2));
    }

    figure.points = points;
    figure.faces = {{0,1,2}, {0,2,3}, {0,3,4}, {0,4,5}, {0,5,1}, {1,6,2}, {2,6,7}, {2,7,3}, {3,7,8}, {3,8,4}, {4,8,9},
                    {4,9,5}, {5,9,10}, {5,10,1}, {1,10,6}, {11,7,6}, {11,8,7}, {11,9,8}, {11,10,9}, {11,6,10}};
}

void createDodecahedron(Figure3D& figure){
    static std::vector<Vector3D> points;
    if (points.empty()){
        Figure3D ico(figure.color); createIcosahedron(ico);
        points.reserve(20);
        for (const Face &face : ico.faces) points.push_back((ico.points[face[0]]+ico.points[face[1]]+ico.points[face[2]])/3);
    }

    figure.points = points;
    figure.faces = {{0,1,2,3,4}, {0,5,6,7,1}, {1,7,8,9,2}, {2,9,10,11,3}, {3,11,12,13,4}, {4,13,14,5,0},
                    {19,18,17,16,15}, {19,14,13,12,18}, {18,12,11,10,17}, {17,10,9,8,16}, {16,8,7,6,15}, {15,6,5,14,19}};

}

void createCylinder(Figure3D& figure, const unsigned int n, const double h){
    figure.points.clear(); figure.faces.clear();
    figure.points.reserve(2*n);
    figure.faces.reserve(n+2);

    Face base;
    for (unsigned int i=0 ; i<n ; ++i){
        figure.points.push_back(Vector3D::point(cos(2*i*M_PI/n), sin(2*i*M_PI/n), 0));
        base.push_back(n-i-1);
    }

    Face top;
    for (unsigned int i=0 ; i<n ; ++i){
        figure.points.push_back(Vector3D::point(cos(2*i*M_PI/n), sin(2*i*M_PI/n), h));
        top.push_back(n+i);
        figure.faces.push_back({i, (i+1)%n, (i+1)%n + n, i+n});
    }
    figure.faces.push_back(base);
    figure.faces.push_back(top);
}

void createCone(Figure3D& figure, const unsigned int n, const double h){
    figure.points.clear(); figure.faces.clear();
    figure.points.reserve(n+1);
    figure.faces.reserve(n+1);

    Face base;
    for (unsigned int i=0 ; i<n ; ++i){
        figure.points.push_back(Vector3D::point(cos(2*i*M_PI/n), sin(2*i*M_PI/n), 0));
        figure.faces.push_back({i, (i + 1) % n, n});
        base.push_back(n-i-1);
    }
    figure.points.push_back(Vector3D::point(0, 0, h));
    figure.faces.push_back(base);
}

void createSphere(Figure3D& figure, const unsigned int n){
    createIcosahedron(figure);
    figure.points.reserve(5 * pow(4, n+1) - 8);
    figure.faces.reserve(20 * pow(4,n));
    for (unsigned int i=0 ; i<n ; i++){
        unsigned int facesEnd = figure.faces.size();
        for (unsigned int j=0 ; j<facesEnd ; j++){
            Face face = figure.faces[j];
            unsigned int pointsEnd = figure.points.size();
            const Vector3D a = figure.points[face[0]];
            const Vector3D b = figure.points[face[1]];
            const Vector3D c = figure.points[face[2]];

            figure.points.push_back((a+b)/2); // D
            figure.points.push_back((a+c)/2); // E
            figure.points.push_back((b+c)/2); // F

            figure.faces.push_back({face[0],    pointsEnd,      pointsEnd+1});  //ADE
            figure.faces.push_back({face[1],    pointsEnd+2,    pointsEnd});    //BFD
            figure.faces.push_back({face[2],    pointsEnd+1,    pointsEnd+2});  //CEF
            figure.faces[j] = {pointsEnd,  pointsEnd+2,    pointsEnd+1};        //DFE
        }
    }
    for (Vector3D& point : figure.points) point.normalise();
}

void createTorus(Figure3D& figure, const double r, const double R, const unsigned int n, const unsigned int m){
    figure.points.clear(); figure.faces.clear();
    figure.points.reserve(n*m);
    figure.faces.reserve(n*m);
    for (unsigned int i=0 ; i<n ; i++){
        double u = 2*i*M_PI/n;
        for (unsigned int j=0 ; j<m ; j++){
            double v = 2*j*M_PI/m;
            figure.points.push_back(Vector3D::point((R+r*cos(v))*cos(u),
                                                    (R+r*cos(v))*sin(u),
                                                    r*sin(v)));
            figure.faces.push_back({n*i + j, ((i+1)%n)*n + j, ((i+1)%n)*n + (j+1)%m, n*i + (j+1)%m});
        }
    }
}
