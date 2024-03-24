#pragma once
#include <cmath>
#include <unordered_map>
#include <vector>
#include "utils.h"


void createCube(Figure3D& figure){
//    Timer timer("Cube");
    figure.points = {Vector3D::point(1,-1,-1), Vector3D::point(-1,1,-1), Vector3D::point(1,1,1),
              Vector3D::point(-1,-1,1), Vector3D::point(1,1,-1), Vector3D::point(-1,-1,-1),
              Vector3D::point(1,-1,1), Vector3D::point(-1,1,1)};
    figure.faces = {{0,4,2,6}, {4,1,7,2}, {1,5,3,7}, {5,0,6,3}, {6,2,7,3}, {0,5,1,4}};
}

void createTetrahedron(Figure3D& figure){
//    Timer timer("Tetrahedron");
    figure.points = {Vector3D::point(1,-1,-1), Vector3D::point(-1,1,-1),
              Vector3D::point(1,1,1), Vector3D::point(-1,-1,1)};
    figure.faces = {{0,1,2}, {1,3,2}, {0,3,1}, {0,2,3}};
}

void createOctahedron(Figure3D& figure){
//    Timer timer("Octahedron");
    auto& [points, faces, color] = figure;
    points = {Vector3D::point(1,0,0), Vector3D::point(0,1,0), Vector3D::point(-1,0,0),
              Vector3D::point(0,-1,0), Vector3D::point(0,0,-1), Vector3D::point(0,0,1)};
    faces = {{0,1,5}, {1,2,5}, {2,3,5}, {3,0,5}, {1,0,4}, {2,1,4}, {3,2,4}, {0,3,4}};
}

void createIcosahedron(Figure3D& figure){
//    Timer timer("Icosahedron");
    static std::vector<Vector3D> mem_points;
    if (mem_points.empty()){
        mem_points.reserve(12);
        mem_points.push_back(Vector3D::point(0, 0, sqrt(5) / 2));
        for (unsigned int i=2 ; i<=6 ; ++i)
            mem_points.push_back(Vector3D::point(cos((i - 2) * 2 * M_PI / 5),
                                                 sin((i - 2) * 2 * M_PI / 5), 0.5));
        for (unsigned int i=7 ; i<=11 ; ++i)
            mem_points.push_back(Vector3D::point(cos(M_PI / 5 + (i - 7) * 2 * M_PI / 5),
                                                 sin(M_PI/5 + (i-7)*2*M_PI/5),
                                                 -0.5));
        mem_points.push_back(Vector3D::point(0, 0, -sqrt(5) / 2));
    }
    figure.points = mem_points;
    figure.faces = {{0,1,2}, {0,2,3}, {0,3,4}, {0,4,5}, {0,5,1}, {1,6,2}, {2,6,7}, {2,7,3}, {3,7,8}, {3,8,4}, {4,8,9},
             {4,9,5}, {5,9,10}, {5,10,1}, {1,10,6}, {11,7,6}, {11,8,7}, {11,9,8}, {11,10,9}, {11,6,10}};
}

void createDodecahedron(Figure3D& figure){
//    Timer timer("Dodecahedron");
    static std::vector<Vector3D> mem_points;
    if (mem_points.empty()){
        Figure3D ico({0,0,0}); createIcosahedron(ico);
        mem_points.reserve(20);
        for (const Face& face : ico.faces) mem_points.push_back((ico.points[face[0]] + ico.points[face[1]] + ico.points[face[2]]) / 3);
    }
    figure.points = mem_points;
    figure.faces = {{0,1,2,3,4}, {0,5,6,7,1}, {1,7,8,9,2}, {2,9,10,11,3}, {3,11,12,13,4}, {4,13,14,5,0},
             {19,18,17,16,15}, {19,14,13,12,18}, {18,12,11,10,17}, {17,10,9,8,16}, {16,8,7,6,15}, {15,6,5,14,19}};
}

void createCylinder(Figure3D& figure, const unsigned int n, const double h){
//    Timer timer("Cylinder");
    static std::unordered_map<unsigned int, std::vector<double>> mem_cos;
    static std::unordered_map<unsigned int, std::vector<double>> mem_sin;
    if (mem_cos.find(n) == mem_cos.end()){
        mem_cos[n].reserve(n);
        mem_sin[n].reserve(n);
        for (unsigned int i=0 ; i<n ; ++i) mem_cos[n].push_back(cos(2*i*M_PI/n));
        for (unsigned int i=0 ; i<n ; ++i) mem_sin[n].push_back(sin(2*i*M_PI/n));
    }
    figure.points.reserve(2*n);
    figure.faces.reserve(n+2);

    Face base(n);
    Face top(n);
    for (unsigned int i=0 ; i<n ; ++i){
        base.push_back(n-i-1);
        top.push_back(i + n);
        figure.points.push_back(Vector3D::point(mem_cos[n][i], mem_sin[n][i], 0));
        figure.faces.push_back({i, (i + 1) % n, (i + 1) % n + n, i + n});
    }
    for (unsigned int i=0 ; i<n ; ++i) figure.points.push_back(Vector3D::point(mem_cos[n][i], mem_sin[n][i], h));
    figure.faces.push_back(base);
    figure.faces.push_back(top);
}

void createCone(Figure3D& figure, const unsigned int n, const double h){
//    Timer timer("Cone");
    static std::unordered_map<unsigned int, std::vector<double>> mem_cos;
    static std::unordered_map<unsigned int, std::vector<double>> mem_sin;
    if (mem_cos.find(n) == mem_cos.end()){
        mem_cos[n].reserve(n);
        mem_sin[n].reserve(n);
        for (unsigned int i=0 ; i<n ; ++i) mem_cos[n].push_back(cos(2*i*M_PI/n));
        for (unsigned int i=0 ; i<n ; ++i) mem_sin[n].push_back(sin(2*i*M_PI/n));
    }

    figure.points.reserve(n+1);
    figure.faces.reserve(n+1);

    Face base;
    for (unsigned int i=0 ; i<n ; ++i){
        figure.points.push_back(Vector3D::point(mem_cos[n][i], mem_sin[n][i], 0));
        figure.faces.push_back({i, (i + 1) % n, n});
        base.push_back(n-i-1);
    }
    figure.points.push_back(Vector3D::point(0, 0, h));
    figure.faces.push_back(base);
}

void createSphere(Figure3D& figure, const unsigned int n){
//    Timer timer("Sphere");
    static int maxN = -1;
    static std::vector<Vector3D> mem_points; // points are always added to the back but shared
    static std::unordered_map<unsigned int, std::vector<Face>> mem_faces;

    if (maxN == -1){
        Figure3D ico({0, 0, 0}); createIcosahedron(ico);
        mem_points = ico.points;
        mem_faces[0] = ico.faces;
        maxN = 0;
    }

    while (maxN < n){
        std::vector<Face>& newFaces = mem_faces[maxN + 1];
        newFaces = mem_faces[maxN];

        unsigned int facesEnd = newFaces.size();
        newFaces.reserve(20 * pow(4,maxN+1));
        mem_points.reserve(5 * pow(4, maxN + 2) - 8);

        for (unsigned int i=0 ; i < facesEnd ; i++){
            Face face = newFaces[i];
            unsigned int pointsEnd = mem_points.size();
            const Vector3D& a = mem_points[face[0]];
            const Vector3D& b = mem_points[face[1]];
            const Vector3D& c = mem_points[face[2]];

            mem_points.push_back((a + b) / 2);
            mem_points.push_back((a + c) / 2);
            mem_points.push_back((b + c) / 2);

            newFaces.push_back({face[0],    pointsEnd,      pointsEnd+1});  //ADE
            newFaces.push_back({face[1],    pointsEnd+2,    pointsEnd});    //BFD
            newFaces.push_back({face[2],    pointsEnd+1,    pointsEnd+2});  //CEF
            newFaces[i] = {pointsEnd, pointsEnd + 2, pointsEnd + 1};        //DFE
        }
        ++maxN;
    }

    figure.points.assign(mem_points.begin(), mem_points.begin() + (5 * pow(4, n + 1) - 8));
    for (Vector3D& p : figure.points) p.normalise();
    figure.faces = mem_faces[n];
}

void createTorus(Figure3D& figure, const double r, const double R, const unsigned int n, const unsigned int m){
//    Timer timer("Torus");

    static std::unordered_map<unsigned int, std::vector<double>> mem_cos;
    static std::unordered_map<unsigned int, std::vector<double>> mem_sin;
    if (mem_cos.find(n) == mem_cos.end()){
        mem_cos[n].reserve(n);
        mem_sin[n].reserve(n);
        for (unsigned int i=0 ; i<n ; ++i) mem_cos[n].push_back(cos(2*i*M_PI/n));
        for (unsigned int i=0 ; i<n ; ++i) mem_sin[n].push_back(sin(2*i*M_PI/n));
    }
    if (mem_cos.find(m) == mem_cos.end()){
        mem_cos[m].reserve(m);
        mem_sin[m].reserve(m);
        for (unsigned int i=0 ; i<m ; ++i) mem_cos[m].push_back(cos(2*i*M_PI/m));
        for (unsigned int i=0 ; i<m ; ++i) mem_sin[m].push_back(sin(2*i*M_PI/m));
    }

    figure.points.clear();
    figure.faces.clear();
    figure.points.reserve(n*m);
    figure.faces.reserve(n*m);
    for (unsigned int i=0 ; i<n ; i++){
        for (unsigned int j=0 ; j<m ; j++){
            figure.points.push_back(Vector3D::point((R+r*mem_cos[m][j])*mem_cos[n][i],
                                             (R+r*mem_cos[m][j])*mem_sin[n][i],
                                             r*mem_sin[m][j]));
            figure.faces.push_back({n*i + j, ((i+1)%n)*n + j, ((i+1)%n)*n + (j+1)%m, n*i + (j+1)%m});
        }
    }
}
