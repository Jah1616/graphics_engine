#include <cmath>
#include <unordered_map>
#include <vector>
#include "utils.h"


static std::unordered_map<int, std::vector<std::pair<double, double>>> mem_trig;

static const std::vector<std::pair<double, double>>& getTrigValues(int x){
    assert(x>0);
    if (mem_trig.find(x) == mem_trig.end()) {
        mem_trig[x].reserve(x);
        for (int i=0; i<x; ++i)
            mem_trig[x].emplace_back(cos(2 * i * M_PI / x), sin(2 * i * M_PI / x));
    }
    return mem_trig[x];
}

Figure3D createCube(const Light& reflection){
//    Timer timer("Cube");
    Figure3D cube(reflection);
    cube.points = {Vector3D::point(1,-1,-1), Vector3D::point(-1,1,-1), Vector3D::point(1,1,1),
              Vector3D::point(-1,-1,1), Vector3D::point(1,1,-1), Vector3D::point(-1,-1,-1),
              Vector3D::point(1,-1,1), Vector3D::point(-1,1,1)};
    cube.faces = {{0,4,2,6}, {4,1,7,2}, {1,5,3,7}, {5,0,6,3}, {6,2,7,3}, {0,5,1,4}};
    return cube;
}

Figure3D createTetrahedron(const Light& reflection){
//    Timer timer("Tetrahedron");
    Figure3D tetra(reflection);
    tetra.points = {Vector3D::point(1, -1, -1), Vector3D::point(-1, 1, -1),
                    Vector3D::point(1,1,1), Vector3D::point(-1,-1,1)};
    tetra.faces = {{0, 1, 2}, {1, 3, 2}, {0, 3, 1}, {0, 2, 3}};
    return tetra;
}

Figure3D createOctahedron(const Light& reflection){
//    Timer timer("Octahedron");
    Figure3D octa(reflection);
    octa.points = {Vector3D::point(1,0,0), Vector3D::point(0,1,0), Vector3D::point(-1,0,0),
              Vector3D::point(0,-1,0), Vector3D::point(0,0,-1), Vector3D::point(0,0,1)};
    octa.faces = {{0,1,5}, {1,2,5}, {2,3,5}, {3,0,5}, {1,0,4}, {2,1,4}, {3,2,4}, {0,3,4}};
    return octa;
}

Figure3D createIcosahedron(const Light& reflection){
//    Timer timer("Icosahedron");
    Figure3D ico(reflection);
    static std::vector<Vector3D> mem_points;
    if (mem_points.empty()){
        mem_points.reserve(12);
        mem_points.push_back(Vector3D::point(0, 0, sqrt(5) / 2));
        for (int i=2 ; i<=6 ; i++)
            mem_points.push_back(Vector3D::point(cos((i - 2) * 2 * M_PI / 5),
                                                 sin((i - 2) * 2 * M_PI / 5), 0.5));
        for (int i=7 ; i<=11 ; i++)
            mem_points.push_back(Vector3D::point(cos(M_PI / 5 + (i - 7) * 2 * M_PI / 5),
                                                 sin(M_PI/5 + (i-7)*2*M_PI/5),
                                                 -0.5));
        mem_points.push_back(Vector3D::point(0, 0, -sqrt(5) / 2));
    }
    ico.points = mem_points;
    ico.faces = {{0,1,2}, {0,2,3}, {0,3,4}, {0,4,5}, {0,5,1}, {1,6,2}, {2,6,7}, {2,7,3}, {3,7,8}, {3,8,4}, {4,8,9},
             {4,9,5}, {5,9,10}, {5,10,1}, {1,10,6}, {11,7,6}, {11,8,7}, {11,9,8}, {11,10,9}, {11,6,10}};
    return ico;
}

Figure3D createDodecahedron(const Light& reflection){
//    Timer timer("Dodecahedron");
    Figure3D dodeca(reflection);
    static std::vector<Vector3D> mem_points;
    if (mem_points.empty()){
        Figure3D ico = createIcosahedron({{0,0,0}});
        mem_points.reserve(20);
        for (const Face& face : ico.faces) mem_points.push_back((ico.points[face[0]] + ico.points[face[1]] + ico.points[face[2]]) / 3);
    }
    dodeca.points = mem_points;
    dodeca.faces = {{0,1,2,3,4}, {0,5,6,7,1}, {1,7,8,9,2}, {2,9,10,11,3}, {3,11,12,13,4}, {4,13,14,5,0},
             {19,18,17,16,15}, {19,14,13,12,18}, {18,12,11,10,17}, {17,10,9,8,16}, {16,8,7,6,15}, {15,6,5,14,19}};
    return dodeca;
}

Figure3D createCylinder(const Light& reflection, int n, double h){
//    Timer timer("Cylinder");
    Figure3D cylinder(reflection);
    const auto& trig = getTrigValues(n);
    cylinder.points.reserve(2*n);
    cylinder.faces.reserve(n+2);

    Face base; base.reserve(n);
    Face top; top.reserve(n);
    for (int i=0 ; i<n ; i++){
        base.push_back(n-i-1);
        top.push_back(i + n);
        const auto& [cos_i, sin_i] = trig[i];
        cylinder.points.push_back(Vector3D::point(cos_i, sin_i, 0));
        cylinder.faces.push_back({i, (i + 1) % n, (i + 1) % n + n, i + n});
    }
    for (int i=0 ; i<n ; i++){
        const auto& [cos_i, sin_i] = trig[i];
        cylinder.points.push_back(Vector3D::point(cos_i, sin_i, h));
    }
    cylinder.faces.push_back(base);
    cylinder.faces.push_back(top);
    return cylinder;
}

Figure3D createCone(const Light& reflection, int n, double h){
//    Timer timer("Cone");
    Figure3D cone(reflection);

    const auto& trig = getTrigValues(n);
    cone.points.reserve(n+1);
    cone.faces.reserve(n+1);

    Face base;
    for (int i=0 ; i<n ; i++){
        const auto& [cos_i, sin_i] = trig[i];
        cone.points.push_back(Vector3D::point(cos_i, sin_i, 0));
        cone.faces.push_back({i, (i + 1) % n, n});
        base.push_back(n-i-1);
    }
    cone.points.push_back(Vector3D::point(0, 0, h));
    cone.faces.push_back(base);
    return cone;
}

Figure3D createSphere(const Light& reflection, int n){
//    Timer timer("Sphere");
    Figure3D sphere(reflection);
    static int maxN = -1;
    static std::vector<Vector3D> mem_points; // points are always added to the back but shared
    static std::unordered_map<int, std::vector<Face>> mem_faces;

    if (maxN == -1){
        Figure3D ico = createIcosahedron({{0, 0, 0}});
        mem_points = ico.points;
        mem_faces[0] = ico.faces;
        maxN = 0;
    }

    while (maxN < n){
        std::vector<Face>& newFaces = mem_faces[maxN + 1];
        newFaces = mem_faces[maxN];

        auto facesEnd = newFaces.size();
        newFaces.reserve(20 * pow(4,maxN+1));
        mem_points.reserve(5 * pow(4, maxN + 2) - 8);

        for (int i=0 ; i < facesEnd ; i++){
            Face face = newFaces[i];
            auto pointsEnd = (int)mem_points.size();
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

    sphere.points.assign(mem_points.begin(), mem_points.begin() + (5 * pow(4, n + 1) - 8));
    for (Vector3D& p : sphere.points) p.normalise();
    sphere.faces = mem_faces[n];
    return sphere;
}

Figure3D createTorus(const Light& reflection, double r, double R, int n, int m){
//    Timer timer("Torus");
    Figure3D torus(reflection);
    const auto& trig_n = getTrigValues(n);
    const auto& trig_m = getTrigValues(n);

    torus.points.clear();
    torus.faces.clear();
    torus.points.reserve(n*m);
    torus.faces.reserve(n*m);

    for (int i = 0; i < n; ++i){
        for (int j = 0; j < m; ++j){
            const auto& [cos_ni, sin_ni] = trig_n[i];
            const auto& [cos_mj, sin_mj] = trig_m[j];

            double x = (R + r * cos_mj) * cos_ni;
            double y = (R + r * cos_mj) * sin_ni;
            double z = r * sin_mj;
            torus.points.push_back(Vector3D::point(x, y, z));
            torus.faces.push_back({n*i + j, ((i+1)%n)*n + j, ((i+1)%n)*n + (j+1)%m, n*i + (j+1)%m});
        }
    }
    return torus;
}
