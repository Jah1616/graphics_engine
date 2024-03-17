#pragma once
#include "../utils/utils.hpp"


Figure3D createCube(const Color& color){
    return {{Vector3D::point(1,-1,-1), Vector3D::point(-1,1,-1),
             Vector3D::point(1,1,1), Vector3D::point(-1,-1,1),
             Vector3D::point(1,1,-1), Vector3D::point(-1,-1,-1),
             Vector3D::point(1,-1,1), Vector3D::point(-1,1,1)},
            {{0,4,2,6}, {4,1,7,2}, {1,5,3,7}, {5,0,6,3}, {6,2,7,3}, {0,5,1,4}},
            color};
}

Figure3D createTetrahedron(const Color& color){
    return {{Vector3D::point(1,-1,-1), Vector3D::point(-1,1,-1),
             Vector3D::point(1,1,1), Vector3D::point(-1,-1,1)},
            {{0,1,2}, {1,3,2}, {0,3,1}, {0,2,3}},
            color};
}

Figure3D createOctahedron(const Color& color){
    return {{Vector3D::point(1,0,0), Vector3D::point(0,1,0),
             Vector3D::point(-1,0,0), Vector3D::point(0,-1,0),
             Vector3D::point(0,0,-1), Vector3D::point(0,0,1)},
            {{0,1,5}, {1,2,5}, {2,3,5}, {3,0,5}, {1,0,4}, {2,1,4}, {3,2,4}, {0,3,4}},
            color};
}

Figure3D createIcosahedron(const Color& color){
    std::vector<Vector3D> points;
    points.push_back(Vector3D::point(0, 0, sqrt(5)/2));
    for (unsigned int i=2 ; i<=6 ; ++i){
        points.push_back(Vector3D::point(cos((i-2)*2*M_PI/5), sin((i-2)*2*M_PI/5), 0.5));
    }
    for (unsigned int i=7 ; i<=11 ; ++i){
        points.push_back(Vector3D::point(cos(M_PI/5 + (i-7)*2*M_PI/5),
                                         sin(M_PI/5 + (i-7)*2*M_PI/5),
                                         -0.5));
    }
    points.push_back(Vector3D::point(0, 0, -sqrt(5)/2));

    return {points,
            {{0,1,2}, {0,2,3}, {0,3,4}, {0,4,5}, {0,5,1}, {1,6,2}, {2,6,7}, {2,7,3}, {3,7,8}, {3,8,4}, {4,8,9},
             {4,9,5}, {5,9,10}, {5,10,1}, {1,10,6}, {11,7,6}, {11,8,7}, {11,9,8}, {11,10,9}, {11,6,10}},
            color};
}

Figure3D createDodecahedron(const Color& color){
    Figure3D ico = createIcosahedron({0, 0, 0});
    std::vector<Vector3D> points;
    for (Face face : ico.faces){
        points.push_back(Vector3D::point((ico.points[face[0]].x + ico.points[face[1]].x + ico.points[face[2]].x)/3,
                                         (ico.points[face[0]].y + ico.points[face[1]].y + ico.points[face[2]].y)/3,
                                         (ico.points[face[0]].z + ico.points[face[1]].z + ico.points[face[2]].z)/3));
    }

    return {points,
            {{0,1,2,3,4}, {0,5,6,7,1}, {1,7,8,9,2}, {2,9,10,11,3}, {3,11,12,13,4}, {4,13,14,5,0},
             {19,18,17,16,15}, {19,14,13,12,18}, {18,12,11,10,17}, {17,10,9,8,16}, {16,8,7,6,15}, {15,6,5,14,19}},
            color};
}

Figure3D createCylinder(const unsigned int n, const double h, const Color& color){
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Face base;
    for (unsigned int i=0 ; i<n ; ++i){
        points.push_back(Vector3D::point(cos(2*i*M_PI/n), sin(2*i*M_PI/n), 0));
        base.push_back(n-i-1);
    }

    Face top;
    for (unsigned int i=0 ; i<n ; ++i){
        points.push_back(Vector3D::point(cos(2*i*M_PI/n), sin(2*i*M_PI/n), h));
        top.push_back(n+i);
        faces.push_back({i, (i+1)%n, (i+1)%n + n, i+n});
    }
    faces.push_back(base);
    faces.push_back(top);

    return {points, faces, color};
}

Figure3D createCone(const unsigned int n, const double h, const Color& color){
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Face base;

    for (unsigned int i=0 ; i<n ; ++i){
        points.push_back(Vector3D::point(cos(2*i*M_PI/n), sin(2*i*M_PI/n), 0));
        faces.push_back({i, (i + 1) % n, n});
        base.push_back(n-i-1);
    }
    points.push_back(Vector3D::point(0, 0, h));
    faces.push_back(base);

    return {points, faces, color};
}

Figure3D createSphere(const unsigned int n, const Color& color){
    Figure3D ico = createIcosahedron(color);
    for (unsigned int i=0 ; i<n ; ++i){
        Figure3D nextIter{color};
        for (unsigned int idx=0 ; idx<ico.faces.size() ; ++idx){
            Face face = ico.faces[idx];
            Vector3D a = ico.points[face[0]];
            Vector3D b = ico.points[face[1]];
            Vector3D c = ico.points[face[2]];

            nextIter.points.push_back(a);
            nextIter.points.push_back(b);
            nextIter.points.push_back(c);
            nextIter.points.push_back(Vector3D::point((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2)); // d
            nextIter.points.push_back(Vector3D::point((a.x + c.x) / 2, (a.y + c.y) / 2, (a.z + c.z) / 2)); // e
            nextIter.points.push_back(Vector3D::point((c.x + b.x) / 2, (c.y + b.y) / 2, (c.z + b.z) / 2)); // f

            nextIter.faces.push_back({6 * idx + 0, 6 * idx + 3, 6 * idx + 4}); //ADE
            nextIter.faces.push_back({6 * idx + 1, 6 * idx + 5, 6 * idx + 3}); //BFD
            nextIter.faces.push_back({6 * idx + 2, 6 * idx + 4, 6 * idx + 5}); //CEF
            nextIter.faces.push_back({6 * idx + 3, 6 * idx + 5, 6 * idx + 4}); //DFE
        }
        ico = nextIter;
    }
    for (Vector3D& point : ico.points) point.normalise();
    return ico;
}

Figure3D createTorus(const double r, const double R, const unsigned int n, const unsigned int m, const Color& color){
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    for (unsigned int i=0 ; i<n ; i++){
        double u = 2*i*M_PI/n;
        for (unsigned int j=0 ; j<m ; j++){
            double v = 2*j*M_PI/m;
            points.push_back(Vector3D::point((R+r*cos(v))*cos(u),
                                             (R+r*cos(v))*sin(u),
                                             r*sin(v)));
            faces.push_back({n*i + j, ((i+1)%n)*n + j, ((i+1)%n)*n + (j+1)%m, n*i + + (j+1)%m});
        }
    }
    return {points, faces, color};
}
