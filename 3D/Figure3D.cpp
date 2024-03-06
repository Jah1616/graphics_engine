#include "vector3D.h"
#include "../utils.hpp"

#include <vector>
#include <list>

namespace system3D{
    struct Face{
        std::vector<unsigned int> points;
    };

    struct Figure3D{
        std::vector<Vector3D*> points;
        std::vector<Face*> faces;
        Color color;

        void scale(const double factor){
            Matrix m;
            m(0, 0) = factor;
            m(1, 1) = factor;
            m(2, 2) = factor;
            m(3, 3) = 1;

            for (Vector3D* point : points){
                Matrix p;
                m(0, 0) = point->x;
                m(0, 1) = point->y;
                m(0, 2) = point->z;
                m(0, 3) = 0;

                p *= m;
                point->x = p(0, 0);
                point->y = p(0, 1);
                point->z = p(0, 2);
            }
        }
    };

    using Figures3D = std::list<Figure3D>;
};

