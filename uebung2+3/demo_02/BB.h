#ifndef BB_H
#define BB_H


#include <cmath>
#include <vecmath.h>
#include <ostream>
#include <vector>




class OBB{
public:

    float c11, c22, c33, c12, c13, c23;
    Vector3d axis1, axis2, axis3;
    float a1, a2, a3;
    Matrix4d c;
    Matrix4d V;
    int nrot;
    Vector4d d;

    OBB(const std::vector<Vector3d>& p);
};

class AABB{
public:
    Vector3d huellQuader [8];
    double xmin,xmax, ymin, ymax, zmin, zmax;
    AABB(const std::vector<Vector3d> p);
    bool intersect (const AABB& B);
    void draw(double rot, double gruen, double blau);
};


#endif // BB_H
