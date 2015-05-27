#ifndef BB_H
#define BB_H


#include <cmath>
#include <vecmath.h>
#include <ostream>
#include <vector>




class OBB{
public:

    float c11, c22, c33, c12, c13, c23;
    Vector3d axis1, axis2, axis3, center;
    float a1, a2, a3;
    Matrix4d c;
    Matrix4d V;
    int nrot;
    Vector4d d;
    std::vector<Vector3d> p;

    OBB(const std::vector<Vector3d>& vertices);
    bool intersect(const OBB& B);
    void splitOBB(const OBB& A, OBB& A1, OBB& A2);
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
