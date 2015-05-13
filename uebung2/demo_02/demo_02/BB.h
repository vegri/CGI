#ifndef BB_H
#define BB_H


#include <cmath>
#include <ostream>
#include <vector>




class OBB{
  public:
    Vector3d huellQuader [8];
    double xmin,xmax, ymin, ymax, zmin, zmax;
     OBB(const std::vector<Vector3d>& p);
    void draw();
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
