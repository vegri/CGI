#include "BVT.h"

#include "Sphere.h"

#include <iostream>

#define maximal_points 10
using namespace std;

// Construktor
BVT::BVT (const std::vector<Vector3d>& points) : points_ (points), ball_ (points)
{

  for(unsigned int i=0;i<points_.size();i++)
    mass_center_ += points_[i];
  mass_center_ /= points_.size();

	// Compute inetria matrix
  for(unsigned int i=0;i<points_.size ();i++) {
    const Vector3d& r = points_[i] - mass_center_;
    inertia_(0,0) += r[1]*r[1]+r[2]*r[2];
    inertia_(1,1) += r[0]*r[0]+r[2]*r[2];
    inertia_(2,2) += r[0]*r[0]+r[1]*r[1];
    inertia_(0,1) -= r[0]*r[1];
    inertia_(0,2) -= r[0]*r[2];
    inertia_(1,2) -= r[1]*r[2];
  }
 
	/// Due to the fact that we work with Matrix4d, expand the last row/column by (0,0,0,1)
  inertia_(0,3) = inertia_(3,0) = 0.0;
  inertia_(1,3) = inertia_(3,1) = 0.0;
  inertia_(2,3) = inertia_(3,2) = 0.0;
  inertia_(3,3) = 1.0;
	
	// Kinder! Blaetter haben NULL-Pointer als Kinder!
	left_ = NULL;
	right_ = NULL;
}

/********************************************************
** Construcs an optimal splitting of points_ into two disjoint sets
** links and rechts and increases the BVT-tree by 1. 
** The splitting follows the idea of the lecture:
** (1) Compute the inertia matrix M of points_
** (2) Compute the eigenvalues/eigenvectors of M by M.jacoby() (vecmath!)
** (3) Let v be the most stable eigenvector and c be the mass center.
**     Split points_ by plane p: (x-c)^T*v = 0 
*********************************************************/
void BVT::split ()
{
	std::vector<Vector3d> links;
	std::vector<Vector3d> rechts;

	// Computes the eigenvalues/vectors from the inertia matrix
	Vector4d eigenvalue;
	Matrix4d eigenvector;
	int nrot; /// not important

	/// Dont forget to compute inertia_ first!
	inertia_.jacobi (eigenvalue, eigenvector, nrot);

	/// ADD YOUR CODE HERE!!!


	/// Kinder sind wieder Baeume! Wichtig das NEW!
	left_ = new BVT (links);
	right_ = new BVT (rechts);
}


