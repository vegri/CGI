#ifndef BVT_H
#define BVT_H

#include "vecmath.h"
#include <vector>
#include "Sphere.h"


/// Klassische rekursive Datenstruktur!
/** In den Knoten speichern wir die Info (points_, und ball)
		Die Kinder muessen Pointer sein (c++!)
*/
class BVT
{

	private:

		const std::vector<Vector3d> points_;

		/// mass center of point set
		Vector3d mass_center_;

		/// inetria matrix of point set
		Matrix4d inertia_;
	
		/// smallest enclosing sphere of point set	
		const Sphere ball_;
	
		BVT * left_;
		BVT * right_;

	public:

		/// create new node
		BVT (const std::vector<Vector3d>& points);

		/// get children
		BVT * left() {return left_;};
		BVT * right() {return right_;};

		/// one recursion step to create left and right child
		void split ();

		/// get sphere
		const Sphere& ball() {return ball_;};

		/// anzahl der Punkte
		int nr_of_points () {return points_.size ();};
	
};






#endif //BVT
