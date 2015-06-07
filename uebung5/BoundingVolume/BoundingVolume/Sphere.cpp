#include <algorithm>
#include "Sphere.h"

#include <QtOpenGL>

Sphere::Sphere() {
  center = Vector3d(0.0,0.0,0.0);
  radius = 0.0;
}

// Konstruiere die Kugel fuer einen Punkt
Sphere::Sphere(Vector3d& a) {
  center = a;
  radius = 0.0;
}

// Konstruiere die Umkugel fuer zwei Punkte
Sphere::Sphere(Vector3d& a, Vector3d& b) {
  Vector3d ba = b-a;
  
  center = (a+b)*0.5;
  radius = 0.5*ba.length();
}

// Konstruiere die Umkugel fuer drei Punkte
Sphere::Sphere(Vector3d& a, Vector3d& b, Vector3d& c) {
  Vector3d ba = b-a;
  Vector3d ca = c-a;
  Vector3d baxca = ba%ca;
  Vector3d r;
  Matrix4d T(ba[0],   ba[1],   ba[2],   0.0,
             ca[0],   ca[1],   ca[2],   0.0,
             baxca[0],baxca[1],baxca[2],0.0,
             0.0,     0.0,     0.0,     1.0);
  Matrix4d T1 = Matrix4d::inverse(T);
  
  r[0] = 0.5*ba.lengthSquared();
  r[1] = 0.5*ca.lengthSquared();
  r[2] = 0.0;
  
  center = T1*r;
  radius = center.length();
  center += a;
}

// Konstruiere die Umkugel fuer vier Punkte
Sphere::Sphere(Vector3d& a, Vector3d& b, Vector3d& c, Vector3d& d) {
  Vector3d ba = b-a;
  Vector3d ca = c-a;
  Vector3d da = d-a;
  Vector3d r;
  Matrix4d T(ba[0],ba[1],ba[2],0.0,
             ca[0],ca[1],ca[2],0.0,
             da[0],da[1],da[2],0.0,
             0.0,  0.0,  0.0,  1.0);
  Matrix4d T1 = Matrix4d::inverse(T);
  
  r[0] = 0.5*ba.lengthSquared();
  r[1] = 0.5*ca.lengthSquared();
  r[2] = 0.5*da.lengthSquared();
  center = T1*r;
  radius = center.length();
  center += a;
}

// Berechne den Schwerpunkt der Punktmenge p
Sphere com(const std::vector<Vector3d>& p) {
  Sphere S;
  int i,n = int(p.size());
    
  for(i=0;i<n;i++)
    S.center += p[i];
  S.center /= n;
  
  for(i=0;i<n;i++) {
    Vector3d d = S.center - p[i];
    double r = d.lengthSquared();
    if (r > S.radius) 
      S.radius = r;
  }
  S.radius = sqrt(S.radius);
  return S;
}

void Sphere::draw( Vector3d color) const
	{
		glColor3d( color[0], color[1], color[2]);
		
		GLUquadricObj *quadric;
		quadric = gluNewQuadric();
		
		glPushMatrix();
		glTranslated(center[0],center[1],center[2]);
		gluQuadricDrawStyle(quadric, GLU_LINE);
		gluSphere( quadric , radius,20,20);
		//glutWireSphere(ball_.radius,50,50);
		glPopMatrix();
	}

// Berechne die kleinste einschliessende Kugel fuer die Punktmenge
Sphere::Sphere(const std::vector<Vector3d>& p) {

  std::vector<Vector3d> v(p);
  std::sort(v.begin(),v.end());
  v.erase(std::unique(v.begin(),v.end(),epsilonEquals),v.end());

  Vector3d d;
  int n = int(v.size());

	//random pertutation!
  for(int i=n-1;i>0;i--) {
    int j = (int) floor(i*double(rand())/RAND_MAX);
    d = v[i];//+epsilon;
    v[i] = v[j];//-epsilon; 
    v[j] = d;
  }
   
	/// alter the points to avoid degenerate cases
	Vector3d epsilon(0.00001, -0.00001, 0.0001);
	for(int i=0;i<n;i++)
		if (rand()<0.5*RAND_MAX) v[i] += epsilon;
		else										 v[i] -= epsilon;
 
  Sphere S = Sphere(v[0],v[1]);
    
  for(int i=2;i<n;i++) {
    d = v[i] - S.center;  
    if (d.lengthSquared() > S.radius*S.radius)
      S = ses1(i,v,v[i]);
  }

  center = S.center;
  radius = S.radius;
}

// Berechne die kleinste einschliessende Kugel fuer n Punkte, wobei
// die Punkte q1,q2 und q3 auf dem Rand der gesuchten Kugel liegen
Sphere Sphere::ses3(int n, std::vector<Vector3d>& p,Vector3d& q1,Vector3d& q2,Vector3d& q3) {  
  Sphere S(q1,q2,q3);
  
  for(int i=0;i<n;i++) {
    Vector3d d = p[i] - S.center;
    if (d.lengthSquared() > S.radius*S.radius)
      S = Sphere(q1,q2,q3,p[i]);
  }  
  return S;
}

// Berechne die kleinste einschliessende Kugel fuer n Punkte,
// wobei die Punkte q1 und q2 auf dem Rand der gesuchten Kugel liegen
Sphere Sphere::ses2(int n,std::vector<Vector3d>& p,Vector3d& q1,Vector3d& q2) {
  Sphere S(q1,q2);
  
  for(int i=0;i<n;i++) {
    Vector3d d = p[i] - S.center;  
    if (d.lengthSquared() > S.radius*S.radius)
      S = ses3(i,p,q1,q2,p[i]);
  }
  return S;
}

// Berechne die kleinste einschliessende Kugel fuer n Punkte,
// wobei der Punkt q1 auf dem Rand der gesuchten Kugel liegt
Sphere Sphere::ses1(int n,std::vector<Vector3d>& p,Vector3d& q1) {
  Sphere S(p[0],q1);
  
  for(int i=1;i<n;i++) {
    Vector3d d = p[i] - S.center;
    if (d.lengthSquared() > S.radius*S.radius)    
      S = ses2(i,p,q1,p[i]);
  }
  return S;
}


