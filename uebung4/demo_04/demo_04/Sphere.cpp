#include <algorithm>
#include "Sphere.h"
#include <iostream>

Sphere::Sphere() {
    center = Vector3d(0.0,0.0,0.0);
    radius = 0.0;
}

// Konstruiere die Kugel fuer einen Punkt
Sphere::Sphere(Vector3d& a) {

    center=a;
    radius=0;
}

// Konstruiere die Umkugel fuer zwei Punkte
Sphere::Sphere(Vector3d& a, Vector3d& b) {

    Vector3d c=(a-b)*0.5;
    center=b+c;
    radius=c.length();
    //std::cout << "2 punkte, center= " << center[0] << ", " << center[1] << ", " << center[2] << std::endl;
    //std::cout << "2 punkte, radius= "  << radius << std::endl;
}

// Konstruiere die Umkugel fuer drei Punkte
Sphere::Sphere(Vector3d& a, Vector3d& b, Vector3d& c) {

    Matrix4d triangle = Matrix4d(a[0], a[1], a[2], 0, b[1], b[1], b[2], 0, c[0], c[1], c[2], 0, 0,0,0,1 );
    Matrix4d triangle2;
    triangle2.invert(triangle);
    Vector3d coeffTri=triangle2*Vector3d(1,1,1);
    //std::cout << "3 punkte, coeffTRI= " << coeffTri[0] << ", " << coeffTri[1] << ", " << coeffTri[2] << std::endl;

    Vector3d x=b-a;
    Vector3d y=c-a;
    Matrix4d coeff=Matrix4d(x[0],x[1],x[2],0,y[0],y[1],y[2],0,coeffTri[0],coeffTri[1],coeffTri[2],0,0,0,0,1);
    std::cout << "3 punkte, coeff= " << std::endl;
    coeff.print();
    Matrix4d coeffInv;
    coeffInv.invert(coeff);
    std::cout << "3 punkte, coeff invers= " << std::endl;
    coeffInv.print();
    Vector3d centerCoord= coeffInv*Vector3d(b*b-a*a, c*c-a*a, 2)*0.5;
    center=Vector3d(centerCoord[0],centerCoord[1],centerCoord[2]);
    std::cout << "3 punkte, center= " << center[0] << ", " << center[1] << ", " << center[2] << std::endl;
    Vector3d n=center-a;
    //radius=(center-a).length();
    radius=n.length();
    std::cout << "3 punkte, radius= "  << radius << std::endl;
}

// Konstruiere die Umkugel fuer vier Punkte
Sphere::Sphere(Vector3d& a, Vector3d& b, Vector3d& c, Vector3d& d) {

    Vector3d x=b-a;
    Vector3d y=c-a;
    Vector3d z=d-a;
    Matrix4d coeff=Matrix4d(x[0],x[1],x[2],0,y[0],y[1],y[2],0,z[0],z[1],z[2],0,0,0,0,1);
    //std::cout << "4 punkte, coeff= " << std::endl;
    //coeff.print();
    Matrix4d coeffInv;
    coeffInv.invert(coeff);
    //std::cout << "4 punkte, coeff invers= " << std::endl;
    //coeffInv.print();
    Vector3d centerCoord= coeffInv*Vector3d(b*b-a*a, c*c-a*a, d*d-a*a)*0.5;
    center=Vector3d(centerCoord[0],centerCoord[1],centerCoord[2]);
    //std::cout << "4 punkte, center= " << center[0] << ", " << center[1] << ", " << center[2] << std::endl;
    Vector3d n=center-a;
    radius=n.length();
    //std::cout << "4 punkte, radius= "  << radius << std::endl;
}

bool Sphere::isPointInSphere(const Vector3d p, Sphere sphere){

    float distance=(sphere.center-p).length();
    if(distance<sphere.radius){
        return true;
    }

    return false;
}

// Berechne den Schwerpunkt der Punktmenge p
Sphere Sphere::com(const std::vector<Vector3d>& p) {
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

// Berechne die kleinste einschliessende Kugel fuer die Punktmenge
Sphere::Sphere(const std::vector<Vector3d>& p) {

    std::cout << "laenge p= "  << p.size() << std::endl;
    std::vector<Vector3d> v(p);
    std::sort(v.begin(),v.end());
    v.erase(std::unique(v.begin(),v.end(),epsilonEquals),v.end());

    int n = int(v.size());
    std::cout << "n= "  << n << std::endl;

    //random permutation!
    for(int i=n-1;i>0;i--) {
        int j = (int) floor(i*double(rand())/RAND_MAX);
        Vector3d d = v[i];//+epsilon;
        v[i] = v[j];//-epsilon;
        v[j] = d;
    }

    /// alter the points to avoid degenerate cases
    Vector3d epsilon(0.00001, -0.00001, 0.0001);
    for(int i=0;i<n;i++)
        if (rand()<0.5*RAND_MAX) v[i] += epsilon;
        else										 v[i] -= epsilon;

    ///Hier berechnen wir die Kugel!
    Sphere S = ses0 (n,v);
    
    center = S.center;
    radius = S.radius;
}

// Berechne die kleinste einschliessende Kugel fuer n Punkte,
Sphere Sphere::ses0(int n,std::vector<Vector3d>& p) {

    Sphere S = Sphere(p[0]);

    for(int i=1; i<n; i++){
        if(!isPointInSphere(p[i], S)){
            S=ses1(i, p, p[i]);
        }
    }

    return S;
}

// Berechne die kleinste einschliessende Kugel fuer n Punkte,
// wobei der Punkt q1 auf dem Rand der gesuchten Kugel liegt
Sphere Sphere::ses1(int n,std::vector<Vector3d>& p,Vector3d& q1) {

    //Sphere S(p[0],q1);
    Sphere S(q1);

    for(int j=0; j<n; j++){
        if(!isPointInSphere(p[j], S)){
            S=ses2(j, p, q1, p[j]);
        }
    }

    return S;
}

// Berechne die kleinste einschliessende Kugel fuer n Punkte,
// wobei die Punkte q1 und q2 auf dem Rand der gesuchten Kugel liegen
Sphere Sphere::ses2(int n,std::vector<Vector3d>& p,Vector3d& q1,Vector3d& q2) {

    Sphere S(q1,q2);

    for(int k=0; k<n; k++){
        if(!isPointInSphere(p[k], S)){
            S=ses3(k, p, q1, q2, p[k]);
        }
    }

    return S;
}

// Berechne die kleinste einschliessende Kugel fuer n Punkte, wobei
// die Punkte q1,q2 und q3 auf dem Rand der gesuchten Kugel liegen
Sphere Sphere::ses3(int n, std::vector<Vector3d>& p,Vector3d& q1,Vector3d& q2,Vector3d& q3) {  

    Sphere S(q1,q2,q3);

    for(int l=0; l<n; l++){
        if(!isPointInSphere(p[l], S)){
            S=Sphere(q1, q2, q3, p[l]);
        }
    }

    return S;
}
