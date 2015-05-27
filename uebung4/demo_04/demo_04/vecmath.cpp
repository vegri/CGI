#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>

#include "vecmath.h"

void Quat4d::set(const Matrix4d& A) {
    *this = A.getRotate();
}

void Quat4d::get(Matrix4d& A) const {
    A.makeRotate(*this);
}

void Quat4d::makeRotate( double angle, double x, double y, double z ) {
    const double epsilon = 0.0000001;

    double length = sqrt( x*x + y*y + z*z );
    if (length < epsilon) {
        // ~zero length axis, so reset rotation to zero.
        *this = Quat4d();
        return;
    }

    double inversenorm  = 1.0/length;
    double coshalfangle = cos( 0.5*angle );
    double sinhalfangle = sin( 0.5*angle );

    q[0] = x * sinhalfangle * inversenorm;
    q[1] = y * sinhalfangle * inversenorm;
    q[2] = z * sinhalfangle * inversenorm;
    q[3] = coshalfangle;
}

void Quat4d::makeRotate( double angle, const Vector3d& v) {
    makeRotate(angle,v[0],v[1],v[2]);
}

void Quat4d::makeRotate ( double angle1, const Vector3d& axis1, 
                        double angle2, const Vector3d& axis2,
                        double angle3, const Vector3d& axis3) {
    Quat4d q1; q1.makeRotate(angle1,axis1);
    Quat4d q2; q2.makeRotate(angle2,axis2);
    Quat4d q3; q3.makeRotate(angle3,axis3);

    *this = q1*q2*q3;
}                        

void Quat4d::makeRotate( const Vector3d& from, const Vector3d& to ) {
    Vector3d sourceVector = from;
    Vector3d targetVector = to;
    
    double fromLen2 = from.lengthSquared();
    double fromLen;
    // normalize only when necessary, epsilon test
    if ((fromLen2 < 1.0-1e-7) || (fromLen2 > 1.0+1e-7)) {
        fromLen = sqrt(fromLen2);
        sourceVector /= fromLen;
    } else fromLen = 1.0;
    
    double toLen2 = to.lengthSquared();
    // normalize only when necessary, epsilon test
    if ((toLen2 < 1.0-1e-7) || (toLen2 > 1.0+1e-7)) {
        double toLen;
        // re-use fromLen for case of mapping 2 vectors of the same length
        if ((toLen2 > fromLen2-1e-7) && (toLen2 < fromLen2+1e-7)) {
            toLen = fromLen;
        } 
        else toLen = sqrt(toLen2);
        targetVector /= toLen;
    }
    
    // Now let's get into the real stuff
    // Use "dot product plus one" as test as it can be re-used later on
    double dotProdPlus1 = 1.0 + sourceVector * targetVector;
    
    // Check for degenerate case of full u-turn. Use epsilon for detection
    if (dotProdPlus1 < 1e-7) {
    
        // Get an orthogonal vector of the given vector
        // in a plane with maximum vector coordinates.
        // Then use it as quaternion axis with pi angle
        // Trick is to realize one value at least is >0.6 for a normalized vector.
        if (fabs(sourceVector.x()) < 0.6) {
            const double norm = sqrt(1.0 - sourceVector.x() * sourceVector.x());
            q[0] = 0.0; 
            q[1] = sourceVector.z() / norm;
            q[2] = -sourceVector.y() / norm;
            q[3] = 0.0;
        } else if (fabs(sourceVector.y()) < 0.6) {
            const double norm = sqrt(1.0 - sourceVector.y() * sourceVector.y());
            q[0] = -sourceVector.z() / norm;
            q[1] = 0.0;
            q[2] = sourceVector.x() / norm;
            q[3] = 0.0;
        } else {
            const double norm = sqrt(1.0 - sourceVector.z() * sourceVector.z());
            q[0] = sourceVector.y() / norm;
            q[1] = -sourceVector.x() / norm;
            q[2] = 0.0;
            q[3] = 0.0;
        }
    } else {
        // Find the shortest angle quaternion that transforms normalized vectors
        // into one other. Formula is still valid when vectors are colinear
        const double s = sqrt(0.5 * dotProdPlus1);
        const Vector3d tmp = sourceVector % targetVector / (2.0*s);
        q[0] = tmp.x();
        q[1] = tmp.y();
        q[2] = tmp.z();
        q[3] = s;
    }
}

void Quat4d::getRotate(double& angle, Vector3d& v) const {
    double x,y,z;
    getRotate(angle,x,y,z);
    v[0] = x;
    v[1] = y;
    v[2] = z;
}

void Quat4d::getRotate( double& angle, double& x, double& y, double& z ) const {
    double sinhalfangle = sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] );

    angle = 2.0 * atan2( sinhalfangle, q[3] );
    if(sinhalfangle) {
        x = q[0] / sinhalfangle;
        y = q[1] / sinhalfangle;
        z = q[2] / sinhalfangle;
    } else {
        x = 0.0;
        y = 0.0;
        z = 1.0;
    }
}

/// Spherical Linear Interpolation
/// As t goes from 0 to 1, the Quat4d object goes from "from" to "to"
/// Reference: Shoemake at SIGGRAPH 89
/// See also
/// http://www.gamasutra.com/features/programming/19980703/quaternions_01.htm
void Quat4d::slerp( double t, const Quat4d& from, const Quat4d& to ) {
    const double epsilon = 0.00001;
    double omega, cosomega, sinomega, scale_from, scale_to ;
    
    Quat4d quatTo(to);
    // this is a dot product
    
    cosomega = from[0]*to[0]+from[1]*to[1]+from[2]*to[2]+from[3]*to[3];
    
    if (cosomega < 0.0) { 
        cosomega = -cosomega; 
        quatTo = -to;
    }

    if( (1.0 - cosomega) > epsilon ) {
        omega= acos(cosomega) ;  // 0 <= omega <= Pi (see man acos)
        sinomega = sin(omega) ;  // this sinomega should always be +ve so
        // could try sinomega=sqrt(1-cosomega*cosomega) to avoid a sin()?
        scale_from = sin((1.0-t)*omega)/sinomega ;
        scale_to = sin(t*omega)/sinomega ;
    } else {
        /* --------------------------------------------------
           The ends of the vectors are very close
           we can use simple linear interpolation - no need
           to worry about the "spherical" interpolation
           -------------------------------------------------- */
        scale_from = 1.0 - t ;
        scale_to = t ;
    }

    *this = (from*scale_from) + (quatTo*scale_to);
}

#define SET_ROW(row, v1, v2, v3, v4 )    \
    M[(row)][0] = (v1); \
    M[(row)][1] = (v2); \
    M[(row)][2] = (v3); \
    M[(row)][3] = (v4);

#define INNER_PRODUCT(a,b,r,c) \
     ((a).M[r][0] * (b).M[0][c]) \
    +((a).M[r][1] * (b).M[1][c]) \
    +((a).M[r][2] * (b).M[2][c]) \
    +((a).M[r][3] * (b).M[3][c])


Matrix4d::Matrix4d( double a00, double a01, double a02, double a03,
                  double a10, double a11, double a12, double a13,
                  double a20, double a21, double a22, double a23,
                  double a30, double a31, double a32, double a33) {
    SET_ROW(0, a00, a01, a02, a03 )
    SET_ROW(1, a10, a11, a12, a13 )
    SET_ROW(2, a20, a21, a22, a23 )
    SET_ROW(3, a30, a31, a32, a33 )
}

void Matrix4d::set( double a00, double a01, double a02, double a03,
                   double a10, double a11, double a12, double a13,
                   double a20, double a21, double a22, double a23,
                   double a30, double a31, double a32, double a33) {
    SET_ROW(0, a00, a01, a02, a03 )
    SET_ROW(1, a10, a11, a12, a13 )
    SET_ROW(2, a20, a21, a22, a23 )
    SET_ROW(3, a30, a31, a32, a33 )
}

Matrix4d Matrix4d::transpose() const {
    Matrix4d A;
    A.M[0][0] = M[0][0]; A.M[0][1] = M[1][0]; A.M[0][2] = M[2][0]; A.M[0][3] = M[3][0];
    A.M[1][0] = M[0][1]; A.M[1][1] = M[1][1]; A.M[1][2] = M[2][1]; A.M[1][3] = M[3][1];
    A.M[2][0] = M[0][2]; A.M[2][1] = M[1][2]; A.M[2][2] = M[2][2]; A.M[2][3] = M[3][2];
    A.M[3][0] = M[0][3]; A.M[3][1] = M[1][3]; A.M[3][2] = M[2][3]; A.M[3][3] = M[3][3];

    return A;
}

#define QX  q.q[0]
#define QY  q.q[1]
#define QZ  q.q[2]
#define QW  q.q[3]

void Matrix4d::setRotate(const Quat4d& q_in) {
    Quat4d q(q_in);
    double length2 = q.lengthSquared();
    if (length2 != 1.0 && length2 != 0.0) {
        q /= sqrt(length2);
    }

    double wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;

    x2 = QX + QX;
    y2 = QY + QY;
    z2 = QZ + QZ;

    xx = QX * x2;
    xy = QX * y2;
    xz = QX * z2;

    yy = QY * y2;
    yz = QY * z2;
    zz = QZ * z2;

    wx = QW * x2;
    wy = QW * y2;
    wz = QW * z2;

    M[0][0] = 1.0 - (yy + zz);
    M[0][1] = xy - wz;
    M[0][2] = xz + wy;
    M[0][3] = 0.0;

    M[1][0] = xy + wz;
    M[1][1] = 1.0 - (xx + zz);
    M[1][2] = yz - wx;
    M[1][3] = 0.0;

    M[2][0] = xz - wy;
    M[2][1] = yz + wx;
    M[2][2] = 1.0 - (xx + yy);
    M[2][3] = 0.0;

    M[3][0] = 0.0;
    M[3][1] = 0.0;
    M[3][2] = 0.0;
    M[3][3] = 1.0;
}

Quat4d Matrix4d::getRotate() const {
    Quat4d q;

    double s;
    double tq[4];
    int    i, j;

    // Use tq to store the largest trace
    tq[0] = 1 + M[0][0]+M[1][1]+M[2][2];
    tq[1] = 1 + M[0][0]-M[1][1]-M[2][2];
    tq[2] = 1 - M[0][0]+M[1][1]-M[2][2];
    tq[3] = 1 - M[0][0]-M[1][1]+M[2][2];

    // Find the maximum (could also use stacked if's later)
    j = 0;
    for(i=1;i<4;i++) j = (tq[i]>tq[j])? i : j;

    // check the diagonal
    if (j==0) {
        /* perform instant calculation */
        QW = tq[0];
        QX = M[1][2]-M[2][1]; 
        QY = M[2][0]-M[0][2]; 
        QZ = M[0][1]-M[1][0]; 
    } else if (j==1) {
        QW = M[1][2]-M[2][1]; 
        QX = tq[1];
        QY = M[0][1]+M[1][0]; 
        QZ = M[2][0]+M[0][2]; 
    } else if (j==2) {
        QW = M[2][0]-M[0][2]; 
        QX = M[0][1]+M[1][0]; 
        QY = tq[2];
        QZ = M[1][2]+M[2][1]; 
    } else { /* if (j==3) */
        QW = M[0][1]-M[1][0]; 
        QX = M[2][0]+M[0][2]; 
        QY = M[1][2]+M[2][1]; 
        QZ = tq[3];
    }

    s = sqrt(0.25/tq[j]);
    QW *= s;
    QX *= -s;
    QY *= -s;
    QZ *= -s;

    return q;
}

int Matrix4d::compare(const Matrix4d& m) const {
    const double* lhs = reinterpret_cast<const double*>(M);
    const double* end_lhs = lhs+16;
    const double* rhs = reinterpret_cast<const double*>(m.M);
    for(;lhs!=end_lhs;++lhs,++rhs) {
        if (*lhs < *rhs) return -1;
        if (*rhs < *lhs) return 1;
    }
    return 0;
}

void Matrix4d::setTrans( double tx, double ty, double tz ) {
    M[0][3] = tx;
    M[1][3] = ty;
    M[2][3] = tz;
}

void Matrix4d::setTrans( const Vector3d& v ) {
    M[0][3] = v[0];
    M[1][3] = v[1];
    M[2][3] = v[2];
}

void Matrix4d::makeIdentity() {
    SET_ROW(0, 1, 0, 0, 0 )
    SET_ROW(1, 0, 1, 0, 0 )
    SET_ROW(2, 0, 0, 1, 0 )
    SET_ROW(3, 0, 0, 0, 1 )
}

void Matrix4d::makeScale( const Vector3d& v ) {
    makeScale(v[0], v[1], v[2] );
}

void Matrix4d::makeScale( double x, double y, double z ) {
    SET_ROW(0, x, 0, 0, 0 )
    SET_ROW(1, 0, y, 0, 0 )
    SET_ROW(2, 0, 0, z, 0 )
    SET_ROW(3, 0, 0, 0, 1 )
}

void Matrix4d::makeTranslate( const Vector3d& v ) {
    makeTranslate( v[0], v[1], v[2] );
}

void Matrix4d::makeTranslate( double x, double y, double z ) {
    SET_ROW(0, 1, 0, 0, x )
    SET_ROW(1, 0, 1, 0, y )
    SET_ROW(2, 0, 0, 1, z )
    SET_ROW(3, 0, 0, 0, 1 )
}

void Matrix4d::makeRotate( const Vector3d& from, const Vector3d& to ) {
    makeIdentity();

    Quat4d q;
    q.makeRotate(from,to);
    setRotate(q);
}

void Matrix4d::makeRotate( double angle, const Vector3d& axis ) {
    makeIdentity();

    Quat4d q;
    q.makeRotate( angle, axis);
    setRotate(q);
}

void Matrix4d::makeRotate( double angle, double x, double y, double z ) {
    makeIdentity();

    Quat4d q;
    q.makeRotate( angle, x, y, z);
    setRotate(q);
}

void Matrix4d::makeRotate( const Quat4d& q ) {
    makeIdentity();

    setRotate(q);
}

void Matrix4d::makeRotate( double angle1, const Vector3d& axis1, 
                         double angle2, const Vector3d& axis2,
                         double angle3, const Vector3d& axis3) {
    makeIdentity();

    Quat4d q;
    q.makeRotate(angle1, axis1, angle2, axis2, angle3, axis3);
    setRotate(q);
}

void Matrix4d::mult( const Matrix4d& lhs, const Matrix4d& rhs ) {   
    if (&lhs==this) {
        postMult(rhs);
        return;
    }
    if (&rhs==this) {
        preMult(lhs);
        return;
    }

// PRECONDITION: We assume neither &lhs nor &rhs == this
// if it did, use preMult or postMult instead
    M[0][0] = INNER_PRODUCT(lhs, rhs, 0, 0);
    M[0][1] = INNER_PRODUCT(lhs, rhs, 0, 1);
    M[0][2] = INNER_PRODUCT(lhs, rhs, 0, 2);
    M[0][3] = INNER_PRODUCT(lhs, rhs, 0, 3);
    M[1][0] = INNER_PRODUCT(lhs, rhs, 1, 0);
    M[1][1] = INNER_PRODUCT(lhs, rhs, 1, 1);
    M[1][2] = INNER_PRODUCT(lhs, rhs, 1, 2);
    M[1][3] = INNER_PRODUCT(lhs, rhs, 1, 3);
    M[2][0] = INNER_PRODUCT(lhs, rhs, 2, 0);
    M[2][1] = INNER_PRODUCT(lhs, rhs, 2, 1);
    M[2][2] = INNER_PRODUCT(lhs, rhs, 2, 2);
    M[2][3] = INNER_PRODUCT(lhs, rhs, 2, 3);
    M[3][0] = INNER_PRODUCT(lhs, rhs, 3, 0);
    M[3][1] = INNER_PRODUCT(lhs, rhs, 3, 1);
    M[3][2] = INNER_PRODUCT(lhs, rhs, 3, 2);
    M[3][3] = INNER_PRODUCT(lhs, rhs, 3, 3);
}

void Matrix4d::preMult( const Matrix4d& A ) {
    // brute force method requiring a copy
    //Matrix4d tmp(A* *this);
    // *this = tmp;

    // more efficient method just use a double[4] for temporary storage.
    double t[4];
    for(int col=0; col<4; ++col) {
        t[0] = INNER_PRODUCT( A, *this, 0, col );
        t[1] = INNER_PRODUCT( A, *this, 1, col );
        t[2] = INNER_PRODUCT( A, *this, 2, col );
        t[3] = INNER_PRODUCT( A, *this, 3, col );
        M[0][col] = t[0];
        M[1][col] = t[1];
        M[2][col] = t[2];
        M[3][col] = t[3];
    }
}

void Matrix4d::postMult( const Matrix4d& A ) {
    // brute force method requiring a copy
    //Matrix4d tmp(*this * A);
    // *this = tmp;

    // more efficient method just use a double[4] for temporary storage.
    double t[4];
    for(int row=0; row<4; ++row) {
        t[0] = INNER_PRODUCT( *this, A, row, 0 );
        t[1] = INNER_PRODUCT( *this, A, row, 1 );
        t[2] = INNER_PRODUCT( *this, A, row, 2 );
        t[3] = INNER_PRODUCT( *this, A, row, 3 );
        SET_ROW(row, t[0], t[1], t[2], t[3] )
    }
}

#undef INNER_PRODUCT

// orthoNormalize the 3x3 rotation matrix
void Matrix4d::orthoNormalize(const Matrix4d& A) {
    double x_colMag = (A.M[0][0] * A.M[0][0]) + (A.M[1][0] * A.M[1][0]) + (A.M[2][0] * A.M[2][0]);
    double y_colMag = (A.M[0][1] * A.M[0][1]) + (A.M[1][1] * A.M[1][1]) + (A.M[2][1] * A.M[2][1]);
    double z_colMag = (A.M[0][2] * A.M[0][2]) + (A.M[1][2] * A.M[1][2]) + (A.M[2][2] * A.M[2][2]);
    
    if(!equivalent((double)x_colMag, 1.0) && !equivalent((double)x_colMag, 0.0))
    {
      x_colMag = sqrt(x_colMag);
      M[0][0] = A.M[0][0] / x_colMag;
      M[1][0] = A.M[1][0] / x_colMag;
      M[2][0] = A.M[2][0] / x_colMag;
    } else {
      M[0][0] = A.M[0][0];
      M[1][0] = A.M[1][0];
      M[2][0] = A.M[2][0];
    }

    if(!equivalent((double)y_colMag, 1.0) && !equivalent((double)y_colMag, 0.0))
    {
      y_colMag = sqrt(y_colMag);
      M[0][1] = A.M[0][1] / y_colMag;
      M[1][1] = A.M[1][1] / y_colMag;
      M[2][1] = A.M[2][1] / y_colMag;
    } else {
      M[0][1] = A.M[0][1];
      M[1][1] = A.M[1][1];
      M[2][1] = A.M[2][1];
    }

    if(!equivalent((double)z_colMag, 1.0) && !equivalent((double)z_colMag, 0.0))
    {
      z_colMag = sqrt(z_colMag);
      M[0][2] = A.M[0][2] / z_colMag;
      M[1][2] = A.M[1][2] / z_colMag;
      M[2][2] = A.M[2][2] / z_colMag;
    } else {
      M[0][2] = A.M[0][2];
      M[1][2] = A.M[1][2];
      M[2][2] = A.M[2][2];
    }

    M[3][0] = A.M[3][0];
    M[3][1] = A.M[3][1];
    M[3][2] = A.M[3][2];

    M[0][3] = A.M[0][3];
    M[1][3] = A.M[1][3];
    M[2][3] = A.M[2][3];
    M[3][3] = A.M[3][3];
}

/******************************************
  Matrix inversion technique:
Given a matrix mat, we want to invert it.
mat = [ r00 r01 r02 a
        r10 r11 r12 b
        r20 r21 r22 c
        tx  ty  tz  d ]
We note that this matrix can be split into three matrices.
mat = rot * trans * corr, where rot is rotation part, trans is translation part, and corr is the correction due to perspective (if any).
rot = [ r00 r01 r02 0
        r10 r11 r12 0
        r20 r21 r22 0
        0   0   0   1 ]
trans = [ 1  0  0  0
          0  1  0  0
          0  0  1  0
          tx ty tz 1 ]
corr = [ 1 0 0 px
         0 1 0 py
         0 0 1 pz
         0 0 0 s ]
where the elements of corr are obtained from linear combinations of the elements of rot, trans, and mat.
So the inverse is mat' = (trans * corr)' * rot', where rot' must be computed the traditional way, which is easy since it is only a 3x3 matrix.
This problem is simplified if [px py pz s] = [0 0 0 1], which will happen if mat was composed only of rotations, scales, and translations (which is common).  In this case, we can ignore corr entirely which saves on a lot of computations.
******************************************/

bool Matrix4d::invert_4x3( const Matrix4d& mat ) {
    if (&mat==this)
    {
       Matrix4d tm(mat);
       return invert_4x3(tm);
    }

    register double r00, r01, r02,
                        r10, r11, r12,
                        r20, r21, r22;
      // Copy rotation components directly into registers for speed
    r00 = mat.M[0][0]; r01 = mat.M[0][1]; r02 = mat.M[0][2];
    r10 = mat.M[1][0]; r11 = mat.M[1][1]; r12 = mat.M[1][2];
    r20 = mat.M[2][0]; r21 = mat.M[2][1]; r22 = mat.M[2][2];

        // Partially compute inverse of rot
    M[0][0] = r11*r22 - r12*r21;
    M[0][1] = r02*r21 - r01*r22;
    M[0][2] = r01*r12 - r02*r11;

      // Compute determinant of rot from 3 elements just computed
    register double one_over_det = 1.0/(r00*M[0][0] + r10*M[0][1] + r20*M[0][2]);
    r00 *= one_over_det; r10 *= one_over_det; r20 *= one_over_det;  // Saves on later computations

      // Finish computing inverse of rot
    M[0][0] *= one_over_det;
    M[0][1] *= one_over_det;
    M[0][2] *= one_over_det;
    M[0][3] = 0.0;
    M[1][0] = r12*r20 - r10*r22; // Have already been divided by det
    M[1][1] = r00*r22 - r02*r20; // same
    M[1][2] = r02*r10 - r00*r12; // same
    M[1][3] = 0.0;
    M[2][0] = r10*r21 - r11*r20; // Have already been divided by det
    M[2][1] = r01*r20 - r00*r21; // same
    M[2][2] = r00*r11 - r01*r10; // same
    M[2][3] = 0.0;
    M[3][3] = 1.0;

// We no longer need the rxx or det variables anymore, so we can reuse them for whatever we want.  But we will still rename them for the sake of clarity.

#define d r22
    d  = mat.M[3][3];

    if ( (d-1.0)*(d-1.0) > 1.0e-6 ) {
        // Involves perspective, so we must
        // compute the full inverse
    
        Matrix4d TPinv;
        M[3][0] = M[3][1] = M[3][2] = 0.0;

#define px r00
#define py r01
#define pz r02
#define one_over_s  one_over_det
#define a  r10
#define b  r11
#define c  r12

        a  = mat.M[0][3]; b  = mat.M[1][3]; c  = mat.M[2][3];
        px = M[0][0]*a + M[0][1]*b + M[0][2]*c;
        py = M[1][0]*a + M[1][1]*b + M[1][2]*c;
        pz = M[2][0]*a + M[2][1]*b + M[2][2]*c;

#undef a
#undef b
#undef c
#define tx r10
#define ty r11
#define tz r12

        tx = mat.M[3][0]; ty = mat.M[3][1]; tz = mat.M[3][2];
        one_over_s  = 1.0/(d - (tx*px + ty*py + tz*pz));

        tx *= one_over_s; ty *= one_over_s; tz *= one_over_s;  // Reduces number of calculations later on

        // Compute inverse of trans*corr
        TPinv.M[0][0] = tx*px + 1.0;
        TPinv.M[0][1] = ty*px;
        TPinv.M[0][2] = tz*px;
        TPinv.M[0][3] = -px * one_over_s;
        TPinv.M[1][0] = tx*py;
        TPinv.M[1][1] = ty*py + 1.0;
        TPinv.M[1][2] = tz*py;
        TPinv.M[1][3] = -py * one_over_s;
        TPinv.M[2][0] = tx*pz;
        TPinv.M[2][1] = ty*pz;
        TPinv.M[2][2] = tz*pz + 1.0;
        TPinv.M[2][3] = -pz * one_over_s;
        TPinv.M[3][0] = -tx;
        TPinv.M[3][1] = -ty;
        TPinv.M[3][2] = -tz;
        TPinv.M[3][3] = one_over_s;

        preMult(TPinv); // Finish computing full inverse of mat

#undef px
#undef py
#undef pz
#undef one_over_s
#undef d
    }
    else // Rightmost column is [0; 0; 0; 1] so it can be ignored
    {
        tx = mat.M[3][0]; ty = mat.M[3][1]; tz = mat.M[3][2];

        // Compute translation components of mat'
        M[3][0] = -(tx*M[0][0] + ty*M[1][0] + tz*M[2][0]);
        M[3][1] = -(tx*M[0][1] + ty*M[1][1] + tz*M[2][1]);
        M[3][2] = -(tx*M[0][2] + ty*M[1][2] + tz*M[2][2]);

#undef tx
#undef ty
#undef tz
    }

    return true;
}

template <class T>
inline T SGL_ABS(T a) {
   return (a >= 0 ? a : -a);
}

#ifndef SGL_SWAP
#define SGL_SWAP(a,b,temp) ((temp)=(a),(a)=(b),(b)=(temp))
#endif

bool Matrix4d::invert_4x4( const Matrix4d& mat ) {
    if (&mat==this) {
       Matrix4d tm(mat);
       return invert_4x4(tm);
    }

    unsigned int indxc[4], indxr[4], ipiv[4];
    unsigned int i,j,k,l,ll;
    unsigned int icol = 0;
    unsigned int irow = 0;
    double temp, pivinv, dum, big;

    // copy in place this may be unnecessary
    *this = mat;

    for (j=0; j<4; j++) ipiv[j]=0;

    for(i=0;i<4;i++) {
       big=0.0;
       for (j=0; j<4; j++)
          if (ipiv[j] != 1)
             for (k=0; k<4; k++) {
                if (ipiv[k] == 0) {
                   if (SGL_ABS(operator()(j,k)) >= big) {
                      big = SGL_ABS(operator()(j,k));
                      irow=j;
                      icol=k;
                   }
                } else if (ipiv[k] > 1)
                   return false;
             }
       ++(ipiv[icol]);
       if (irow != icol)
          for (l=0; l<4; l++) SGL_SWAP(operator()(irow,l),
                                       operator()(icol,l),
                                       temp);

       indxr[i]=irow;
       indxc[i]=icol;
       if (operator()(icol,icol) == 0)
          return false;

       pivinv = 1.0/operator()(icol,icol);
       operator()(icol,icol) = 1;
       for (l=0; l<4; l++) operator()(icol,l) *= pivinv;
       for (ll=0; ll<4; ll++)
          if (ll != icol) {
             dum=operator()(ll,icol);
             operator()(ll,icol) = 0;
             for (l=0; l<4; l++) operator()(ll,l) -= operator()(icol,l)*dum;
          }
    }
    for (int lx=4; lx>0; --lx) {
       if (indxr[lx-1] != indxc[lx-1])
          for (k=0; k<4; k++) SGL_SWAP(operator()(k,indxr[lx-1]),
                                       operator()(k,indxc[lx-1]),temp);
    }

    return true;
}

void Matrix4d::makeOrtho(double left, double right,
                       double bottom, double top,
                       double zNear, double zFar) {
    // note transpose of Matrix4d wr.t OpenGL documentation, since the OSG use post multiplication rather than pre.
    double tx = -(right+left)/(right-left);
    double ty = -(top+bottom)/(top-bottom);
    double tz = -(zFar+zNear)/(zFar-zNear);
    SET_ROW(0, 2.0/(right-left),               0.0,               0.0, 0.0 )
    SET_ROW(1,              0.0,  2.0/(top-bottom),               0.0, 0.0 )
    SET_ROW(2,              0.0,               0.0,  -2.0/(zFar-zNear), 0.0 )
    SET_ROW(3,               tx,                ty,                 tz, 1.0 )
}

bool Matrix4d::getOrtho(double& left, double& right,
                      double& bottom, double& top,
                      double& zNear, double& zFar) const {
    if (M[0][3]!=0.0 || M[1][3]!=0.0 || M[2][3]!=0.0 || M[3][3]!=1.0) return false;

    zNear = (M[3][2]+1.0) / M[2][2];
    zFar = (M[3][2]-1.0) / M[2][2];
    
    left = -(1.0+M[3][0]) / M[0][0];
    right = (1.0-M[3][0]) / M[0][0];

    bottom = -(1.0+M[3][1]) / M[1][1];
    top = (1.0-M[3][1]) / M[1][1];
    
    return true;
}            


void Matrix4d::makeFrustum(double left, double right,
                         double bottom, double top,
                         double zNear, double zFar) {
    // note transpose of Matrix4d wr.t OpenGL documentation, since the OSG use post multiplication rather than pre.
    double A = (right+left)/(right-left);
    double B = (top+bottom)/(top-bottom);
    double C = -(zFar+zNear)/(zFar-zNear);
    double D = -2.0*zFar*zNear/(zFar-zNear);
    SET_ROW(0, 2.0*zNear/(right-left),                    0.0, 0.0,  0.0 )
    SET_ROW(1,                    0.0, 2.0*zNear/(top-bottom), 0.0,  0.0 )
    SET_ROW(2,                      A,                      B,   C, -1.0 )
    SET_ROW(3,                    0.0,                    0.0,   D,  0.0 )
}

bool Matrix4d::getFrustum(double& left, double& right,
    double& bottom, double& top, double& zNear, double& zFar) const {

    if (M[0][3]!=0.0 || M[1][3]!=0.0 || M[2][3]!=-1.0 
        || M[3][3]!=0.0) return false;

    zNear = M[3][2] / (M[2][2]-1.0);
    zFar = M[3][2] / (1.0+M[2][2]);
    
    left = zNear * (M[2][0]-1.0) / M[0][0];
    right = zNear * (1.0+M[2][0]) / M[0][0];

    top = zNear * (1.0+M[2][1]) / M[1][1];
    bottom = zNear * (M[2][1]-1.0) / M[1][1];
    
    return true;
}                 

void Matrix4d::makePerspective(double fovy,double aspectRatio,
    double zNear, double zFar) {
    // calculate the appropriate left, right etc.
    double tan_fovy = tan(M_PI/180.0*(fovy*0.5));
    double right  =  tan_fovy * aspectRatio * zNear;
    double left   = -right;
    double top    =  tan_fovy * zNear;
    double bottom =  -top;
    makeFrustum(left,right,bottom,top,zNear,zFar);
}

bool Matrix4d::getPerspective(double& fovy,double& aspectRatio,
    double& zNear, double& zFar) const {
    double right  =  0.0;
    double left   =  0.0;
    double top    =  0.0;
    double bottom =  0.0;
    if (getFrustum(left,right,bottom,top,zNear,zFar)) {
        fovy = 180.0/M_PI*(atan(top/zNear)-atan(bottom/zNear));
        aspectRatio = (right-left)/(top-bottom);
        return true;
    }
    return false;
}

void Matrix4d::makeLookAt(const Vector3d& eye,const 
    Vector3d& center,const Vector3d& up) {
    Vector3d f(center-eye);
    f.normalize();
    Vector3d s(f%up);
    s.normalize();
    Vector3d u(s%f);
    u.normalize();

    set( s[0], u[0],-f[0], 0.0,
         s[1], u[1],-f[1], 0.0,
         s[2], u[2],-f[2], 0.0,
         0.0,   0.0,  0.0, 1.0);

    preMult(Matrix4d::translate(-eye));
}

void Matrix4d::getLookAt(Vector3d& eye,Vector3d& center,
    Vector3d& up,double lookDistance) const {
    Matrix4d inv;
    inv.invert(*this);
    eye = Vector3d(0.0,0.0,0.0)*inv;
    up = transform3x3(*this,Vector3d(0.0,1.0,0.0));
    center = transform3x3(*this,Vector3d(0.0,0.0,-1));
    center.normalize();
    center = eye + center*lookDistance;
}

void Matrix4d::jacobiRot(double s, double tau, int i, int j, int k, int l) {
  double g,h;

  g = M[i][j];
  h = M[k][l];
  M[i][j] = g-s*(h+g*tau);
  M[k][l] = h+s*(g-h*tau);
}

int Matrix4d::jacobi(Vector4d& d, Matrix4d& V, int& nrot) {
  Matrix4d A = *this;
  Vector4d b,z;
  int i,j,ip,iq;
  double tresh,theta,tau,t,sm,s,h,g,c;
  const int n = 4;

  for(ip=0;ip<n;ip++) {
    for(iq=0;iq<n;iq++) 
       V(ip,iq) = 0.0;
    V(ip,ip) = 1.0;
  }

  for(ip=0;ip<n;ip++) {
    b[ip] = d[ip] = A(ip,ip);
    z[ip] = 0.0;
  }

  nrot = 0;
  for(i=1;i<=50;i++) {
    sm = 0.0;
    for(ip=0;ip<n-1;ip++) {
      for(iq=ip+1;iq<n;iq++)
        sm += fabs(A(ip,iq));
    }
    if (sm == 0.0)
      return 1;
    if (i < 4)
      tresh = 0.2*sm/(n*n);
    else
      tresh = 0.0;
    for(ip=0;ip<n-1;ip++) {
      for(iq=ip+1;iq<n;iq++) {
        g = 100.0*fabs(A(ip,iq));
        if (i > 4 && (fabs(d[ip])+g) == fabs(d[ip])
          && (fabs(d[iq])+g) == fabs(d[iq]))
            A(ip,iq)=0.0;
        else if (fabs(A(ip,iq)) > tresh) {
          h = d[iq]-d[ip];
          if ((fabs(h)+g) == fabs(h))
            t = (A(ip,iq))/h;
          else {
            theta = 0.5*h/(A(ip,iq));
            t = 1.0/(fabs(theta)+sqrt(1.0+theta*theta));
            if (theta < 0.0) t = -t;
          }
          c = 1.0/sqrt(1+t*t);
          s = t*c;
          tau = s/(1.0+c);
          h = t*A(ip,iq);
          z[ip] -= h;
          z[iq] += h;
          d[ip] -= h;
          d[iq] += h;
          A(ip,iq) = 0.0;
          for (j=0;j<ip;j++)
            A.jacobiRot(s,tau,j,ip,j,iq);
          for (j=ip+1;j<iq;j++)
            A.jacobiRot(s,tau,ip,j,j,iq);
          for (j=iq+1;j<n;j++)
            A.jacobiRot(s,tau,ip,j,iq,j);
          for (j=0;j<n;j++)
            V.jacobiRot(s,tau,j,ip,j,iq);
          ++nrot;
        }
      }
    }
    for(ip=0;ip<n;ip++) {
      b[ip] += z[ip];
      d[ip] = b[ip];
      z[ip] = 0.0;
    }
  }
  return 0;
}
#undef SET_ROW

void Vectord::print() {
  for(int i=0;i<n;i++)
    std::cout << v[i] << " ";
  std::cout << std::endl;
}

void Matrix4d::print() {
    for(int i=0;i<4;i++) {
        for(int j=0;j<4;j++)
            std::cout << M[i][j] << " ";
        std::cout << std::endl;
    }
}

void Matrixd::print() {
  for(int i=0;i<n;i++) {
    for(int j=0;j<m;j++)
      std::cout << M[i][j] << " ";
    std::cout << std::endl;
  }
}

double Matrixd::pythag(const double a, const double b) {
  double absa,absb;

  absa = fabs(a);
  absb = fabs(b);
  if (absa > absb) return absa*sqrt(1.0+sqr(absb/absa));
  else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+sqr(absa/absb)));
}

bool Matrixd::svdcmp(Vectord &w, Matrixd &V) {
  bool flag;
  int i,its,j,jj,k,l,nm;
  double anorm,c,f,g,h,s,scale,x,y,z;
  Matrixd& A = *this;

  int m = A.nrows();
  int n = A.ncols();
  Vectord rv1(n);
  g=scale=anorm=0.0;
  for(i=0;i<n;i++) {
    l=i+2;
    rv1[i]=scale*g;
    g=s=scale=0.0;
    if (i < m) {
      for(k=i;k<m;k++) scale += fabs(A(k,i));
      if (scale != 0.0) {
        for(k=i;k<m;k++) {
          A(k,i) /= scale;
          s += A(k,i)*A(k,i);
        }
        f=A(i,i);
        g = -sign(sqrt(s),f);
        h=f*g-s;
        A(i,i)=f-g;
        for(j=l-1;j<n;j++) {
          for(s=0.0,k=i;k<m;k++) s += A(k,i)*A(k,j);
          f=s/h;
          for(k=i;k<m;k++) A(k,j) += f*A(k,i);
        }
        for(k=i;k<m;k++) A(k,i) *= scale;
      }
    }
    w[i]=scale *g;
    g=s=scale=0.0;
    if (i+1 <= m && i != n) {
      for(k=l-1;k<n;k++) scale += fabs(A(i,k));
      if (scale != 0.0) {
        for(k=l-1;k<n;k++) {
          A(i,k) /= scale;
          s += A(i,k)*A(i,k);
        }
        f=A(i,l-1);
        g = -sign(sqrt(s),f);
        h=f*g-s;
        A(i,l-1)=f-g;
        for(k=l-1;k<n;k++) rv1[k]=A(i,k)/h;
        for(j=l-1;j<m;j++) {
          for(s=0.0,k=l-1;k<n;k++) s += A(j,k)*A(i,k);
          for(k=l-1;k<n;k++) A(j,k) += s*rv1[k];
        }
        for(k=l-1;k<n;k++) A(i,k) *= scale;
      }
    }
    anorm=max(anorm,(fabs(w[i])+fabs(rv1[i])));
  }
  for(i=n-1;i>=0;i--) {
    if (i < n-1) {
      if (g != 0.0) {
        for(j=l;j<n;j++)
          V(j,i)=(A(i,j)/A(i,l))/g;
        for(j=l;j<n;j++) {
          for(s=0.0,k=l;k<n;k++) s += A(i,k)*V(k,j);
          for(k=l;k<n;k++) V(k,j) += s*V(k,i);
        }
      }
      for(j=l;j<n;j++) V(i,j)=V(j,i)=0.0;
    }
    V(i,i)=1.0;
    g=rv1[i];
    l=i;
  }
  for(i=std::min(m,n)-1;i>=0;i--) {
    l=i+1;
    g=w[i];
    for(j=l;j<n;j++) A(i,j)=0.0;
    if (g != 0.0) {
      g=1.0/g;
      for(j=l;j<n;j++) {
        for(s=0.0,k=l;k<m;k++) s += A(k,i)*A(k,j);
        f=(s/A(i,i))*g;
        for(k=i;k<m;k++) A(k,j) += f*A(k,i);
      }
      for(j=i;j<m;j++) A(j,i) *= g;
    } else for(j=i;j<m;j++) A(j,i)=0.0;
    ++A(i,i);
  }
  for(k=n-1;k>=0;k--) {
    for(its=0;its<30;its++) {
      flag=true;
      for(l=k;l>=0;l--) {
        nm=l-1;
        if (fabs(rv1[l])+anorm == anorm) {
          flag=false;
          break;
        }
        if (fabs(w[nm])+anorm == anorm) break;
      }
      if (flag) {
        c=0.0;
        s=1.0;
        for(i=l-1;i<k+1;i++) {
          f=s*rv1[i];
          rv1[i]=c*rv1[i];
          if (fabs(f)+anorm == anorm) break;
          g=w[i];
          h=pythag(f,g);
          w[i]=h;
          h=1.0/h;
          c=g*h;
          s = -f*h;
          for(j=0;j<m;j++) {
            y=A(j,nm);
            z=A(j,i);
            A(j,nm)=y*c+z*s;
            A(j,i)=z*c-y*s;
          }
        }
      }
      z=w[k];
      if (l == k) {
        if (z < 0.0) {
          w[k] = -z;
          for(j=0;j<n;j++) V(j,k) = -V(j,k);
        }
        break;
      }
      if (its == 29) {
        std::cout << "no convergence in 30 svdcmp iterations" << std::endl;
        return false;
      }
      x=w[l];
      nm=k-1;
      y=w[nm];
      g=rv1[nm];
      h=rv1[k];
      f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
      g=pythag(f,1.0);
      f=((x-z)*(x+z)+h*((y/(f+sign(g,f)))-h))/x;
      c=s=1.0;
      for(j=l;j<=nm;j++) {
        i=j+1;
        g=rv1[i];
        y=w[i];
        h=s*g;
        g=c*g;
        z=pythag(f,h);
        rv1[j]=z;
        c=f/z;
        s=h/z;
        f=x*c+g*s;
        g=g*c-x*s;
        h=y*s;
        y *= c;
        for(jj=0;jj<n;jj++) {
          x=V(jj,j);
          z=V(jj,i);
          V(jj,j)=x*c+z*s;
          V(jj,i)=z*c-x*s;
        }
        z=pythag(f,h);
        w[j]=z;
        if (z) {
          z=1.0/z;
          c=f*z;
          s=h*z;
        }
        f=c*g+s*y;
        x=c*y-s*g;
        for(jj=0;jj<m;jj++) {
          y=A(jj,j);
          z=A(jj,i);
          A(jj,j)=y*c+z*s;
          A(jj,i)=z*c-y*s;
        }
      }
      rv1[l]=0.0;
      rv1[k]=f;
      w[k]=x;
    }
  }
  return true;
}

void Matrixd::svbksb(Vectord &w, Matrixd &V, const Vectord &b, Vectord &x){
  Matrixd& U = *this;
  int jj,j,i;
  double s;

  int m = U.nrows();
  int n = U.ncols();
  Vectord tmp(n);
  for(j=0;j<n;j++) {
    s=0.0;
    if (w[j] != 0.0) {
      for(i=0;i<m;i++) s += U(i,j)*b[i];
      s /= w[j];
    }
    tmp[j]=s;
  }
  for(j=0;j<n;j++) {
    s=0.0;
    for(jj=0;jj<n;jj++) s += V(j,jj)*tmp[jj];
    x[j]=s;
  }
}

bool Matrixd::solve(const Vectord& b,Vectord& x) {
  Matrixd V(m,m);
  Vectord w(n);

  bool regular = svdcmp(w,V);
  if (!regular) return false;
  svbksb(w,V,b,x);
  return true;
}

