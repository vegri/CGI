#ifndef VECMATH
#define VECMATH

#define _USE_MATH_DEFINES
#include <cmath>
#include <ostream>

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#define VECMATH_VERSION 2

class Vector3d {
    public:

        double v[3];

        Vector3d() { v[0]=0.0; v[1]=0.0; v[2]=0.0;}
        Vector3d(double x,double y,double z) { v[0]=x; v[1]=y; v[2]=z; }
		Vector3d(double x) { v[0]=v[1]=v[2]=x; }

        inline bool operator == (const Vector3d& a) const { return v[0]==a.v[0] && v[1]==a.v[1] && v[2]==a.v[2]; }
        
        inline bool operator != (const Vector3d& a) const { return v[0]!=a.v[0] || v[1]!=a.v[1] || v[2]!=a.v[2]; }

        inline bool operator <  (const Vector3d& a) const {
            if (v[0]<a.v[0]) return true;
            else if (v[0]>a.v[0]) return false;
            else if (v[1]<a.v[1]) return true;
            else if (v[1]>a.v[1]) return false;
            else return (v[2]<a.v[2]);
        }

        inline bool epsilonEquals(const Vector3d& a,double eps) const {
            return (sqrt(dot(a)) < eps);
        }

        inline double* ptr() { return v; }
        inline const double* ptr() const { return v; }

		inline const double& maxComp() {
			if (v[0] < v[1])	return (v[1]<v[2])? v[2] : v[1];
			else				return (v[0]<v[2])? v[2] : v[0];
		}

		inline const double& minComp() {
			if (v[0] < v[1])	return (v[0]<v[2])? v[0] : v[2];
			else				return (v[1]<v[2])? v[1] : v[2];
		}

        inline void set( double x, double y, double z) {
            v[0]=x; v[1]=y; v[2]=z;
        }

        inline void set( const Vector3d& a) {
            v[0]=a.v[0]; v[1]=a.v[1]; v[2]=a.v[2];
        }

        inline double& operator [] (int i) { return v[i]; }
        inline double operator [] (int i) const { return v[i]; }

        inline double& x() { return v[0]; }
        inline double& y() { return v[1]; }
        inline double& z() { return v[2]; }

        inline double x() const { return v[0]; }
        inline double y() const { return v[1]; }
        inline double z() const { return v[2]; }

        inline double dot(const Vector3d& a) const {
            return v[0]*a.v[0]+v[1]*a.v[1]+v[2]*a.v[2];
        }

        inline double operator * (const Vector3d& a) const {
            return v[0]*a.v[0]+v[1]*a.v[1]+v[2]*a.v[2];
        }

        inline void cross(const Vector3d& a,const Vector3d& b) {
            v[0] = a.v[1]*b.v[2]-a.v[2]*b.v[1];
            v[1] = a.v[2]*b.v[0]-a.v[0]*b.v[2];
            v[2] = a.v[0]*b.v[1]-a.v[1]*b.v[0];
        }

        inline const Vector3d operator % (const Vector3d& a) const {
            return Vector3d(v[1]*a.v[2]-v[2]*a.v[1],
                         v[2]*a.v[0]-v[0]*a.v[2] ,
                         v[0]*a.v[1]-v[1]*a.v[0]);
        }

        inline const Vector3d operator * (double s) const {
            return Vector3d(v[0]*s,v[1]*s,v[2]*s);
        }

        inline Vector3d& operator *= (double s) {
            v[0] *= s;
            v[1] *= s;
            v[2] *= s;
            return *this;
        }

        inline const Vector3d operator / (double s) const {
            return Vector3d(v[0]/s,v[1]/s,v[2]/s);
        }

        inline Vector3d& operator /= (double s) {
            v[0] /= s;
            v[1] /= s;
            v[2] /= s;
            return *this;
        }

        inline const Vector3d operator + (const Vector3d& a) const {
            return Vector3d(v[0]+a.v[0],v[1]+a.v[1],v[2]+a.v[2]);
        }

        inline Vector3d& operator += (const Vector3d& a) {
            v[0] += a.v[0];
            v[1] += a.v[1];
            v[2] += a.v[2];
            return *this;
        }

        inline const Vector3d operator - (const Vector3d& a) const {
            return Vector3d(v[0]-a.v[0],v[1]-a.v[1],v[2]-a.v[2]);
        }

        inline Vector3d& operator -= (const Vector3d& a) {
            v[0] -= a.v[0];
            v[1] -= a.v[1];
            v[2] -= a.v[2];
            return *this;
        }

        inline const Vector3d operator - () const {
            return Vector3d (-v[0],-v[1],-v[2]);
        }

        inline double length() const {
            return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
        }

        inline double lengthSquared() const {
            return v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
        }

        double angle(const Vector3d& a) {
            double l = length();
            double al = a.length();
            double c = (v[0]*a.v[0]+v[1]*a.v[1]+v[2]*a.v[2])/(l*al);
            return acos(c);
        }

        inline void normalize() {
            double norm = length();
            if (norm > 0.0) {
                double inv = 1.0/norm;
                v[0] *= inv;
                v[1] *= inv;
                v[2] *= inv;
            }                
        }

		inline Vector3d normalized() const {
			Vector3d ret = *this;
			ret.normalize();
			return ret;
		}

        inline void normalize(const Vector3d& a) {
            double norm = a.length();
            if (norm > 0.0) {
                double inv = 1.0/norm;
                v[0] = a.v[0]*inv;
                v[1] = a.v[1]*inv;
                v[2] = a.v[2]*inv;
            }
        }
};

class Vector4d {
    public:

        double v[4];

        Vector4d() { v[0]=0.0; v[1]=0.0; v[2]=0.0; v[3]=0.0; }

        Vector4d(double x, double y, double z, double w) {
            v[0] = x;
            v[1] = y;
            v[2] = z;
            v[3] = w;
        }

        Vector4d(const Vector3d& v3,double w) {
            v[0] = v3[0];
            v[1] = v3[1];
            v[2] = v3[2];
            v[3] = w;
        }
            
        inline bool operator == (const Vector4d& v) const { 
            return v[0]==v.v[0] && v[1]==v.v[1] && 
                   v[2]==v.v[2] && v[3]==v.v[3]; 
        }

        inline bool operator != (const Vector4d& v) const { 
            return v[0]!=v.v[0] || v[1]!=v.v[1] || 
                   v[2]!=v.v[2] || v[3]!=v.v[3]; }

        inline bool operator <  (const Vector4d& v) const {
            if (v[0] < v.v[0]) return true;
            else if (v[0] > v.v[0]) return false;
            else if (v[1] < v.v[1]) return true;
            else if (v[1] > v.v[1]) return false;
            else if (v[2] < v.v[2]) return true;
            else if (v[2] > v.v[2]) return false;
            else return (v[3] < v.v[3]);
        }

        inline double* ptr() { return v; }
        inline const double* ptr() const { return v; }

        inline void set( double x, double y, double z, double w) {
            v[0]=x; v[1]=y; v[2]=z; v[3]=w;
        }

        inline double& operator [] (unsigned int i) { return v[i]; }
        inline double  operator [] (unsigned int i) const { return v[i]; }

        inline double& x() { return v[0]; }
        inline double& y() { return v[1]; }
        inline double& z() { return v[2]; }
        inline double& w() { return v[3]; }

        inline double x() const { return v[0]; }
        inline double y() const { return v[1]; }
        inline double z() const { return v[2]; }
        inline double w() const { return v[3]; }

        inline double& r() { return v[0]; }
        inline double& g() { return v[1]; }
        inline double& b() { return v[2]; }
        inline double& a() { return v[3]; }

        inline double r() const { return v[0]; }
        inline double g() const { return v[1]; }
        inline double b() const { return v[2]; }
        inline double a() const { return v[3]; }

        inline double operator * (const Vector4d& a) const {
            return v[0]*a.v[0]+ v[1]*a.v[1]+ v[2]*a.v[2]+ v[3]*a.v[3];
        }

        inline Vector4d operator * (double s) const {
            return Vector4d(v[0]*s, v[1]*s, v[2]*s, v[3]*s);
        }

        inline Vector4d& operator *= (double s) {
            v[0] *= s;
            v[1] *= s;
            v[2] *= s;
            v[3] *= s;
            return *this;
        }

        inline Vector4d operator / (double s) const {
            return Vector4d(v[0]/s, v[1]/s, v[2]/s, v[3]/s);
        }

        inline Vector4d& operator /= (double s) {
            v[0] /= s;
            v[1] /= s;
            v[2] /= s;
            v[3] /= s;
            return *this;
        }

        inline Vector4d operator + (const Vector4d& a) const {
            return Vector4d(v[0]+a.v[0],v[1]+a.v[1],v[2]+a.v[2],v[3]+a.v[3]);
        }

        inline Vector4d& operator += (const Vector4d& a) {
            v[0] += a.v[0];
            v[1] += a.v[1];
            v[2] += a.v[2];
            v[3] += a.v[3];
            return *this;
        }

        inline Vector4d operator - (const Vector4d& a) const {
            return Vector4d(v[0]-a.v[0],v[1]-a.v[1],v[2]-a.v[2],v[3]-a.v[3]);
        }

        inline Vector4d& operator -= (const Vector4d& a) {
            v[0] -= a.v[0];
            v[1] -= a.v[1];
            v[2] -= a.v[2];
            v[3] -= a.v[3];
            return *this;
        }

        inline const Vector4d operator - () const {
            return Vector4d (-v[0],-v[1],-v[2],-v[3]);
        }

        inline double length() const {
            return sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);
        }

        inline double lengthSquared() const {
            return v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3];
        }

        inline double normalize() {
            double norm = Vector4d::length();
            if (norm > 0.0) {
                double inv = 1.0/norm;
                v[0] *= inv;
                v[1] *= inv;
                v[2] *= inv;
                v[3] *= inv;
            }
            return( norm );
        }
};

class Vectord {
  double *v;
  int n;

  public:

  Vectord(int k) { 
    n = k;
    v = new double[n];
  }

  ~Vectord() {
    delete[] v;
  }

  inline double& operator [] (int i) { return v[i]; }
  inline double operator [] (int i) const { return v[i]; }
  void print();
};

class Matrix4d;

class Quat4d {

    public:

        double  q[4];

        inline Quat4d() { q[0]=0.0; q[1]=0.0; q[2]=0.0; q[3]=1.0; }

        inline Quat4d(double x, double y, double z, double w) {
            q[0] = x; q[1] = y; q[2] = z; q[3] = w;
        }

        inline Quat4d(Vector3d& v) {
            q[0] = v[0]; q[1] = v[1]; q[2] = v[2]; q[3] = 0.0;
        }

        inline Quat4d( double angle, const Vector3d& axis) {
            makeRotate(angle,axis);
        }

        inline Quat4d( double angle1, const Vector3d& axis1, 
                     double angle2, const Vector3d& axis2,
                     double angle3, const Vector3d& axis3) {
            makeRotate(angle1,axis1,angle2,axis2,angle3,axis3);
        }

        inline Quat4d& operator = (const Quat4d& a) { q[0]=a.q[0];  q[1]=a.q[1]; q[2]=a.q[2]; q[3]=a.q[3]; return *this; }

        inline bool operator == (const Quat4d& a) const { return q[0]==a.q[0] && q[1]==a.q[1] && q[2]==a.q[2] && q[3]==a.q[3]; }

        inline bool operator != (const Quat4d& a) const { return q[0]!=a.q[0] || q[1]!=a.q[1] || q[2]!=a.q[2] || q[3]!=a.q[3]; }

        inline bool operator < (const Quat4d& a) const {
            if (q[0] < a.q[0]) return true;
            else if (q[0] > a.q[0]) return false;
            else if (q[1] < a.q[1]) return true;
            else if (q[1] > a.q[1]) return false;
            else if (q[2] < a.q[2]) return true;
            else if (q[2] > a.q[2]) return false;
            else return (q[3] < a.q[3]);
        }

        inline void set(double x, double y, double z, double w) {
            q[0] = x;
            q[1] = y;
            q[2] = z;
            q[3] = w;
        }
        
        void set(const Matrix4d& M);
        
        void get(Matrix4d& M) const;

        inline double & operator [] (int i) { return q[i]; }
        inline double   operator [] (int i) const { return q[i]; }

        inline double & x() { return q[0]; }
        inline double & y() { return q[1]; }
        inline double & z() { return q[2]; }
        inline double & w() { return q[3]; }

        inline double x() const { return q[0]; }
        inline double y() const { return q[1]; }
        inline double z() const { return q[2]; }
        inline double w() const { return q[3]; }

        inline const Quat4d operator * (double s) const {
            return Quat4d(q[0]*s, q[1]*s, q[2]*s, q[3]*s);
        }

        inline Quat4d& operator *= (double s) {
            q[0] *= s;
            q[1] *= s;
            q[2] *= s;
            q[3] *= s;
            return *this;
        }

				/// BUGFIXED ELmar Sch"omer 2008 verified Kai Werth 20100519
        inline const Quat4d operator*(const Quat4d& a) const {
            return
            Quat4d( q[3]*a.q[0] + q[0]*a.q[3] + q[1]*a.q[2] - q[2]*a.q[1],
                    q[3]*a.q[1] - q[0]*a.q[2] + q[1]*a.q[3] + q[2]*a.q[0],
                    q[3]*a.q[2] + q[0]*a.q[1] - q[1]*a.q[0] + q[2]*a.q[3],
                    q[3]*a.q[3] - q[0]*a.q[0] - q[1]*a.q[1] - q[2]*a.q[2] );
        }

				/// BUGFIXED ELmar Sch"omer 2008 verified Kai Werth 20100519
        inline Quat4d& operator*=(const Quat4d& a) {
            double x = q[3]*a.q[0] + q[0]*a.q[3] + q[1]*a.q[2] - q[2]*a.q[1];
            double y = q[3]*a.q[1] - q[0]*a.q[2] + q[1]*a.q[3] + q[2]*a.q[0];
            double z = q[3]*a.q[2] + q[0]*a.q[1] - q[1]*a.q[0] + q[2]*a.q[3];
            q[3]     = q[3]*a.q[3] - q[0]*a.q[0] - q[1]*a.q[1] - q[2]*a.q[2];

            q[2] = z;
            q[1] = y;
            q[0] = x;

            return (*this);
        }

        inline Quat4d operator / (double s) const {
            double div = 1.0/s;
            return Quat4d(q[0]*div, q[1]*div, q[2]*div, q[3]*div);
        }

        inline Quat4d& operator /= (double s) {
            double div = 1.0/s;
            q[0] *= div;
            q[1] *= div;
            q[2] *= div;
            q[3] *= div;
            return *this;
        }

        inline const Quat4d operator/(const Quat4d& a) const {
            return ( (*this) * a.inverse() );
        }

        inline Quat4d& operator/=(const Quat4d& a) {
            (*this) = (*this) * a.inverse();
            return (*this);
        }

        inline const Quat4d operator + (const Quat4d& a) const {
            return Quat4d(q[0]+a.q[0],q[1]+a.q[1],q[2]+a.q[2],q[3]+a.q[3]);
        }

        inline Quat4d& operator += (const Quat4d& a) {
            q[0] += a.q[0];
            q[1] += a.q[1];
            q[2] += a.q[2];
            q[3] += a.q[3];
            return *this;
        }

        inline const Quat4d operator - (const Quat4d& a) const {
            return Quat4d(q[0]-a.q[0],q[1]-a.q[1],q[2]-a.q[2],q[3]-a.q[3] );
        }

        inline Quat4d& operator -= (const Quat4d& a) {
            q[0]-= a.q[0];
            q[1]-= a.q[1];
            q[2]-= a.q[2];
            q[3]-= a.q[3];
            return *this;
        }

        inline const Quat4d operator - () const {
            return Quat4d(-q[0],-q[1],-q[2],-q[3]);
        }

        double length() const {
            return sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
        }

        double lengthSquared() const {
            return q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
        }

        inline Quat4d conjugate () const { 
             return Quat4d(-q[0],-q[1],-q[2],q[3] );
        }

        inline const Quat4d inverse () const {
             return conjugate() / lengthSquared();
        }

        inline void normalize() {
            double norm = length();
            if (norm > 0.0) {
                double inv = 1.0/norm;
                q[0] *= inv;
                q[1] *= inv;
                q[2] *= inv;
                q[3] *= inv;
            }                
        }

        void makeRotate( double  angle, double  x, double  y, double  z );

        void makeRotate( double  angle, const Vector3d& vec );

        void makeRotate( double  angle1, const Vector3d& axis1, 
                         double  angle2, const Vector3d& axis2,
                         double  angle3, const Vector3d& axis3);

        void makeRotate( const Vector3d& vec1, const Vector3d& vec2 );
    
        void getRotate ( double & angle, double & x, double & y, double & z ) const;

        void getRotate ( double & angle, Vector3d& vec ) const;

        void slerp( double  t, const Quat4d& from, const Quat4d& to);
               
        Vector3d operator* (const Vector3d& v) const {
            Vector3d uv, uuv; 
            Vector3d r(q[0],q[1],q[2]);
            uv = r % v;
            uuv = r % uv; 
            uv *= 2.0 * q[3]; 
            uuv *= 2.0; 
            return v + uv + uuv;
        }
};

class Matrix4d {
    public:
    
        inline Matrix4d() { makeIdentity(); }
        inline Matrix4d( const Matrix4d& A) { set(A.ptr()); }
        inline Matrix4d( double const * const ptr ) { set(ptr); }
        inline Matrix4d( const Quat4d& quat ) { makeRotate(quat); }

        Matrix4d(double a00, double a01, double a02, double a03,
                 double a10, double a11, double a12, double a13,
                 double a20, double a21, double a22, double a23,
                 double a30, double a31, double a32, double a33);

        ~Matrix4d() {}

        int compare(const Matrix4d& A) const;

        bool operator < (const Matrix4d& A) const { return compare(A)<0; }
        bool operator == (const Matrix4d& A) const { return compare(A)==0; }
        bool operator != (const Matrix4d& A) const { return compare(A)!=0; }

        inline double& operator()(int row, int col) { return M[row][col]; }
        inline double operator()(int row, int col) const { return M[row][col]; }

        inline Matrix4d& operator = (const Matrix4d& A) {
            if( &A == this ) return *this;
            set(A.ptr());
            return *this;
        }
        
        inline void set(const Matrix4d& A) { set(A.ptr()); }

        inline void set(double const * const ptr) {
            double* local_ptr = (double*)M;
            for(int i=0;i<16;++i) 
              local_ptr[i]=(double)ptr[i];
        }

        void set(double a00, double a01, double a02,double a03,
                 double a10, double a11, double a12,double a13,
                 double a20, double a21, double a22,double a23,
                 double a30, double a31, double a32,double a33);
                  
        double * ptr() { return (double*)M; }
        const double * ptr() const { return (const double *)M; }

        bool isIdentity() const {
            return M[0][0]==1.0 && M[0][1]==0.0 && M[0][2]==0.0 &&  M[0][3]==0.0 &&
                   M[1][0]==0.0 && M[1][1]==1.0 && M[1][2]==0.0 &&  M[1][3]==0.0 &&
                   M[2][0]==0.0 && M[2][1]==0.0 && M[2][2]==1.0 &&  M[2][3]==0.0 &&
                   M[3][0]==0.0 && M[3][1]==0.0 && M[3][2]==0.0 &&  M[3][3]==1.0;
        }
        Matrix4d transpose() const;

        inline static Matrix4d identity();
        inline static Matrix4d scale( const Vector3d& sv);
        inline static Matrix4d scale( double sx, double sy, double sz);
        inline static Matrix4d translate( const Vector3d& dv);
        inline static Matrix4d translate( double x, double y, double z);
        inline static Matrix4d rotate( const Vector3d& from, const Vector3d& to);
        inline static Matrix4d rotate( double angle, double x, double y, double z);
        inline static Matrix4d rotate( double angle, const Vector3d& axis);
        inline static Matrix4d rotate( double angle1, const Vector3d& axis1, 
                                      double angle2, const Vector3d& axis2,
                                      double angle3, const Vector3d& axis3);
        inline static Matrix4d rotate( const Quat4d& quat);
        inline static Matrix4d inverse( const Matrix4d& matrix);
        inline static Matrix4d orthoNormal(const Matrix4d& matrix); 
        inline static Matrix4d ortho(double left,   double right,
                                    double bottom, double top,
                                    double zNear,  double zFar);
        inline static Matrix4d ortho2D(double left,   double right,
                                      double bottom, double top);
        inline static Matrix4d frustum(double left,   double right,
                                      double bottom, double top,
                                      double zNear,  double zFar);
        inline static Matrix4d perspective(double fovy,  double aspectRatio,
                                          double zNear, double zFar);
        inline static Matrix4d lookAt(const Vector3d& eye,
                                     const Vector3d& center,
                                     const Vector3d& up);
        void makeIdentity();
        
        void makeScale( const Vector3d& );
        void makeScale( double, double, double );
        
        void makeTranslate( const Vector3d& );
        void makeTranslate( double, double, double );
        
        void makeRotate( const Vector3d& from, const Vector3d& to );
        void makeRotate( double angle, const Vector3d& axis );
        void makeRotate( double angle, double x, double y, double z );
        void makeRotate( const Quat4d& );
        void makeRotate( double angle1, const Vector3d& axis1, 
                         double angle2, const Vector3d& axis2,
                         double angle3, const Vector3d& axis3);

        void decompose( Vector3d& translation, Quat4d& rotation, 
                        Vector3d& scale, Quat4d& so ) const;

        void jacobiRot(double s, double tau, int i, int j, int k, int l);

        int jacobi(Vector4d& d, Matrix4d& V, int& nrot);

        void makeOrtho(double left,   double right,
                       double bottom, double top,
                       double zNear,  double zFar);

        bool getOrtho(double& left,   double& right,
                      double& bottom, double& top,
                      double& zNear,  double& zFar) const;

        inline void makeOrtho2D(double left,   double right,
                                double bottom, double top) {
            makeOrtho(left,right,bottom,top,-1.0,1.0);
        }

        void makeFrustum(double left,   double right,
                         double bottom, double top,
                         double zNear,  double zFar);

        bool getFrustum(double& left,   double& right,
                        double& bottom, double& top,
                        double& zNear,  double& zFar) const;

        void makePerspective(double fovy,  double aspectRatio,
                             double zNear, double zFar);

        bool getPerspective(double& fovy,  double& aspectRatio,
                            double& zNear, double& zFar) const;

        void makeLookAt(const Vector3d& eye,const Vector3d& center,const Vector3d& up);

        void getLookAt(Vector3d& eye,Vector3d& center,Vector3d& up,
                       double lookDistance=1.0f) const;

        inline bool invert( const Matrix4d& A) {
            bool is_4x3 = (A.M[0][3]==0.0 && A.M[1][3]==0.0 &&  A.M[2][3]==0.0 && A.M[3][3]==1.0);
            return is_4x3 ? invert_4x3(A) :  invert_4x4(A);
        }

        /** 4x3 matrix invert, not right hand column is assumed to be 0,0,0,1. */
        bool invert_4x3( const Matrix4d& A);

        /** full 4x4 matrix invert. */
        bool invert_4x4( const Matrix4d& A);

        /** ortho-normalize the 3x3 rotation & scale matrix */ 
        void orthoNormalize(const Matrix4d& A); 

        inline Vector3d preMult( const Vector3d& v ) const;
        inline Vector3d postMult( const Vector3d& v ) const;
        inline Vector3d operator* ( const Vector3d& v ) const;
        inline Vector4d preMult( const Vector4d& v ) const;
        inline Vector4d postMult( const Vector4d& v ) const;
        inline Vector4d operator* ( const Vector4d& v ) const;

        void setRotate(const Quat4d& q);
        Quat4d getRotate() const;

        void setTrans( double tx, double ty, double tz );
        void setTrans( const Vector3d& v );
        
        inline Vector3d getTrans() const { return Vector3d(M[3][0],M[3][1],M[3][2]); } 
        
        inline Vector3d getScale() const {
          Vector3d x_vec(M[0][0],M[1][0],M[2][0]); 
          Vector3d y_vec(M[0][1],M[1][1],M[2][1]); 
          Vector3d z_vec(M[0][2],M[1][2],M[2][2]); 
          return Vector3d(x_vec.length(), y_vec.length(), z_vec.length()); 
        }
        
        /** apply a 3x3 transform of v*M[0..2,0..2]. */
        inline static Vector3d transform3x3(const Vector3d& v,const Matrix4d& A);

        /** apply a 3x3 transform of M[0..2,0..2]*v. */
        inline static Vector3d transform3x3(const Matrix4d& A,const Vector3d& v);

        // basic Matrix4d multiplication, our workhorse methods.
        void mult( const Matrix4d&, const Matrix4d& );
        void preMult( const Matrix4d& );
        void postMult( const Matrix4d& );

        inline void operator *= ( const Matrix4d& other ) {    
            if( this == &other ) {
                Matrix4d temp(other);
                postMult( temp );
            }
            else postMult( other ); 
        }

        inline Matrix4d operator * ( const Matrix4d &m ) const {
            Matrix4d A;
            A.mult(*this,m);
            return A;
        }

    protected:
        double M[4][4];

};

class Matrixd {
  double **M;
  int m,n;

  public:

  Matrixd(int k,int l) {
    m = k; n = l;
    M = new double*[m];
    for(int i=0;i<m;i++)
      M[i] = new double[n];
  }

  ~Matrixd() {
    for(int i=0;i<m;i++)
      delete[] M[i];
    delete M;
  }

	Matrixd(const Matrixd& ref)
	{
		m = ref.m; n = ref.n;
    M = new double*[m];
    for(int i=0;i<m;i++)
      M[i] = new double[n];
		for(int y=0;y<m;y++)
			for(int x=0;x<n;x++)
				M[y][x] = ref.M[y][x];	
	}

  inline int nrows() { return m; }
  inline int ncols() { return n; }
  inline double& operator()(int i, int j) { return M[i][j]; }
  inline double operator()(int i, int j) const { return M[i][j]; }

  void print();
  bool solve(const Vectord &b, Vectord &x); 

  private:

  bool svdcmp(Vectord &w, Matrixd &V);
  void svbksb(Vectord &w, Matrixd &V, const Vectord &b, Vectord &x);

  inline double sqr(const double x) { return x*x; }
  inline double min(const double a,const double b) {
    return (b > a) ? a : b;
  }
  inline double max(const double a,const double b) {
    return (b > a) ? b : a;
  }
  inline double sign(const double a, const double b) {
    return (b >= 0) ? ((a >= 0) ? a : -a) : ((a >= 0) ? -a : a);
  }

  double pythag(const double a, const double b);
};

inline double operator * (const Vector3d& a,const Vector4d& b) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+b[3];
}

inline double operator * (const Vector4d& a,const Vector3d& b) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3];
}

inline Matrix4d Matrix4d::identity(void) {
    Matrix4d A;
    A.makeIdentity();
    return A;
}

inline Matrix4d Matrix4d::scale(double sx, double sy, double sz) {
    Matrix4d A;
    A.makeScale(sx,sy,sz);
    return A;
}

inline Matrix4d Matrix4d::scale(const Vector3d& v ) {
    return scale(v.x(),v.y(),v.z());
}

inline Matrix4d Matrix4d::translate(double tx, double ty, double tz) {
    Matrix4d A;
    A.makeTranslate(tx,ty,tz);
    return A;
}

inline Matrix4d Matrix4d::translate(const Vector3d& v ) {
    return translate(v.x(),v.y(),v.z() );
}

inline Matrix4d Matrix4d::rotate( const Quat4d& q ) {
    return Matrix4d(q);
}

inline Matrix4d Matrix4d::rotate(double angle, double x, double y, double z ) {
    Matrix4d A;
    A.makeRotate(angle,x,y,z);
    return A;
}

inline Matrix4d Matrix4d::rotate(double angle, const Vector3d& axis ) {
    Matrix4d A;
    A.makeRotate(angle,axis);
    return A;
}

inline Matrix4d Matrix4d::rotate( double angle1, const Vector3d& axis1, 
                                  double angle2, const Vector3d& axis2,
                                  double angle3, const Vector3d& axis3) {
    Matrix4d A;
    A.makeRotate(angle1,axis1,angle2,axis2,angle3,axis3);
    return A;
}

inline Matrix4d Matrix4d::rotate(const Vector3d& from, const Vector3d& to ) {
    Matrix4d A;
    A.makeRotate(from,to);
    return A;
}

inline Matrix4d Matrix4d::inverse( const Matrix4d& matrix) {
    Matrix4d A;
    A.invert(matrix);
    return A;
}

inline Matrix4d Matrix4d::orthoNormal(const Matrix4d& matrix) {
  Matrix4d A;
  A.orthoNormalize(matrix);
  return A; 
}

inline Matrix4d Matrix4d::ortho(double left,   double right,
                                double bottom, double top,
                                double zNear,  double zFar) {
    Matrix4d A;
    A.makeOrtho(left,right,bottom,top,zNear,zFar);
    return A;
}

inline Matrix4d Matrix4d::ortho2D(double left,   double right,
                                double bottom, double top) {
    Matrix4d A;
    A.makeOrtho2D(left,right,bottom,top);
    return A;
}

inline Matrix4d Matrix4d::frustum(double left,   double right,
                                  double bottom, double top,
                                  double zNear,  double zFar) {
    Matrix4d A;
    A.makeFrustum(left,right,bottom,top,zNear,zFar);
    return A;
}

inline Matrix4d Matrix4d::perspective(double fovy,  double aspectRatio,
                                      double zNear, double zFar) {
    Matrix4d A;
    A.makePerspective(fovy,aspectRatio,zNear,zFar);
    return A;
}

inline Matrix4d Matrix4d::lookAt(const Vector3d& eye,
                                 const Vector3d& center,
                                 const Vector3d& up) {
    Matrix4d A;
    A.makeLookAt(eye,center,up);
    return A;
}

inline Vector3d Matrix4d::postMult( const Vector3d& v ) const {
    double d = 1.0f/(M[3][0]*v.x()+M[3][1]*v.y()+M[3][2]*v.z()+M[3][3]) ;
    return Vector3d( (M[0][0]*v.x() + M[0][1]*v.y() + M[0][2]*v.z() + M[0][3])*d,
        (M[1][0]*v.x() + M[1][1]*v.y() + M[1][2]*v.z() + M[1][3])*d,
        (M[2][0]*v.x() + M[2][1]*v.y() + M[2][2]*v.z() + M[2][3])*d) ;
}

inline Vector3d Matrix4d::preMult( const Vector3d& v ) const {
    double d = 1.0f/(M[0][3]*v.x()+M[1][3]*v.y()+M[2][3]*v.z()+M[3][3]) ;
    return Vector3d( (M[0][0]*v.x() + M[1][0]*v.y() + M[2][0]*v.z() + M[3][0])*d,
        (M[0][1]*v.x() + M[1][1]*v.y() + M[2][1]*v.z() + M[3][1])*d,
        (M[0][2]*v.x() + M[1][2]*v.y() + M[2][2]*v.z() + M[3][2])*d);
}

inline Vector4d Matrix4d::postMult( const Vector4d& v ) const {
    return Vector4d( (M[0][0]*v.x() + M[0][1]*v.y() + M[0][2]*v.z() + M[0][3]*v.w()),
        (M[1][0]*v.x() + M[1][1]*v.y() + M[1][2]*v.z() + M[1][3]*v.w()),
        (M[2][0]*v.x() + M[2][1]*v.y() + M[2][2]*v.z() + M[2][3]*v.w()),
        (M[3][0]*v.x() + M[3][1]*v.y() + M[3][2]*v.z() + M[3][3]*v.w())) ;
}

inline Vector4d Matrix4d::preMult( const Vector4d& v ) const {
    return Vector4d( (M[0][0]*v.x() + M[1][0]*v.y() + M[2][0]*v.z() + M[3][0]*v.w()),
        (M[0][1]*v.x() + M[1][1]*v.y() + M[2][1]*v.z() + M[3][1]*v.w()),
        (M[0][2]*v.x() + M[1][2]*v.y() + M[2][2]*v.z() + M[3][2]*v.w()),
        (M[0][3]*v.x() + M[1][3]*v.y() + M[2][3]*v.z() + M[3][3]*v.w()));
}

inline Vector3d Matrix4d::transform3x3(const Vector3d& v,const Matrix4d& m) {
    return Vector3d( (m.M[0][0]*v.x() + m.M[1][0]*v.y() + m.M[2][0]*v.z()),
                 (m.M[0][1]*v.x() + m.M[1][1]*v.y() + m.M[2][1]*v.z()),
                 (m.M[0][2]*v.x() + m.M[1][2]*v.y() + m.M[2][2]*v.z()));
}

inline Vector3d Matrix4d::transform3x3(const Matrix4d& m,const Vector3d& v) {
    return Vector3d( (m.M[0][0]*v.x() + m.M[0][1]*v.y() + m.M[0][2]*v.z()),
                 (m.M[1][0]*v.x() + m.M[1][1]*v.y() + m.M[1][2]*v.z()),
                 (m.M[2][0]*v.x() + m.M[2][1]*v.y() + m.M[2][2]*v.z()) ) ;
}

inline Vector3d operator* (const Vector3d& v, const Matrix4d& m ) {
    return m.preMult(v);
}

inline Vector4d operator* (const Vector4d& v, const Matrix4d& m ) {
    return m.preMult(v);
}

inline Vector3d Matrix4d::operator* (const Vector3d& v) const {
    return postMult(v);
}

inline Vector4d Matrix4d::operator* (const Vector4d& v) const {
    return postMult(v);
}

inline std::ostream& operator << (std::ostream& out, const Vector3d& a) {
            out << "(" << a.v[0] << "," << a.v[1] << "," << a.v[2] << ")";
            return out;
}

inline bool epsilonEquals(const Vector3d& a,const Vector3d& b) {
  return (a-b)*(a-b) < 1e-10;
}

inline Vector3d normal(const Vector3d& a,const Vector3d& b,const Vector3d& c) {
  Vector3d n = (b-a)%(c-a);
  n.normalize();
  return n;
}

inline double det(const Vector3d& a,const Vector3d& b,const Vector3d& c) {
  return a*(b%c);
}

inline bool equivalent(double a,double b,double epsilon=1e-6) { 
  double delta = b-a; return delta<0.0?delta>=-epsilon:delta<=epsilon; 
}

#endif
