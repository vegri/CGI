#include <iostream>
#include <cmath>
#include <QGLWidget>
#include "vecmath.h"
#include "BB.h"


OBB::OBB(const std::vector<Vector3d>& vertices){
    //calculate covariance matrix
    p=vertices;
    c11=0, c22=0, c33=0, c12=0, c13=0, c23=0;

    for(unsigned int i=0;i<p.size();i++) {
        c11+=p[i][0]*p[i][0];
        c22+=p[i][1]*p[i][1];
        c33+=p[i][2]*p[i][2];
        c12+=p[i][0]*p[i][1];
        c13+=p[i][0]*p[i][2];
        c23+=p[i][1]*p[i][2];
    }
    c=Matrix4d(c11,c12,c13,0,c12,c22,c23,0,c13,c23,c33,0,0,0,0,1);
    c.jacobi(d, V, nrot);
    axis1=Vector3d(V(0,0), V(1,0), V(2,0));
    axis2=Vector3d(V(0,1), V(1,1), V(2,1));
    axis3=Vector3d(V(0,2), V(1,2), V(2,2));
    a1=0;
    a2=0;
    a3=0;
    std::cout <<"axis 1: " << axis1[0] << ", " << axis1[1] << ", " << axis1[2] << std::endl;
    std::cout <<"axis 2: " << axis2[0] << ", " << axis2[1] << ", " << axis2[2] << std::endl;
    std::cout <<"axis 3: " << axis3[0] << ", " << axis3[1] << ", " << axis3[2] << std::endl;
//    std::cout <<"1. ev: " <<  axis1[0]  << std::endl;
//    std::cout <<"2. ev: " <<  a2 << std::endl;
//    std::cout <<"3. ev: " <<  a3 << std::endl;

    for(int i=0; i<3; i++){
        if(a1<V(0,i)){
            a1=V(0,i);
        }
    }
    for(int i=0; i<3; i++){
        if(a2<V(1,i)){
            a2=V(1,i);
        }
    }
    for(int i=0; i<3; i++){
        if(a3<V(2,i)){
            a3=V(2,i);
        }
    }
    center=Vector3d(0,0,0);
        std::cout <<"a1: " <<  a1  << std::endl;
        std::cout <<"a2: " <<  a2 << std::endl;
        std::cout <<"a3: " <<  a3 << std::endl;

}

bool OBB::intersect(const OBB& B){
    Vector3d c=center-B.center;
    std::vector<Vector3d> v;
    v.resize(15);
    v.push_back(axis1);
    v.push_back(axis2);
    v.push_back(axis3);
    v.push_back(B.axis1);
    v.push_back(B.axis2);
    v.push_back(B.axis3);
    v.push_back(axis1%B.axis1);
    v.push_back(axis1%B.axis2);
    v.push_back(axis1%B.axis3);
    v.push_back(axis2%B.axis1);
    v.push_back(axis2%B.axis2);
    v.push_back(axis2%B.axis3);
    v.push_back(axis3%B.axis1);
    v.push_back(axis3%B.axis2);
    v.push_back(axis3%B.axis3);
    for(unsigned int i=0; i<v.size(); i++){
        if(v[i]*c < (a1*abs(axis1*v[i])+a2*abs(axis2*v[i])+a3*abs(axis3*v[i]) + B.a1*abs(B.axis1*v[i]) + B.a2*abs(B.axis2*v[i]) + B.a3*abs(B.axis3*v[i]))){
            return true;
        }
    }
    return false;
}

void OBB::splitOBB(const OBB& A, OBB& A1, OBB& A2){

    float longestSide=0;
    Vector3d longestAxis;
    if(longestSide<a1){
        longestSide=a1;
        longestAxis=axis1;
    }
    if(longestSide<a2){
        longestSide=a2;
        longestAxis=axis2;
    }
    if(longestSide<a3){
        longestSide=a3;
        longestAxis=axis3;
    }
    for(unsigned int i=0;i<p.size();i++) {

    }
}

AABB::AABB(const std::vector<Vector3d> p){
    xmin= p[0][0];
    xmax=xmin;
    ymin=p[0][1];
    ymax=ymin;
    zmin= p[0][2];
    zmax=zmin;
    for(unsigned int i=1;i<p.size();i++) {
        if(p[i][0]<xmin ){
            xmin=p[i][0];
        }
        if(p[i][0]>xmax ){
            xmax=p[i][0];
        }
        if(p[i][1]<ymin ){
            ymin=p[i][1];
        }
        if(p[i][1]>ymax ){
            ymax=p[i][1];
        }
        if(p[i][2]<zmin ){
            zmin=p[i][2];
        }
        if(p[i][2]>zmax ){
            zmax=p[i][2];
        }
    }
    huellQuader[0]=Vector3d(xmin,ymin,zmin);
    huellQuader[1]=Vector3d(xmin,ymax,zmin);
    huellQuader[2]=Vector3d(xmin,ymin,zmax);
    huellQuader[3]=Vector3d(xmin,ymax,zmax);
    huellQuader[4]=Vector3d(xmax,ymin,zmin);
    huellQuader[5]=Vector3d(xmax,ymin,zmax);
    huellQuader[6]=Vector3d(xmax,ymax,zmin);
    huellQuader[7]=Vector3d(xmax,ymax,zmax);

}


void AABB::draw(double rot, double gruen, double blau){
    glLineWidth(3.);
    glBegin(GL_LINE_LOOP);
    glColor3d(rot, gruen, blau);
    glVertex3dv(huellQuader[2].ptr());
    glVertex3dv(huellQuader[5].ptr());
    glVertex3dv(huellQuader[4].ptr());
    glVertex3dv(huellQuader[0].ptr());
    glVertex3dv(huellQuader[2].ptr());
    glVertex3dv(huellQuader[3].ptr());
    glVertex3dv(huellQuader[7].ptr());
    glVertex3dv(huellQuader[5].ptr());
    glVertex3dv(huellQuader[7].ptr());
    glVertex3dv(huellQuader[6].ptr());
    glVertex3dv(huellQuader[4].ptr());
    glVertex3dv(huellQuader[6].ptr());
    glVertex3dv(huellQuader[1].ptr());
    glVertex3dv(huellQuader[0].ptr());
    glVertex3dv(huellQuader[1].ptr());
    glVertex3dv(huellQuader[3].ptr());
    glEnd();
}

bool AABB::intersect (const AABB& B){

    if(xmax>=B.xmin && B.xmax>=xmin && ymax>=B.ymin && B.ymax>=ymin && zmax>=B.zmin && B.zmax>=zmin){
        return true;
    }
    return false;
}
