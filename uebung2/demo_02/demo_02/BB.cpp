#include <iostream>
#include <cmath>
#include <QGLWidget>
#include "vecmath.h"
#include "BB.h"


OBB::OBB(const std::vector<Vector3d>& p){
    //calculate covariance matrix
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
    axis1(V[0][0], V[0][1], V[0][2]);
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
