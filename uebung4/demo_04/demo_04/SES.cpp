#include <QApplication>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QGLWidget>
#include <QKeyEvent>
#include <QMessageBox>
#include <QHBoxLayout>

#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <limits>

#include "SES.h"
#include "Sphere.h"

const int numpoints = 100;
static std::vector<Vector3d> point;
Sphere SES;

CGMainWindow::CGMainWindow (QWidget* parent, Qt::WindowFlags flags)
    : QMainWindow (parent, flags) {
    resize (604, 614);

    // Create a nice frame to put around the OpenGL widget
    QFrame* f = new QFrame (this);
    f->setFrameStyle(QFrame::Sunken | QFrame::Panel);
    f->setLineWidth(2);

    // Create our OpenGL widget
    ogl = new CGView (this,f);

    // Create a menu
    QMenu *file = new QMenu("&File",this);
    file->addAction ("Quit", this, SLOT(close()), Qt::CTRL+Qt::Key_Q);

    menuBar()->addMenu(file);

    QAction *action;
    // Create a 'View' menu
    QMenu *show = new QMenu("&Show",this);
    action = show->addAction("show sphere", ogl, SLOT(toggleSphere()), Qt::Key_C);
    action->setCheckable(true);
    menuBar()->addMenu(show);

    // Put the GL widget inside the frame
    QHBoxLayout* layout = new QHBoxLayout();
    layout->addWidget(ogl);
    layout->setMargin(0);
    f->setLayout(layout);

    setCentralWidget(f);

    statusBar()->showMessage("Ready",1000);
}

CGMainWindow::~CGMainWindow () {}

CGView::CGView (CGMainWindow *mainwindow,QWidget* parent ) : QGLWidget (parent), quad(NULL) {
    main = mainwindow;

    show_circle=false;
    bbox_on = true;

    SES = Sphere::com (point);

    picked = 0;
    kthPoint=0;

    /// Um Keyboard-Events durchzulassen
    setFocusPolicy(Qt::StrongFocus);

}

void CGView::initializeGL() {
    glClearColor(1.0,1.0,1.0,1.0);
    center = 0.0;
    zoom = .7;
    q_now = Quat4d(1,1,-1,-1);//Quat4d(0.0,0.0,0.0,1.0);

    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

    Vector3d c = Vector3d(0.0,0.0,0.0);
    Vector3d l = Vector3d(2.0,0.8,0.1);
    Matrix4d R = Matrix4d::rotate(0.5,-1.0,0.4,0.5);

    //Zufallsgenerator fuer Punkte
    point.resize(numpoints);
    for(int i=0;i<numpoints;i++) {
        for(int j=0;j<3;j++)
            point[i][j] = l[j]*(double(rand())/RAND_MAX-0.5);
        point[i] = R*point[i]+c;
    }
}

void CGView::drawBoundingBox() {
    double maxX = 1;
    double maxY = 1;
    double maxZ = 1;
    double minX = -maxX;
    double minY = -maxY;
    double minZ = -maxZ;

    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);

    glBegin(GL_LINE_LOOP);
    glVertex3d(minX,minY,minZ);
    glVertex3d(maxX,minY,minZ);
    glVertex3d(maxX,maxY,minZ);
    glVertex3d(minX,maxY,minZ);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3d(minX,minY,maxZ);
    glVertex3d(maxX,minY,maxZ);
    glVertex3d(maxX,maxY,maxZ);
    glVertex3d(minX,maxY,maxZ);
    glEnd();
    glBegin(GL_LINES);
    glVertex3d(minX,minY,minZ);
    glVertex3d(minX,minY,maxZ);
    glVertex3d(maxX,minY,minZ);
    glVertex3d(maxX,minY,maxZ);
    glVertex3d(minX,maxY,minZ);
    glVertex3d(minX,maxY,maxZ);
    glVertex3d(maxX,maxY,minZ);
    glVertex3d(maxX,maxY,maxZ);
    glEnd();
}

void CGView::paintGL() {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


    glTranslated(0.0,0.0,-3.0);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Matrix4d R(q_now);
    Matrix4d RT = R.transpose();
    glMultMatrixd(RT.ptr());

    glScaled(zoom,zoom,zoom);
    if (bbox_on) drawBoundingBox();

    GLUquadricObj *quadric;
    quadric = gluNewQuadric();


    glDisable (GL_CULL_FACE);

    Sphere ses;

    glColor3d(0.0,1.0,0.0);
    if(kthPoint==0){
        ses=Sphere(point[0]);
        glPushMatrix();
        glTranslated(ses.center[0],ses.center[1],ses.center[2]);
        gluQuadricDrawStyle(quadric, GLU_FILL);
        gluSphere( quadric , 0.03 , 10 , 10);
        glPopMatrix();
    }
    if(kthPoint==1){
        ses=Sphere(point[0], point[1]);
        glPushMatrix();
        glTranslated(ses.center[0],ses.center[1],ses.center[2]);
        gluQuadricDrawStyle(quadric, GLU_FILL);
        gluSphere( quadric , ses.radius , 10 , 10);
        glPopMatrix();

        for(unsigned int i=0;i<2;i++) {
            glColor3d(1.0,0.0,0.0);
            glPushMatrix();
            glTranslated(point[i][0],point[i][1],point[i][2]);
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluSphere( quadric , 0.03  , 10 , 10);
            glPopMatrix();
        }
    }
    if(kthPoint==2){
        ses=Sphere(point[0], point[1], point[2]);
        glPushMatrix();
        glTranslated(ses.center[0],ses.center[1],ses.center[2]);
        gluQuadricDrawStyle(quadric, GLU_FILL);
        gluSphere( quadric , ses.radius , 10 , 10);
        glPopMatrix();

        for(unsigned int i=0;i<3;i++) {
            glColor3d(1.0,0.0,0.0);
            glPushMatrix();
            glTranslated(point[i][0],point[i][1],point[i][2]);
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluSphere( quadric , 0.03  , 10 , 10);
            glPopMatrix();
        }
    }
    if(kthPoint==3){
        ses=Sphere(point[0], point[1], point[2], point[3]);
        glPushMatrix();
        glTranslated(ses.center[0],ses.center[1],ses.center[2]);
        gluQuadricDrawStyle(quadric, GLU_FILL);
        gluSphere( quadric , ses.radius , 10 , 10);
        glPopMatrix();

        for(unsigned int i=0;i<4;i++) {
            glColor3d(1.0,0.0,0.0);
            glPushMatrix();
            glTranslated(point[i][0],point[i][1],point[i][2]);
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluSphere( quadric , 0.03  , 20 , 20);
            glPopMatrix();
        }
    }
    if(kthPoint>3){
        ses = Sphere(point);
        glPushMatrix();
        glTranslated(ses.center[0],ses.center[1],ses.center[2]);
        gluQuadricDrawStyle(quadric, GLU_FILL);
        gluSphere( quadric , ses.radius , 30 , 30);
        glPopMatrix();

        for(unsigned int i=4;i<point.size();i++) {
            glColor3d(1.0,0.0,0.0);
            if (i == picked) glColor3d(1.0,1.0,0.0);
            glPushMatrix();
            glTranslated(point[i][0],point[i][1],point[i][2]);
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluSphere( quadric , 0.03  , 10 , 10);
            glPopMatrix();
        }
    }


    if (show_circle)
    {
        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable (GL_CULL_FACE);
        glColor4d(0.0,0.0,1.0,0.5);
        glCullFace (GL_BACK);
        glPushMatrix();
        glTranslated(SES.center.x(), SES.center.y(), SES.center.z());
        gluQuadricDrawStyle(quadric, GLU_FILL);
        gluSphere( quadric , SES.radius , 40 , 40);
        glPopMatrix();
    }


}

void CGView::resizeGL(int width, int height) {
    glViewport(0,0,width,height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (width > height) {
        double ratio = width/(double) height;
        gluPerspective(45,ratio,1.0,100.0);
    }
    else {
        double ratio = height/(double) width;
        gluPerspective(45,1.0/ratio,1.0,100.0);
    }
    glMatrixMode (GL_MODELVIEW);
}

void CGView::worldCoord(int x, int y, int z, Vector3d &v) {
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    GLdouble M[16], P[16];
    glGetDoublev(GL_PROJECTION_MATRIX,P);
    glGetDoublev(GL_MODELVIEW_MATRIX,M);
    gluUnProject(x,viewport[3]-1-y,z,M,P,viewport,&v[0],&v[1],&v[2]);
}

void CGView::mousePressEvent(QMouseEvent *event) {
    oldX = event->x();
    oldY = event->y();
}

void CGView::mouseReleaseEvent(QMouseEvent*) {}

void CGView::wheelEvent(QWheelEvent* event) {
    if (event->delta() < 0) zoom *= 1.2; else zoom *= 1/1.2;
    updateGL();
}

void CGView::mouseToTrackball(int x, int y, int W, int H, Vector3d &v) {
    if (W > H) {
        v[0] = (2.0*x-W)/H;
        v[1] = 1.0-y*2.0/H;
    } else {
        v[0] = (2.0*x-W)/W;
        v[1] = (H-2.0*y)/W;
    }
    double d = v[0]*v[0]+v[1]*v[1];
    if (d > 1.0) {
        v[2] = 0.0;
        v /= sqrt(d);
    } else v[2] = sqrt(1.0-d*d);
}

Quat4d CGView::trackball(const Vector3d& u, const Vector3d& v) {
    Vector3d uxv = u % v;
    Quat4d ret(uxv[0],uxv[1],uxv[2],1.0+u*v);
    ret.normalize();
    return ret;
}

void CGView::mouseMoveEvent(QMouseEvent* event) {
    Vector3d p1,p2;

    mouseToTrackball(oldX,oldY,width(),height(),p1);
    mouseToTrackball(event->x(),event->y(),width(),height(),p2);

    Quat4d q = trackball(p1,p2);
    q_now = q * q_now;
    q_now.normalize();

    oldX = event->x();
    oldY = event->y();

    updateGL();
}

void CGView::keyPressEvent( QKeyEvent * event) 
{
    bool changed = true;

    switch (event->key()) {
    case Qt::Key_Space : picked = (picked+1)%numpoints; changed=false;break;
    case Qt::Key_X     : if (event->modifiers() & Qt::ShiftModifier) point[picked][0] -= 0.05; else point[picked][0] += 0.05; break;
    case Qt::Key_Y     : if (event->modifiers() & Qt::ShiftModifier) point[picked][1] -= 0.05; else point[picked][1] += 0.05; break;
    case Qt::Key_Z     : if (event->modifiers() & Qt::ShiftModifier) point[picked][2] -= 0.05; else point[picked][2] += 0.05; break;
    case Qt::Key_Plus  : kthPoint>3?kthPoint=0:kthPoint++ ; break;
    case Qt::Key_Minus : kthPoint==0?kthPoint=4:kthPoint-- ; break;
    }

    if (changed) {SES = Sphere::com(point);}

    updateGL();
}
int main (int argc, char **argv) {
    QApplication app(argc, argv);

    if (!QGLFormat::hasOpenGL()) {
        qWarning ("This system has no OpenGL support. Exiting.");
        return 1;
    }

    CGMainWindow *w = new CGMainWindow(NULL);

    w->show();

    return app.exec();
}

