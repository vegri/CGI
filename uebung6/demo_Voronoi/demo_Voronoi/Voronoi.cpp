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

#include "Voronoi.h"

const int numpoints = 8000;
static std::vector<Vector3d> point;

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


    //Zufallsgenerator fuer Punkte
    point.resize(numpoints);
    for(int i=0;i<numpoints;i++) {
        for(int j=0;j<3;j++)
            point[i][j] = 2.0*(double(rand())/RAND_MAX-0.5);
    }


    picked = 0;

    /// Um Keyboard-Events durchzulassen
    setFocusPolicy(Qt::StrongFocus);

}

void CGView::initializeGL() {
    glClearColor(0.4,0.4,0.5,1.0);
    center = 0.0;
    zoom = .7;
    q_now = Quat4d(1,1,-1,-1);//Quat4d(0.0,0.0,0.0,1.0);

    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

    randomSimplex();
    VoronoiCellSize = 0;

}


void CGView::randomSimplex(){
    simplex.clear();
    int size = rand()%4;
    for(int i = 0; i <= size; i++){
        double x = (double(rand())/RAND_MAX-0.5);
        double y = (double(rand())/RAND_MAX-0.5);
        double z = (double(rand())/RAND_MAX-0.5);
        simplex.push_back(Vector3d(x,y,z));
    }
    correctSimplexOrientation(simplex);
}


void CGView::correctSimplexOrientation(std::vector<Vector3d> & simplex){

    if(simplex.size() < 3)
        return;

    if(simplex.size() == 4) {
        Vector3d a = simplex.at(0);
        Vector3d b = simplex.at(1);
        Vector3d c = simplex.at(2);
        Vector3d d = simplex.at(3);
        Vector3d n = (b-a)%(c-a);
        if((d-a).dot(n) > 0){
            simplex.at(1) = d;
            simplex.at(3) = b;
        }
    }

    if(simplex.size() == 3) {
        Vector3d a = simplex.at(0);
        Vector3d b = simplex.at(1);
        Vector3d c = simplex.at(2);
        Vector3d n = (b-a)%(c-a);
        if((Vector3d(0,0,0)-a).dot(n) < 0){
            simplex.at(0) = b;
            simplex.at(1) = a;
        }
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



bool CGView::voronoiSurface(std::vector<Vector3d> &Q, const Vector3d &p, const Vector3d &a, const Vector3d &b,
                            const Vector3d &c, const Vector3d &normal){
    Vector3d h1=(b-a)%normal;
    Vector3d h2=(c-b)%normal;
    Vector3d h3=(a-c)%normal;
    if(((p-a)*h1<=0) && ((p-b)*h2<=0) && ((p-c)*h3<=0) && ((p-a)*normal>0)){
        Q.resize(3);
        return true;
    }
    return false;
}

bool CGView::voronoiPoint(std::vector<Vector3d> &Q, const Vector3d &p, const Vector3d &a, const Vector3d &b,
                          const Vector3d &c){
    if(((p-a)*(b-a)<=0) && ((p-a)*(c-a)<=0)){
        Q.resize(1);
        return true;
    }
    return false;
}

bool CGView::voronoiPoint(std::vector<Vector3d> &Q, const Vector3d &p, const Vector3d &a, const Vector3d &b,
                          const Vector3d &c, const Vector3d &d){

    if(((p-a)*(b-a)<=0) && ((p-a)*(d-a)<=0) && ((p-a)*(c-a)<=0)){
        Q.resize(1);
        return true;
    }
    return false;
}

bool CGView::voronoiEdge(std::vector<Vector3d> &Q, const Vector3d &p, const Vector3d &a, const Vector3d &b,
                         const Vector3d &normal1, const Vector3d &normal2){

    Vector3d h1=(a-b)%normal1;
    Vector3d h2=(b-a)%normal2;
    if(((p-a)*(b-a)>0) && ((p-b)*(a-b)>0) && (((p-a)*h1)<=0) && ((p-b)*h2<=0)){
        Q.resize(2);
        return true;
    }
    return false;
}

bool CGView::voronoiEdge(std::vector<Vector3d> &Q, const Vector3d &p, const Vector3d &a, const Vector3d &b){
    Vector3d h=(b-a)%n_abc;
    if((p-a)*h>=0){
        Q.resize(2);
        return true;
    }
    return false;
}

Vector3d CGView::com(const Vector3d &a, const Vector3d &b,
                     const Vector3d &c){
    Vector3d com;
    std::vector<Vector3d> Q;
    Q.push_back(a);
    Q.push_back(b);
    Q.push_back(c);
    for(int i=0; i<3; i++){
        com[0]+=Q[i][0];
        com[1]+=Q[i][1];
        com[2]+=Q[i][2];
    }
    return com=com/3;
}


// find feature of Q closest to p and direction of feature closest to p
bool CGView::simplexSolver(const Vector3d &p,
                           std::vector<Vector3d> &Q,
                           Vector3d &dir, Vector3d &color){

    if(Q.size() == 1){
        //        nothing to do actually
        return false;
    }

    if(Q.size() == 2){
        Vector3d a = Q.at(0);
        Vector3d b = Q.at(1);
        Q.clear();

        //Test if p is in V_a, V_b

        if((p-a)*(b-a)<=0){
            Q.resize(1);
            color=Vector3d(1.0,0.9,0.0);
            return false;
        }
        if((p-b)*(a-b)<=0){
            Q.resize(1);
            color=Vector3d(0.0,1.0,0.25);
            return false;
        }

        //else: p is in V_ab
        Q.resize(2);
        color=Vector3d(0.2,0.5,1.0);
        return false;
    }

    if(Q.size() == 3){
        Vector3d a = Q.at(0);
        Vector3d b = Q.at(1);
        Vector3d c = Q.at(2);
        Q.clear();

        //Test if p is in V_a, V_b, V_c

        if(voronoiPoint(Q, p, a, b, c)){
            color=Vector3d(1.0,0.9,0.0);
            return false;
        }
        if(voronoiPoint(Q, p, b, a, c)){
            color=Vector3d(0.0,1.0,0.25);
            return false;
        }
        if(voronoiPoint(Q, p, c, a, b)){
            color=Vector3d(0.2,0.5,1.0);
            return false;
        }

        //Test if p is in V_ab, V_ca, V_cb

        if(voronoiEdge(Q, p, a, b)){
            color=Vector3d(1.0,0.9,0.0);
            return false;
        }

        if(voronoiEdge(Q, p, b, c)){
            color=Vector3d(0.0,1.0,0.25);
            return false;
        }

        if(voronoiEdge(Q, p, c, a)){
            color=Vector3d(0.2,0.5,1.0);
            return false;
        }

        //else: p is in V_abc
        Q.resize(3);
        color=Vector3d(0.0,1.0,0.25);

        return false;
    }

    if(Q.size() == 4){
        Vector3d a = Q.at(0);
        Vector3d b = Q.at(1);
        Vector3d c = Q.at(2);
        Vector3d d = Q.at(3);
        Q.clear();

        //Test if p is in V_a, V_b, V_c, V_d

        if(voronoiPoint(Q, p, a, b, c, d)){
            color=Vector3d(1.0,0.9,0.0);
            return false;
        }
        if(voronoiPoint(Q, p, b, a, c, d)){
            color=Vector3d(0.0,1.0,0.25);
            return false;
        }
        if(voronoiPoint(Q, p, c, b, a, d)){
            color=Vector3d(0.2,0.5,1.0);
            return false;
        }
        if(voronoiPoint(Q, p, d, b, c, a)){
            color=Vector3d(1.0,0.0,0.15);
            return false;
        }

        //Test if p is in V_ab, V_bc, V_cd, V_da, V_ca, V_bd

        if(voronoiEdge(Q, p, a, b, n_abc, n_bad)){
            color=Vector3d(1.0,0.9,0.0);
            return false;
        }

        if(voronoiEdge(Q, p, b, c, n_abc, n_bdc)){
            color=Vector3d(0.0,1.0,0.25);
            return false;
        }

        if(voronoiEdge(Q, p, b, d, n_bdc, n_bad)){
            color=Vector3d(0.2,0.5,1.0);
            return false;
        }

        if(voronoiEdge(Q, p, a, d, n_bad, n_dac)){
            color=Vector3d(1.0,0.0,0.15);
            return false;
        }

        if(voronoiEdge(Q, p, a, c, n_dac, n_abc)){
            color=Vector3d(0.9,0.0,0.9);
            return false;
        }

        if(voronoiEdge(Q, p, c, d,n_dac, n_bdc)){
            color=Vector3d(0.0,0.0,0.0);
            return false;
        }

        //Test if p is in V_abc, V_bdc, V_adb, V_dac

        if(voronoiSurface(Q, p, a, b, c, n_abc)){
            color=Vector3d(1.0,0.9,0.0);
            return false;
        }

        if(voronoiSurface(Q, p, b, d, c, n_bdc)){
            color=Vector3d(0.0,1.0,0.25);
            return false;
        }

        if(voronoiSurface(Q, p, a, d, b, n_bad)){
            color=Vector3d(0.2,0.5,1.0);
            return false;
        }

        if(voronoiSurface(Q, p, d, a, c, n_dac)){
            color=Vector3d(1.0,0.0,0.2);
            return false;
        }

        //else: p is in V_abcd
        Q.resize(4);
        color=Vector3d(0.2,0.5,1.0);
        return false;
    }
    return false;
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
    glColor3d(1,0,0);
    quadric = gluNewQuadric();
    gluSphere( quadric , 0.01 , 10 , 10);
    glPushMatrix();
    glColor3d(0,1,0);


    glDisable (GL_CULL_FACE);
    for(unsigned int i=0;i<point.size();i++) {
        if (i == picked) glColor3d(1.0,0.0,0.0);
        else glColor3d(1.0,1.0,0.0);
        glPushMatrix();
        glTranslated(point[i][0],point[i][1],point[i][2]);

        feature = simplex;
        Vector3d dir, color;
        simplexSolver(Vector3d(point[i][0], point[i][1], point[i][2]), feature, dir, color);
        int num = feature.size();

        gluQuadricDrawStyle(quadric, GLU_FILL);

        if(num == VoronoiCellSize){
            glColor3d(color[0],color[1],color[2]);
            gluSphere( quadric , .01 , 10 , 10);
        }
        glPopMatrix();
    }

    //Calculate triangular normals

    if(simplex.size()==3){
        n_abc=(simplex[1]-simplex[0])%(simplex[2]-simplex[0]);
        n_abc.normalize();
    }

    if(simplex.size()==4){
        n_abc=(simplex[1]-simplex[0])%(simplex[2]-simplex[0]);
        n_bad=(simplex[3]-simplex[0])%(simplex[1]-simplex[0]);
        n_bdc=(simplex[3]-simplex[1])%(simplex[2]-simplex[1]);
        n_dac=(simplex[0]-simplex[3])%(simplex[2]-simplex[3]);
        n_abc.normalize();
        n_bad.normalize();
        n_bdc.normalize();
        n_dac.normalize();
    }

    //Draw triangular normals

    if(simplex.size()==3){
        Vector3d coM=com(simplex[0],simplex[1],simplex[2]);
        Vector3d normal=(coM+n_abc*0.75);
        glColor3d(1,0,0);
        glBegin(GL_LINES);
        glVertex3dv(coM.ptr());
        glVertex3dv(normal.ptr());
        glEnd();
    }
    if(simplex.size()==4){
        Vector3d com1=com(simplex[0],simplex[1],simplex[2]);
        Vector3d normal1=(com1+n_abc*0.75);
        Vector3d com2=com(simplex[0],simplex[1],simplex[3]);
        Vector3d normal2=(com2+n_bad*0.75);
        Vector3d com3=com(simplex[0],simplex[3],simplex[2]);
        Vector3d normal3=(com3+n_dac*0.75);
        Vector3d com4=com(simplex[3],simplex[1],simplex[2]);
        Vector3d normal4=(com4+n_bdc*0.75);
        glColor3d(1,0,0);
        glBegin(GL_LINES);
        glVertex3dv(com1.ptr());
        glVertex3dv(normal1.ptr());
        glVertex3dv(com2.ptr());
        glVertex3dv(normal2.ptr());
        glVertex3dv(com3.ptr());
        glVertex3dv(normal3.ptr());
        glVertex3dv(com4.ptr());
        glVertex3dv(normal4.ptr());
        glEnd();
    }
    //End triangular normals

    glColor3d(1,0,0);
    glBegin(GL_LINES);
    for(unsigned int i = 0; i < simplex.size()-1; i++){
        for(unsigned int j = i+1; j < simplex.size(); j++){
            Vector3d a(simplex[i][0], simplex[i][1], simplex[i][2]);
            Vector3d b(simplex[j][0], simplex[j][1], simplex[j][2]);
            glVertex3dv(a.ptr());
            glVertex3dv(b.ptr());
        }
    }
    glEnd();
}

void CGView::resizeGL(int width, int height) {
    glViewport(0,0,width,height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (width > height) {
        double ratio = width/(double) height;
        gluPerspective(45,ratio,1.0,100.0);
        //glOrtho(-1,1,-1,1,-1,1);
    }
    else {
        double ratio = height/(double) width;
        gluPerspective(45,1.0/ratio,1.0,100.0);
        //glOrtho(-1,1,-1,1,-1,1);
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
    switch (event->key()) {
    case Qt::Key_Space :
        randomSimplex();
        break;
    case Qt::Key_X :
        VoronoiCellSize = (VoronoiCellSize + 1)%5;
        std::cout << "VoronoiCellSize: " << VoronoiCellSize << std::endl;
        break;
    case Qt::Key_Y :
        VoronoiCellSize = (VoronoiCellSize - 1)%5;
        if ( VoronoiCellSize < 0)
            VoronoiCellSize+=5;
        std::cout << "VoronoiCellSize: " << VoronoiCellSize << std::endl;
        break;
    }
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
