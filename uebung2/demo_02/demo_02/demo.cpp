#include <QApplication>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QGLWidget>
#include <QKeyEvent>
#include <QMessageBox>
#include <QTextEdit>
#include <QHBoxLayout>
#include "demo.h"
//#include "BB.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <limits>

#ifdef max
#undef max
#endif

float angleX=0, angleY=0, angleZ=0;
float xcoord=0, ycoord=0, zcoord=0;


CGMainWindow::CGMainWindow (QWidget* parent, Qt::WindowFlags flags)
    : QMainWindow (parent, flags) {
    resize (1024, 768);
    setWindowState(Qt::WindowMaximized);

    // Create a menu
    QMenu *file = new QMenu("&File",this);
    file->addAction ("Load Polyhedron", this, SLOT(loadPolyhedron()), Qt::CTRL+Qt::Key_G);
    file->addAction ("Quit", qApp, SLOT(quit()), Qt::CTRL+Qt::Key_Q);
    menuBar()->addMenu(file);

    // Create a nice frame to put around the OpenGL widget
    QFrame* f = new QFrame (this);
    f->setFrameStyle(QFrame::Sunken | QFrame::Panel);
    f->setLineWidth(2);

    // Create our OpenGL widget
    ogl = new CGView (this,f);
    ogl->setFocus();

    // Put the GL widget inside the frame
    QHBoxLayout* layout = new QHBoxLayout();
    layout->addWidget(ogl);
    layout->setMargin(0);
    f->setLayout(layout);

    setCentralWidget(f);

    statusBar()->showMessage("Ready",1000);
}

CGMainWindow::~CGMainWindow () {}




void CGMainWindow::loadPolyhedron() {
    QString filename = QFileDialog::getOpenFileName(this, "Load generator model ...", QString(), "OFF files (*.off)" );

    if (filename.isEmpty()) return;
    statusBar()->showMessage ("Loading model ...");
    std::ifstream file(filename.toLatin1());
    //int vn,fn,en;

    ogl->min = +std::numeric_limits<double>::max();
    ogl->max = -std::numeric_limits<double>::max();

    std::string s;
    file >> s;

    if(ogl->P1.empty()){
        file >> ogl->vn >> ogl->fn >> ogl->en;
        std::cout << "model loaded"<< std::endl;
        std::cout << "number of vertices : " << ogl->vn << std::endl;
        std::cout << "number of faces    : " << ogl->fn << std::endl;
        std::cout << "number of edges    : " << ogl->en << std::endl;

        ogl->P1.resize(ogl->vn);

        for(int i=0;i<ogl->vn;i++) {
            file >> ogl->P1[i][0] >> ogl->P1[i][1] >> ogl->P1[i][2];
        }

        ogl->ind1.resize(ogl->fn*3);

        for(int i=0;i<ogl->fn;i++) {
            int k;
            file >> k;
            file >> ogl->ind1[3*i];
            file >> ogl->ind1[3*i+1];
            file >> ogl->ind1[3*i+2];


        }
    } else{
        file >> ogl->vn2 >> ogl->fn2 >> ogl->en2;
        std::cout << "model loaded"<< std::endl;
        std::cout << "number of vertices : " << ogl->vn2 << std::endl;
        std::cout << "number of faces    : " << ogl->fn2 << std::endl;
        std::cout << "number of edges    : " << ogl->en2 << std::endl;

        ogl->P2.resize(ogl->vn2);

        std::cout << "~~~~~ Tasten zum Bewegen des zweiten Objekts ~~~~~\n Translationen:\n Q/W == Links/Rechts \n A/S == Oben/Unten \n"
                     " X/Y == Vorne/Hinten \n Rotationen: \n Pfeiltasten und Bildtasten"  << std::endl;

        for(int i=0;i<ogl->vn2;i++) {
            file >> ogl->P2[i][0] >> ogl->P2[i][1] >> ogl->P2[i][2];
        }

        ogl->ind2.resize(ogl->fn2*3);

        for(int i=0;i<ogl->fn2;i++) {
            int k;
            file >> k;
            file >> ogl->ind2[3*i];
            file >> ogl->ind2[3*i+1];
            file >> ogl->ind2[3*i+2];
        }

        //calculate center of mass for 2. object for translation etc
        Vector3d sumAllVectors2=Vector3d(0,0,0);
        for(int i=0;i<ogl->vn2;i++) {
            sumAllVectors2 = sumAllVectors2 + ogl->P2[i];
        }
        ogl->centerOfMass2=sumAllVectors2/ogl->vn2;

        //translate center of mass of object 2 to origin
        for(int i=0;i<ogl->vn2;i++) {
            for(int j=0; j<3; j++){
                ogl->P2[i][j]= ogl->P2[i][j]-ogl->centerOfMass2[j];
            }
        }
    }



    file.close();

    //calculate center of mass for 1. object
    Vector3d sumAllVectors=Vector3d(0,0,0);
    for(int i=0;i<ogl->vn;i++) {
        sumAllVectors = sumAllVectors + ogl->P1[i];
    }

    ogl->centerOfMass=sumAllVectors/ogl->vn;
    //translate center of mass of object 1 to origin (for convenience)
    for(int i=0;i<ogl->vn;i++) {
        for(int j=0; j<3; j++){
            ogl->P1[i][j]= ogl->P1[i][j]-ogl->centerOfMass[j];
        }
    }
    ogl->center = Vector3d(0,0,0);


    //making zoom dependant on longest side of bounding box
    AABB huelle(ogl->P1);
    float xSide=std::abs(huelle.xmax-huelle.xmin);
    float ySide=std::abs(huelle.ymax-huelle.ymin);
    float zSide=std::abs(huelle.zmax-huelle.zmin);
    float longestSide=xSide;

    if(ySide>xSide){
        longestSide=ySide;
    } else if (zSide>longestSide){
        longestSide=zSide;
    }

     ogl->zoom = 1/longestSide;

    ogl->updateGL();
    statusBar()->showMessage ("Loading generator model done." ,3000);
}


CGView::CGView (CGMainWindow *mainwindow,QWidget* parent ) : QGLWidget (parent) {
    main = mainwindow;
}


void CGView::drawMesh(){

    if(!P1.empty()){
        glColor3d(0,0,1);
        Vector3d n1;
        glBegin(GL_TRIANGLES);
        for(unsigned int i=0;i<ind1.size();i+=3) {
            n1.cross((P1[ind1[i+1]]-P1[ind1[i]]),(P1[ind1[i+2]]-P1[ind1[i]]));
            n1.normalize(n1);
            glNormal3dv(n1.ptr());
            glVertex3dv(P1[ind1[i]].ptr());
            glVertex3dv(P1[ind1[i+1]].ptr());
            glVertex3dv(P1[ind1[i+2]].ptr());
        }
        glEnd();

        // Draw bounding box object 1
        AABB huelle(P1);
        huelle.draw(0,1,0);

        if(!P2.empty()){
            //std::cout << "P[0][0] ohne rot: " << P2[0][0] << std::endl;
            //Draw second mesh if vector P2 is nonempty

            //push 2. object into "own coordinate system" for rotation and translation
            glPushMatrix();
            glColor3d(1,0,0);
            Vector3d n2;
            glTranslatef(xcoord,ycoord,zcoord);
            glRotatef(angleX, 1, 0, 0);
            glRotatef(angleY, 0, 1, 0);
            glRotatef(angleZ, 0, 0, 1);

            glBegin(GL_TRIANGLES);
            for(unsigned int i=0;i<ind2.size();i+=3) {
                n2.cross((P2[ind2[i+1]]-P2[ind2[i]]),(P2[ind2[i+2]]-P2[ind2[i]]));
                n2.normalize(n2);
                glNormal3dv(n2.ptr());
                glVertex3dv(P2[ind2[i]].ptr());
                glVertex3dv(P2[ind2[i+1]].ptr());
                glVertex3dv(P2[ind2[i+2]].ptr());
            }

            glEnd();

            AABB huelle2(P2);
            huelle2.draw(0,1,0);

            if(huelle.intersect(huelle2)){
                huelle2.draw(1,0,0);
            } else {
                huelle2.draw(0,1,0);
            }
            glPopMatrix();


            if(huelle.intersect(huelle2)){
                huelle.draw(1,0,0);
            } else {
                huelle.draw(0,1,0);
            }
        }
    }
    //draw coordinate axes

    glBegin(GL_LINES);
    glColor3d(0, 0, 0);
    glVertex3d(100,0,0);
    glVertex3d(-100,0,0);
    glVertex3d(0,100,0);
    glVertex3d(0,-100,0);
    glVertex3d(0,0,100);
    glVertex3d(0,0,-100);
    glEnd();
}

void CGView::drawOBB(){
    glPushMatrix();
    glColor3d(1,0,0);

    glPopMatrix();
}

void CGView::initializeGL() {
    qglClearColor(Qt::white);
    zoom = 1.0;
    center = 0.0;
    q_now = Quat4d(0.0,0.0,0.0,1.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glEnable(GL_NORMALIZE);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    float pos[4] = {0,2,2,1};
    glLightfv(GL_LIGHT0,GL_POSITION, pos);

}

void CGView::paintGL() {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0,0,-2);

    Matrix4d R(q_now);
    Matrix4d RT = R.transpose();
    glMultMatrixd(RT.ptr());

    glScaled(zoom,zoom,zoom);
    glTranslated(-center[0],-center[1],-center[2]);

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    drawMesh();

}

void CGView::resizeGL(int width, int height) {
    glViewport(0,0,width,height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (width > height) {
        double ratio = width/(double) height;
        //glOrtho(-ratio,ratio,-1.0,1.0,-10.0,10.0);
        //glFrustum(-ratio,ratio,-1.0,1.0,1.0,10.0);
        gluPerspective(60,ratio,0.1,100.0);
        //gluLookAt(0,0,-100,0,0,0,0,1,0);
    }
    else {
        double ratio = height/(double) width;
        //glOrtho(-1.0,1.0,-ratio,ratio,-10.0,10.0);
        //glFrustum(-1.0,1.0,-ratio,ratio,1.0,10.0);
        gluPerspective(60,1/ratio,0.1,100.0);
        //		glFrustum(-1.0,1.0,-ratio,ratio,1.0,100.0);
        //		gluPerspective(fov,1.0/ratio,1.0,10000.0);
        //gluLookAt(0,0,-1,0,0,1,0,1,0);
    }
    glMatrixMode (GL_MODELVIEW);
}


void CGView::keyPressEvent( QKeyEvent * event) {

    float dx,dy,dz;
    dx = dy = dz = 0.01f;

    switch (event->key()) {
    case Qt::Key_Q :{
        xcoord+=dx;
        update();
        break;}
    case Qt::Key_W :{
        xcoord-=dx;
        update();
        break;}
    case Qt::Key_A : {
        ycoord+=dy;
        update();
        break;}
    case Qt::Key_S : {
        ycoord-=dy;
        update();
        break;}
    case Qt::Key_Y : {
        zcoord+=dz;
        update();
        break;}
    case Qt::Key_X :
        zcoord-=dz;
        update();
        break;
    case Qt::Key_Up :
        angleX-=2;
        updateGL();
        break;
    case Qt::Key_Down :
        angleX+=2;
        updateGL();
        break;
    case Qt::Key_Right :
        angleY+=2;
        updateGL();
        break;
    case Qt::Key_Left :
        angleY-=2;
        updateGL();
        break;
    case Qt::Key_PageUp :
        angleZ+=2;
        updateGL();
        break;
    case Qt::Key_PageDown :
       angleZ-=2;
        updateGL();
        break;
    }
}



void CGView::mousePressEvent(QMouseEvent *event) {
    oldX = event->x();
    oldY = event->y();
    if( event->button() == Qt::LeftButton )
    {
        buttonState = 0;

    }
    else if( event->button() == Qt::RightButton )
    {
        buttonState = 1;
    }
    else
        buttonState = 2;

}

void CGView::mouseReleaseEvent(QMouseEvent*) {}

void CGView::wheelEvent(QWheelEvent* event) {
    if (event->delta() < 0) zoom *= 1.2; else zoom *= 1/1.2;
    update();
}

void CGView::mouseMoveEvent(QMouseEvent* event) {

    //if( event->button() == Qt::LeftButton )
    if( buttonState == 0 )	/// rotate camera
    {
        Vector3d p1,p2;

        mouseToTrackball(oldX,oldY,width(),height(),p1);
        mouseToTrackball(event->x(),event->y(),width(),height(),p2);

        Quat4d q = trackball(p1,p2);
        q_now = q * q_now;
        q_now.normalize();
    }
    else if( buttonState == 1 ) /// translate object
    {
        float dx,dy;
        dx = dy = .001f;
        Matrix4d R(q_now);
        Matrix4d RT = R.transpose();

        Vector3d v(0,0,0);
        v[0] = -dx*(event->x() - oldX);
        v[1] =  dy*(event->y() - oldY);
        v = RT*v;

        center += v;
        //cRef -= v;
    }


    oldX = event->x();
    oldY = event->y();

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

