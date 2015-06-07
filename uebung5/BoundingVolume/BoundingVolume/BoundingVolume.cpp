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

#include "Sphere.h"
#include "BoundingVolume.h"

#include "OffReader.h"

#include "BVT.h"

const int numpoints = 100;
static std::vector<Vector3d> point;
static std::vector<int> indices;
static BVT * root;

/// Die Testkugel f"ur Aufgabe 1d!
static Sphere test_kugel;

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

	show_circle=true;
	bbox_on = true;

  Vector3d c = Vector3d(0.0,0.0,0.0);
  Vector3d l = Vector3d(2.0,0.8,0.1);
  Matrix4d R = Matrix4d::rotate(0.5,-1.0,0.4,0.5);
	
	picked = 0;

	/// Um Keyboard-Events durchzulassen
	setFocusPolicy(Qt::StrongFocus);

}

void CGView::initializeGL() {
	glClearColor(1.0,1.0,1.0,1.0);
	center = 0.0;
	q_now = Quat4d(0.0,0.0,0.0,1.0);
	
  glEnable(GL_CULL_FACE);
  glFrontFace(GL_CCW);
  glCullFace(GL_BACK);
  glEnable(GL_DEPTH_TEST);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
}


/// Rekursiver abstieg bis in Tiefe deep zum malen des Huellkoerpers
void CGView::drawBall (BVT * tree)
{
	tree->ball().draw();
}

/// Draws the off file scaled and fitting into the scene
void CGView::drawOff ()
{
	glPushMatrix();
	glDisable (GL_CULL_FACE);
	glBegin(GL_TRIANGLES);
	for(unsigned int i =0; i < indices.size(); i+=3)
	{
		const Vector3d& a =point[indices[i]];
		const Vector3d& b =point[indices[i+1]];
		const Vector3d& c =point[indices[i+2]];
		glNormal3dv (((b-a)%(c-a)).ptr());
		glVertex3dv( a.ptr());
		glVertex3dv( b.ptr());
		glVertex3dv( c.ptr());
	}
	glEnd();
	glPopMatrix();

}

void CGView::paintGL() {
  static float mat_ambient[4] = {0.0, 0.0, 0.0, 0.0};
  static float mat_shininess[1] = {1.0};
  static float mat_specular[4] = {0.0, 0.0, 0.0, 0.0};
  static float mat_diffuse[4] = {0.0, 0.0, 0.0, 0.0};

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

    glTranslated(0.0,0.0,-3.0);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	Matrix4d R(q_now);
	Matrix4d RT = R.transpose();
	glMultMatrixd(RT.ptr());

	double z = 0.5 / root->ball().radius;
	glScaled(z,z,z);
	glTranslated(-root->ball().center.x(), -root->ball().center.y(), -root->ball().center.z());

  glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

  glColor3d(0.0,0.0,0.0);
  glDisable(GL_LIGHTING);

  if (show_circle) 
		{
			drawBall (root);
			test_kugel.draw (Vector3d (0,1,1));
		}

  glEnable(GL_LIGHTING);

  glColor3d(0.0,0.0,1.0);
	drawOff ();

  glDisable(GL_LIGHTING);

}

void CGView::resizeGL(int width, int height) {
	glViewport(0,0,width,height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (width > height) {
		double ratio = width/(double) height;
        gluPerspective(45,ratio,1.0,10000.0);
        //glOrtho(-ratio,ratio,-1.0,1.0,-10.0,10.0);
	}
	else {
		double ratio = height/(double) width;
        gluPerspective(45,1.0/ratio,0.01,10000.0);
        //glOrtho(-1.0,1.0,-ratio,ratio,-10.0,10.0);
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
		case Qt::Key_X     : if (event->modifiers() & Qt::ShiftModifier) test_kugel.center[0] -= 0.05; else test_kugel.center[0] += 0.05; break;
		case Qt::Key_Y     : if (event->modifiers() & Qt::ShiftModifier) test_kugel.center[1] -= 0.05; else test_kugel.center[1] += 0.05; break;
		case Qt::Key_Z     : if (event->modifiers() & Qt::ShiftModifier) test_kugel.center[2] -= 0.05; else test_kugel.center[2] += 0.05; break;
		default: changed=false; break;
	}
		
	updateGL();
}

int main (int argc, char **argv) {
	QApplication app(argc, argv);

	if (!QGLFormat::hasOpenGL()) {
		qWarning ("This system has no OpenGL support. Exiting.");
		return 1;
	}

	if(argc == 2)
	{
		LoadOffFile(argv[1], point, indices);
	}
	else //Default:
	{
		int fail = LoadOffFile("space_station.off", point, indices);
		if (fail<0) return -1;
	}

	/// Wir bauen die Wurzel, mit allen Punkten
	root = new BVT (point);

	/// Bauen wir die Testkugel!
	Vector3d a = root->ball().center;
	double d = root->ball().radius;
	double r = 0.2 * d;
	a += Vector3d (d,d,d);
	Vector3d b = a + Vector3d (r,r,r);

	test_kugel = Sphere (a,b);

	CGMainWindow *w = new CGMainWindow(NULL);

	w->show();

	return app.exec();
}

