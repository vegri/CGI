#ifndef DEMO_H
#define DEMO_H

#include <QMainWindow>
#include <QGLWidget>
#include <QGridLayout>
#include <QFrame>
#include <vector>
#include <iostream>
#include <algorithm>

#if _MSC_VER
    #include <gl/glu.h>
#elif __APPLE__
  #include <OpenGL/glu.h>
#else
    #include <GL/glu.h>
#endif



#include "vecmath.h"

class CGView;

class CGMainWindow : public QMainWindow {
	Q_OBJECT

public:


    CGMainWindow (QWidget* parent = 0, Qt::WindowFlags flags = Qt::Window);
	~CGMainWindow ();
  //  void keyPressEvent(QKeyEvent*);

public slots:

	void loadPolyhedron();

protected:

//    void keyPressEvent(QKeyEvent*);

private:


	CGView *ogl;

};

class CGView : public QGLWidget {
	Q_OBJECT

public:

	CGView(CGMainWindow*,QWidget*);
	void initializeGL();

     Vector3d min, max, center, centerOfMass, centerOfMass2;
	// min, max and center of the coords of the loaded model

	double zoom;
	int buttonState;
	double phi,theta;
    int vn,fn,en,vn2,fn2,en2;
    std::vector<Vector3d> P1, P2;// the coords of the loaded model
    std::vector<int> ind1, ind2;   // the faces of the loaded model,
				// ind[i] ind[i+1] ind[i+2] 
				// contains the indices of the i-th triangle
    Quat4d q_now;


protected:

	void paintGL();
	void drawMesh();
	void resizeGL(int,int);
	void mouseToTrackball(int x, int y, int W, int H, Vector3d &v);
	Quat4d trackball(const Vector3d&, const Vector3d&);

	void mouseMoveEvent(QMouseEvent*);
	void mousePressEvent(QMouseEvent*);
	void mouseReleaseEvent(QMouseEvent*);
	void wheelEvent(QWheelEvent*);
    void keyPressEvent(QKeyEvent*);
    void drawOBB(std::vector<Vector3d>& p);

	CGMainWindow *main;

        int oldX,oldY,oldZ;

};

#endif
