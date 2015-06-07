#ifndef BOUNDINGVOLUME_H
#define BOUNDINGVOLUME_H

#include <QMainWindow>
#include <QGLWidget>
#include <QGridLayout>
#include <QFrame>
#include <QActionGroup>
#include <QMenu>
#include <vector>
#include <iostream>

#include "BVT.h"
#if _MSC_VER
    #include <gl/glu.h>
#elif __APPLE__
  #include <OpenGL/glu.h>
#else
    #include <GL/glu.h>
#endif


#include "vecmath.h"

#ifndef VECMATH_VERSION
#error "wrong vecmath included, must contain a VECMATH_VERSION macro"
#else
#if VECMATH_VERSION < 2
#error "outdatet vecmath included"
#endif
#endif

class CGView;

class CGMainWindow : public QMainWindow {
	Q_OBJECT

public:

    CGMainWindow (QWidget* parent = 0, Qt::WindowFlags flags = Qt::Window);
	~CGMainWindow ();

	QActionGroup *projectionMode;
	QAction *flatShadingForSphere;

private:

	CGView *ogl;

	QAction* addCheckableActionToMenu(QMenu *menu, const QString &text, QObject *receiver, const char *slot, bool initState = false, QKeySequence shortcut = 0) {
		QAction *action = new QAction(text,menu);
		action->setCheckable(true);
		action->setChecked(initState);
		action->setShortcut(shortcut);
		connect(action,SIGNAL(triggered()),receiver,slot);
		menu->addAction(action);
		return action;
	}

};

class CGView : public QGLWidget {
	Q_OBJECT

public:

	CGView(CGMainWindow*,QWidget*);
	void initializeGL();
	
	/** transforms the picture coords (x,y,z) to world coords by 
		inverting the projection and modelview matrix (as it it is 
		after invocation of paintGL) and stores the result in v */
	void worldCoord(int x, int y, int z, Vector3d &v);

    Vector3d min, max, center;	// min, max and center of the coords of the loaded model
	double zoom;
	bool wireframe_on, bbox_on;

	bool show_circle;

	unsigned int picked;

protected:
	void paintGL();
	void resizeGL(int,int);
	void drawBoundingBox();
	void drawBall (BVT * tree);
	void drawOff ();
	void mouseToTrackball(int x, int y, int W, int H, Vector3d &v);
	Quat4d trackball(const Vector3d&, const Vector3d&);

	void mouseMoveEvent(QMouseEvent*);
	void mousePressEvent(QMouseEvent*);
	void mouseReleaseEvent(QMouseEvent*);
	void wheelEvent(QWheelEvent*);
	void keyPressEvent( QKeyEvent * event); 

	CGMainWindow *main;
	int oldX,oldY;
	GLUquadric *quad;
	Quat4d q_now;

	std::vector<Vector3d> points;

public slots:

	void updateProjectionMode() {
		resizeGL(width(),height());
		updateGL();
	}

	void toggleSphere () { show_circle = !show_circle; updateGL(); }
	void toggleBbox() { bbox_on = !bbox_on; updateGL(); }

};



#endif
