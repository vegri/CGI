#ifndef SES_H
#define SES_H

#include <QMainWindow>
#include <QGLWidget>
#include <QGridLayout>
#include <QFrame>
#include <QActionGroup>
#include <QMenu>
#include <vector>
#include <iostream>

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

public slots:

    void loadPolyhedron();

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
    Vector3d n_abc, n_bad, n_bdc, n_dac;
    double zoom;
    bool wireframe_on, bbox_on;

    bool show_circle;
    int vn,fn,en;
    std::vector<Vector3d> P1;

    unsigned int picked;

protected:
    void paintGL();
    void resizeGL(int,int);
    void drawBoundingBox();
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

    std::vector<Vector3d> Q;
    std::vector<Vector3d> feature;

    Vector3d direction;
    Vector3d sphere_center;


    double EPS;
    Vector3d Cen;
    bool isInCol;
    int VoronoiCellSize;


    bool simplexSolver(const Vector3d &p,
                       std::vector<Vector3d> &Q,
                       Vector3d &dir);
    bool voronoiSurface(std::vector<Vector3d> &Q, const Vector3d &p, const Vector3d &a, const Vector3d &b,
                        const Vector3d &c, const Vector3d &normal);
    bool voronoiEdge(std::vector<Vector3d> &Q, const Vector3d &p, const Vector3d &a, const Vector3d &b, const Vector3d &normal1, const Vector3d &normal2);
    bool voronoiEdge(std::vector<Vector3d> &Q, const Vector3d &p, const Vector3d &a, const Vector3d &b);
    bool voronoiPoint(std::vector<Vector3d> &Q, const Vector3d &p, const Vector3d &a, const Vector3d &b,
                      const Vector3d &c, const Vector3d &d);
    bool voronoiPoint(std::vector<Vector3d> &Q, const Vector3d &p, const Vector3d &a, const Vector3d &b,
                      const Vector3d &c);
    Vector3d comTriangle(const Vector3d &a, const Vector3d &b,
                 const Vector3d &c);
    Vector3d support(const Vector3d &d);
    void triangleNormal(const std::vector<Vector3d> &simplex);
    bool GJK();

    void randomSimplex();
    void correctSimplexOrientation(std::vector<Vector3d> & simplex);

public slots:

    void updateProjectionMode() {
        resizeGL(width(),height());
        updateGL();
    }

    void toggleSphere () { show_circle = !show_circle; updateGL(); }
    void toggleBbox() { bbox_on = !bbox_on; updateGL(); }

};



#endif
