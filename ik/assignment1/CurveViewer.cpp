#include <string>
#include "CurveViewer.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include <algorithm>
#include <AntTweakBar.h>

#pragma warning(disable : 4018)

static CurveViewer* theInstance = 0;

CurveViewer::CurveViewer() : mMode(ADD), mShowControlPoints(false)
{

}


CurveViewer::~CurveViewer()
{
    TwTerminate();
}

void CurveViewer::init(int argc, char** argv, int winwidth, int winheight, int winstartx, int winstarty)
{
    mWindowWidth = winwidth;
    mWindowHeight = winheight;

    theInstance = this;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(winwidth, winheight);
    glutInitWindowPosition(winstartx, winstarty);
    glutCreateWindow("Basic Viewer");

    const bool lSupportVBO = initializeOpenGL();

    glutDisplayFunc(CurveViewer::onDrawCb);
    glutKeyboardFunc(CurveViewer::onKeyboardCb);
    glutSpecialFunc(CurveViewer::onKeyboardSpecialCb);
    glutMouseFunc(CurveViewer::onMouseCb);
    glutMotionFunc(CurveViewer::onMouseMotionCb);
    glutReshapeFunc(CurveViewer::onResizeCb);
    glutTimerFunc(100, CurveViewer::onTimerCb, 0);

    initializeGui();
	mClick = 0;
}

bool CurveViewer::initializeOpenGL()
{
    // Initialize GLEW.
    GLenum lError = glewInit();
    if (lError != GLEW_OK)
    {
        std::cout << "GLEW Error: %s\n" << glewGetErrorString(lError);
        return false;
    }

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glClearColor(0.2, 0.2, 0.2, 1.0);

    // OpenGL 1.5 at least.
    if (!GLEW_VERSION_1_5)
    {
        std::cout << "The OpenGL version should be at least 1.5 to display shaded scene!\n";
        return false;
    }

    glEnable(GL_BLEND);
    glEnable(GL_ALPHA_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glShadeModel(GL_SMOOTH);

    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);

    glClearStencil(0); //clear the stencil buffer
    glClearDepth(1.0f);

    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glEnable(GL_MULTISAMPLE);
    return true;
}

void CurveViewer::run()
{
    glutMainLoop();
}


void CurveViewer::onTimerCb(int value)
{
    theInstance->onTimer(value);
}

void CurveViewer::onTimer(int value)
{
    glutTimerFunc(100, onTimerCb, 0);
    glutPostRedisplay(); // Needed for GUI to update
}

void CurveViewer::onDrawCb()
{
    theInstance->onDraw();
}
void CurveViewer::onDraw()
{
    drawCurveView();
    TwDraw();  // gui
    glutSwapBuffers();
}

void CurveViewer::drawCurveView()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, mWindowWidth, 0, mWindowHeight);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    DrawCurve();

    glPopAttrib();
}

void CurveViewer::DrawCircle(const vec3& p)
{
    int vertices = 20;
    double r = 5.0;
    double tmpX, tmpY, tmpZ;
    double Angle, Angle0;

    Angle = -(2 * 3.14) / vertices;
    Angle0 = 0.0;

    tmpX = p[0];
    tmpY = p[1];
    tmpZ = p[2];

    glBegin(GL_POLYGON);

    for (int i = 0; i < vertices; i++) {
        glVertex3f(tmpX + r * cos(i*Angle + Angle0), tmpY + r * sin(i*Angle + Angle0), tmpZ);
    }
    glEnd();
}


void CurveViewer::DrawCurve()
{
    if (mSpline.getNumKeys() < 1) return;

    ASplineVec3::InterpolationType type = mSpline.getInterpolationType();
    if (type != ASplineVec3::LINEAR && mShowControlPoints && mSpline.getNumKeys() > 1)
    {
        glColor3f(1.0, 1.0, 0.0);
        if (type == ASplineVec3::CUBIC_HERMITE)
        {
            for (int i = 0; i < mSpline.getNumKeys(); i++)
            {
                DrawCircle(mSpline.getKey(i) + mSpline.getControlPoint(i+1));
            }

            glBegin(GL_LINES);
            for (int i = 0; i < mSpline.getNumKeys(); i++)
            {
                vec3 pt = mSpline.getKey(i);
                vec3 ctrl = pt + mSpline.getControlPoint(i+1);
                glVertex3d(pt[0], pt[1], pt[2]);
                glVertex3d(ctrl[0], ctrl[1], ctrl[2]);
            }
            glEnd();
        }
        else
        {
            for (int i = 0; i < mSpline.getNumControlPoints(); i++)
            {
                if (type == ASplineVec3::CUBIC_BSPLINE && (i == 0 || i == mSpline.getNumControlPoints() - 1)) continue;
                vec3 pt = mSpline.getControlPoint(i);
                DrawCircle(pt);
            }

            glBegin(GL_LINES);
            for (int i = 0; i < mSpline.getNumControlPoints() - 1; i++)
            {
                if (type == ASplineVec3::CUBIC_BSPLINE && (i == 0 || i == mSpline.getNumControlPoints() - 2)) continue;
                vec3 pt = mSpline.getControlPoint(i);
                vec3 nextPt = mSpline.getControlPoint(i + 1);
                glVertex2d(pt[0], pt[1]);
                glVertex2d(nextPt[0], nextPt[1]);
            }
            glEnd();
        }
    }

    glColor3f(0.0, 0.0, 1.0);
    for (int i = 0; i < mSpline.getNumKeys(); i++)
    {
        vec3 pt = mSpline.getKey(i);
        DrawCircle(pt);
    }

    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
    for (int i = 0; i < mSpline.getNumCurveSegments()-1; i++)
    {
        vec3 pt = mSpline.getCurvePoint(i);
        vec3 nextPt = mSpline.getCurvePoint(i + 1);
        glVertex2d(pt[0], pt[1]);
        glVertex2d(nextPt[0], nextPt[1]);
    }
    glEnd();
}

void CurveViewer::initializeGui()
{
    glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
    TwGLUTModifiersFunc(glutGetModifiers);
    TwInit(TW_OPENGL, NULL);
    TwWindowSize(mWindowWidth, mWindowHeight);

    TwCopyStdStringToClientFunc(onCopyStdStringToClient);

    mCurveBar = TwNewBar("Curve controls");
    TwDefine(" 'Curve controls' size='200 175' position='5 5' iconified=false fontresizable=false alpha=200");
    TwEnumVal curveTypeEV[] = 
    { 
		{ ASplineVec3::LINEAR, "Linear" },
		{ ASplineVec3::CUBIC_BERNSTEIN, "Berstein-Bezier" },
		{ ASplineVec3::CUBIC_CASTELJAU, "Casteljau-Bezier" },
		{ ASplineVec3::CUBIC_MATRIX, "Matrix-Bezier" },
		{ ASplineVec3::CUBIC_HERMITE, "Hermite" },
		{ ASplineVec3::CUBIC_BSPLINE, "BSpline" }
    };
    curveType = TwDefineEnum("CurveType", curveTypeEV, 6);
    TwAddVarCB(mCurveBar, "Type", curveType, onSetStyleCb, onGetStyleCb, this, " ");

    TwEnumVal modeTypeEV[] = { { ADD, "Add" }, { EDIT, "Edit" }, { REMOVE, "Delete" } };
    modeType = TwDefineEnum("ModeType", modeTypeEV, 3);
    TwAddVarRW(mCurveBar, "Mode", modeType, &mMode, NULL);

    TwAddVarRW(mCurveBar, "Ctrl pts", TW_TYPE_BOOLCPP, &mShowControlPoints, "");
    TwAddButton(mCurveBar, "ClearBtn", onClearCb, this, " label='Clear'");
}

void CurveViewer::onMouseMotionCb(int x, int y)
{
    theInstance->onMouseMotion(x, y);
}
void CurveViewer::onMouseMotion(int pX, int pY)
{
    // Check GUI first
    if (TwEventMouseMotionGLUT(pX, pY)) return;

    int deltaX = mLastX - pX;
    int deltaY = mLastY - pY;
    bool moveLeftRight = abs(deltaX) > abs(deltaY);
    bool moveUpDown = !moveLeftRight;
    if (mMode == EDIT && mButtonState == GLUT_LEFT_BUTTON)
    {
        movePicked(pX, mWindowHeight - pY);
        glutPostRedisplay();
    }

    mLastX = pX;
    mLastY = pY;

    glutPostRedisplay();
}

void CurveViewer::movePicked(int x, int y)
{
    if (mSelected != -1)
    {
        vec3 temp(x, y, 0);        
        if (mPickType == 0) mSpline.editKey(mSelected, temp);
        else mSpline.editControlPoint(mSelected, temp);
    }
}

void CurveViewer::addPoint(int x, int y)
{
    vec3 tmp(x, y, 0);
    // guard against adding multiple copies of the same point
    // (multiple mouse events may be triggered for the same point)
    if (mSpline.getNumKeys() > 0) 
    {
        vec3 lastKey = mSpline.getKey(mSpline.getNumKeys() - 1);
        if ((lastKey-tmp).Length() < 1.0) return;
    }

    mSpline.appendKey(tmp);
}

void CurveViewer::deletePoint(int x, int y)
{
    float radius = 5.0;
    mSelected = -1;
    vec3 tmp = vec3(x, y, 0);

    // check data points -- needs to be done before ctrl points
    for (int i = 0; i < mSpline.getNumKeys(); i++)
    {
        vec3 pt = mSpline.getKey(i);
        if ((tmp - pt).Length() < radius)
        {
            mSpline.deleteKey(i);
            break;
        }
    }
}


void CurveViewer::pickPoint(int x, int y)
{
    float radius = 5.0;
    mSelected = -1;
    vec3 tmp = vec3(x, y, 0);

    // check data points -- needs to be done before ctrl points
    for (int i = 0; mSelected == -1 && i < mSpline.getNumKeys(); i++)
    {
        vec3 pt = mSpline.getKey(i);
        if ((tmp - pt).Length() < radius)
        {
            mPickType = 0;
            mSelected = i;
        }
    }

    // Check control points
    for (int i = 0; mSelected == -1 && i < mSpline.getNumControlPoints(); i++)
    {
        vec3 pt = mSpline.getControlPoint(i);
        if ((tmp - pt).Length() < radius)
        {
            mPickType = 1;
            mSelected = i;
        }
    }
}

void CurveViewer::onMouseCb(int button, int state, int x, int y)
{
    theInstance->onMouse(button, state, x, y);
}
void CurveViewer::onMouse(int pButton, int pState, int pX, int pY)
{
    // Check GUI first
    if (TwEventMouseButtonGLUT(pButton, pState, pX, pY))  return;

    mButtonState = pButton;
    mModifierState = glutGetModifiers();
    mLastX = pX;
    mLastY = pY;


	if (mButtonState == GLUT_LEFT_BUTTON)
		if (pState == GLUT_DOWN)
			 mClick++;
		else mClick = 0; // GLUT_UP
	

	if (mClick == 1) 
	{
		if (mMode == ADD)
		{
			addPoint(pX, mWindowHeight - pY);
			glutPostRedisplay();
		}
		else if (mMode == REMOVE)
		{
			deletePoint(pX, mWindowHeight - pY);
			glutPostRedisplay();
		}
		else
		{
			pickPoint(pX, mWindowHeight - pY);
			onMouseMotionCb(pX, pY);
		}
	}
    

}

void CurveViewer::onKeyboardSpecialCb(int key, int x, int y)
{
    theInstance->onKeyboardSpecial(key, x, y);
}
void CurveViewer::onKeyboardSpecial(int key, int x, int y)
{
    TwEventSpecialGLUT(key, x, y);
}

void CurveViewer::onKeyboardCb(unsigned char key, int x, int y)
{
    theInstance->onKeyboard(key, x, y);
}

void CurveViewer::onKeyboard(unsigned char pKey, int x, int y)
{
    // Exit on ESC key.
    if (pKey == 27)
    {
        exit(0);
    }

    if (TwEventKeyboardGLUT(pKey, x, y)) return;
}

void CurveViewer::onMenuCb(int value)
{
    theInstance->onMenu(value);
}
void CurveViewer::onMenu(int value)
{
    switch (value)
    {
    case -1: exit(0);
    default: onKeyboardCb(value, 0, 0); break;
    }
}

void CurveViewer::onResizeCb(int width, int height)
{
    theInstance->onResize(width, height);
}
void CurveViewer::onResize(int width, int height)
{
    glViewport(0, 0, (GLsizei)width, (GLsizei)height);
    mWindowWidth = width;
    mWindowHeight = height;
    TwWindowSize(width, height);
}

void CurveViewer::onClearCb(void* usr)
{
    ((CurveViewer*)usr)->mSpline.clear();
}

void TW_CALL CurveViewer::onSetStyleCb(const void *value, void *clientData)
{
    CurveViewer* viewer = ((CurveViewer*)clientData);
    ASplineVec3::InterpolationType v = *(const ASplineVec3::InterpolationType *)value;  // for instance
    viewer->mSpline.setInterpolationType(v);
}

void TW_CALL CurveViewer::onGetStyleCb(void *value, void *clientData)
{
    CurveViewer* viewer = ((CurveViewer*)clientData);
    *static_cast<ASplineVec3::InterpolationType *>(value) = viewer->mSpline.getInterpolationType();
}

void TW_CALL CurveViewer::onCopyStdStringToClient(std::string& dest, const std::string& src)
{
    dest = src;
}