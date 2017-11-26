#include <string>
#include "RotationsViewer.h"
#include "aRotation.h"
#include "aTimer.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include <algorithm>
#include <AntTweakBar.h>
#include <iostream>

#pragma warning(disable : 4018)

static RotationsViewer* theInstance = 0;

RotationsViewer::RotationsViewer() : 
mMode(DEMO1), mXAngle(50), mYAngle(0), mZAngle(0), mRotOrder(XYZ)
{
    quat q0, q1, q2;
    q0.FromAxisAngle(vec3(0, 0, 1),  50 * Deg2Rad);
    q1.FromAxisAngle(vec3(0, 0, 1),  -50 * Deg2Rad);
    q2.FromAxisAngle(vec3(0, 1, 0),  90 * Deg2Rad);
    mQuatSpline.appendKey(0, q0);
    mQuatSpline.appendKey(3, q1);
    mQuatSpline.appendKey(6, q2);
    mQuatSpline.appendKey(9, q0);
    mClock = new ATimer();
    mClock->start();
}

RotationsViewer::~RotationsViewer()
{
    delete mClock;
    TwTerminate();
}

void RotationsViewer::init(int argc, char** argv, int winwidth, int winheight, int winstartx, int winstarty)
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

    glutDisplayFunc(RotationsViewer::onDrawCb);
    glutKeyboardFunc(RotationsViewer::onKeyboardCb);
    glutSpecialFunc(RotationsViewer::onKeyboardSpecialCb);
    glutMouseFunc(RotationsViewer::onMouseCb);
    glutMotionFunc(RotationsViewer::onMouseMotionCb);
    glutReshapeFunc(RotationsViewer::onResizeCb);
    glutTimerFunc(100, RotationsViewer::onTimerCb, 0);

    initializeGui();
}

bool RotationsViewer::initializeOpenGL()
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

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glShadeModel(GL_SMOOTH);

    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    glDisable(GL_CULL_FACE);

    // Setup lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    float AmbientColor[] = { 0.01f, 0.01f, 0.01f, 1.0f };   glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientColor);
    float DiffuseColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };   glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseColor);
    float SpecularColor[] = { 0.0f, 0.0f, 0.0f, 1.0f };   glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularColor); 
    float Position[] = { 1.0f, 1.0f, 4.0f, 1.0f };  glLightfv(GL_LIGHT0, GL_POSITION, Position);

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

void RotationsViewer::run()
{
    glutMainLoop();
}

void RotationsViewer::onTimerCb(int value)
{
    theInstance->onTimer(value);
}

void RotationsViewer::onTimer(int value)
{
    glutTimerFunc(100, onTimerCb, 0);
    glutPostRedisplay(); // Needed for GUI to update
}

void RotationsViewer::onDrawCb()
{
    theInstance->onDraw();
}
void RotationsViewer::onDraw()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-5, 5, -5, 5, 0.01, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 0, -10, 0, 0, 0, 0, 1, 0);

    switch (mMode)
    {
    case DEMO1: drawDemo1(); break;
    case DEMO2: drawDemo2(); break;
    }
    TwDraw();  // gui
    glutSwapBuffers();
}

void RotationsViewer::pushEulerRotation(RotationsViewer::RotOrder roo, const vec3& euler)
{
    switch (roo)
    {
	case ZYX:
		glRotatef(euler[2], 0, 0, 1);
		glRotatef(euler[1], 0, 1, 0);
		glRotatef(euler[0], 1, 0, 0);
		break;
	case XYZ:
        glRotatef(euler[0], 1, 0, 0);
        glRotatef(euler[1], 0, 1, 0);
        glRotatef(euler[2], 0, 0, 1);
        break;
	case YZX:
		glRotatef(euler[1], 0, 1, 0);
		glRotatef(euler[2], 0, 0, 1);
		glRotatef(euler[0], 1, 0, 0);
		break;
	case XZY:
        glRotatef(euler[0], 1, 0, 0);
        glRotatef(euler[2], 0, 0, 1);
        glRotatef(euler[1], 0, 1, 0);
        break;
    case YXZ: 
        glRotatef(euler[1], 0, 1, 0);
        glRotatef(euler[0], 1, 0, 0);
        glRotatef(euler[2], 0, 0, 1);
        break;
    case ZXY: 
        glRotatef(euler[2], 0, 0, 1);
        glRotatef(euler[0], 1, 0, 0);
        glRotatef(euler[1], 0, 1, 0);
        break;

    }
}

void RotationsViewer::drawDemo1()
{
    // test euler to rotation matrix convernsions
    mat3 rot;
    vec3 euler(mXAngle, mYAngle, mZAngle);
    switch (mRotOrder)
    {
	case ZYX: rot.FromEulerAngles(mat3::ZYX, euler*Deg2Rad); break;
	case XYZ: rot.FromEulerAngles(mat3::XYZ, euler*Deg2Rad); break;
	case YZX: rot.FromEulerAngles(mat3::YZX, euler*Deg2Rad); break;
	case XZY: rot.FromEulerAngles(mat3::XZY, euler*Deg2Rad); break;
	case YXZ: rot.FromEulerAngles(mat3::YXZ, euler*Deg2Rad); break;
	case ZXY: rot.FromEulerAngles(mat3::ZXY, euler*Deg2Rad); break;

    }

    GLfloat GLmat[16];
	rot.WriteToGLMatrix(GLmat);
    vec3 anglesXYZ, anglesXZY, anglesYXZ, anglesYZX, anglesZXY, anglesZYX;

	rot.ToEulerAngles(mat3::XYZ, anglesXYZ);
	rot.ToEulerAngles(mat3::XZY, anglesXZY);
	rot.ToEulerAngles(mat3::YXZ, anglesYXZ);
	rot.ToEulerAngles(mat3::YZX, anglesYZX);
	rot.ToEulerAngles(mat3::ZXY, anglesZXY);
	rot.ToEulerAngles(mat3::ZYX, anglesZYX);

    anglesXYZ = anglesXYZ * Rad2Deg;
    anglesXZY = anglesXZY * Rad2Deg;
    anglesYXZ = anglesYXZ * Rad2Deg;
    anglesYZX = anglesYZX * Rad2Deg;
    anglesZXY = anglesZXY * Rad2Deg;
    anglesZYX = anglesZYX * Rad2Deg;

    quat q;
    q.FromRotation(rot);
    vec3 axis; double angle;
    q.ToAxisAngle(axis, angle);
    angle = angle * Rad2Deg;

    mat3 qmat = q.ToRotation();
    GLfloat GLqmat[16];
	qmat.WriteToGLMatrix(GLqmat);

    // Teapot 0.  OPenGL-based reference for Euler angles and rotation order selected from GUI
    glColor3f(1, 0, 0); // red
    glPushMatrix();
    glTranslatef(-3, 3, 0);
    pushEulerRotation(mRotOrder, euler);
    glutSolidTeapot(0.5);
    glPopMatrix();

    // Teapot 1.  Rotation Matrix (rot) computed using fromEulerAngles function above with the selected rotation order
    
	glColor3f(0.66, 0.66, 1);  // light blue 
    glPushMatrix();
	glTranslatef(1.75, 1.75, 0); 
    glMultMatrixf(GLmat);
    glutSolidTeapot(0.5);
    glPopMatrix();
    
    // Teapot 2.  Used to verify conversion of rotation matrix to EULER ZYX
	glColor3f(0.25, 0.25, 1);  // medium blue 
	glPushMatrix();
	glTranslatef(1.75, 0, 0); 
    pushEulerRotation(ZYX, anglesZYX);
    glutSolidTeapot(0.5);
    glPopMatrix();

    // Teapot 3.  Used to verify conversion of rotation matrix to EULER XZY
	glColor3f(0, 0, 1); //blue
	glPushMatrix();
	glTranslatef(1.75, -1.75, 0);
    pushEulerRotation(XZY, anglesXZY);
    glutSolidTeapot(0.5);
    glPopMatrix();

	// Teapot 4.  Used to verify conversion of  rotation matrix to EULER XYZ
	glColor3f(1, 1, 1);  // white 
	glPushMatrix();
	glTranslatef(0, 1.75, 0);
    pushEulerRotation(XYZ, anglesXYZ);
    glutSolidTeapot(0.5);
    glPopMatrix();

    // Teapot 5.   Used to verify conversion of  rotation matrix to EULER YZX
	glColor3f(0.6, 0.6, 0.6); // medium gray
	glPushMatrix();
	glTranslatef(0, 0, 0);
    pushEulerRotation(YZX, anglesYZX);
    glutSolidTeapot(0.5);
    glPopMatrix();

    // Teapot 6.   Used to verify conversion of  rotation matrix to EULER ZXY
	glColor3f(0.25, 0.25, 0.25); // dark gray  
	glPushMatrix();
    glTranslatef(0, -1.75, 0);
    pushEulerRotation(ZXY, anglesZXY);
    glutSolidTeapot(0.5);
    glPopMatrix();


	// Teapot 7.   Used to verify conversion of  rotation matrix to EULER YXZ
	glColor3f(0.75, 1, 0.75);  //light green 	
	glPushMatrix();
	glTranslatef(-1.75, 1.75, 0);	
    pushEulerRotation(YXZ, anglesYXZ);
    glutSolidTeapot(0.5);
    glPopMatrix();

    // Teapot 8.   Used to verify conversion of quaternion to angle/axis
	glColor3f(0.33, 1, 0.33);  // medium green
	glPushMatrix();
	glTranslatef(-1.75, 0, 0);
    glRotatef(angle, axis[0], axis[1], axis[2]);
    glutSolidTeapot(0.5);
    glPopMatrix();
    
    // Teatpot 9.   Used to verify conversion of quaternion to rotation matrix
	glColor3f(0, 1, 0); // green
	glPushMatrix();
	glTranslatef(-1.75, -1.75, 0);    
    glMultMatrixf(GLqmat);
    glutSolidTeapot(0.5);
    glPopMatrix();
}

void RotationsViewer::drawDemo2()
{
    double time = mClock->totalElapsedTime();
    quat q = mQuatSpline.getCachedValue(time);
    double angle;
    vec3 axis;
    q.ToAxisAngle(axis, angle);

    glColor3f(1, 1, 0);
    glPushMatrix();
    glRotatef(angle * Rad2Deg, axis[0], axis[1], axis[2]);
    glutSolidTeapot(1.5);
    glPopMatrix();
}

void RotationsViewer::initializeGui()
{
    glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
    TwGLUTModifiersFunc(glutGetModifiers);
    TwInit(TW_OPENGL, NULL);
    TwWindowSize(mWindowWidth, mWindowHeight);

    TwCopyStdStringToClientFunc(onCopyStdStringToClient);

    mDemoBar = TwNewBar("Demo controls");
    TwDefine(" 'Demo controls' size='200 175' position='5 5' iconified=false fontresizable=false alpha=200");

    TwEnumVal modeTypeEV[] = { { DEMO1, "Convert" }, { DEMO2, "Curve" } };
    modeType = TwDefineEnum("ModeType", modeTypeEV, 2);
    TwAddVarRW(mDemoBar, "Demo", modeType, &mMode, NULL);

    TwAddVarRW(mDemoBar, "X Angle", TW_TYPE_DOUBLE, &mXAngle, " group='Convert params' ");
    TwAddVarRW(mDemoBar, "Y Angle", TW_TYPE_DOUBLE, &mYAngle, " group='Convert params' ");
    TwAddVarRW(mDemoBar, "Z Angle", TW_TYPE_DOUBLE, &mZAngle, " group='Convert params' ");
    TwEnumVal rooTypeEV[] = { { XYZ, "XYZ" }, { XZY, "XZY" }, { YXZ, "YXZ" }, { YZX, "YZX" }, { ZXY, "ZXY" }, { ZYX, "ZYX" } };
    rooType = TwDefineEnum("RooType", rooTypeEV, 6);
    TwAddVarRW(mDemoBar, "Rot Order", rooType, &mRotOrder, " group='Convert params' ");

    TwEnumVal splineTypeEV[] = { { ASplineQuat::LINEAR, "Linear" }, { ASplineQuat::CUBIC, "Cubic" } };
    splineType = TwDefineEnum("SplineType", splineTypeEV, 2);
    TwAddVarCB(mDemoBar, "Spline type", splineType, onSetStyleCb, onGetStyleCb, this, " group='Curve params'");
}

void RotationsViewer::onMouseMotionCb(int x, int y)
{
    theInstance->onMouseMotion(x, y);
}
void RotationsViewer::onMouseMotion(int pX, int pY)
{
    // Check GUI first
    if (TwEventMouseMotionGLUT(pX, pY)) return;

    int deltaX = mLastX - pX;
    int deltaY = mLastY - pY;
    bool moveLeftRight = abs(deltaX) > abs(deltaY);
    bool moveUpDown = !moveLeftRight;

    mLastX = pX;
    mLastY = pY;
}

void RotationsViewer::onMouseCb(int button, int state, int x, int y)
{
    theInstance->onMouse(button, state, x, y);
}
void RotationsViewer::onMouse(int pButton, int pState, int pX, int pY)
{
    // Check GUI first
    if (TwEventMouseButtonGLUT(pButton, pState, pX, pY))  return;

    mButtonState = pButton;
    mModifierState = glutGetModifiers();
    mLastX = pX;
    mLastY = pY;

    if (mButtonState == GLUT_LEFT_BUTTON)
    {
    }
}

void RotationsViewer::onKeyboardSpecialCb(int key, int x, int y)
{
    theInstance->onKeyboardSpecial(key, x, y);
}
void RotationsViewer::onKeyboardSpecial(int key, int x, int y)
{
    TwEventSpecialGLUT(key, x, y);
}

void RotationsViewer::onKeyboardCb(unsigned char key, int x, int y)
{
    theInstance->onKeyboard(key, x, y);
}

void RotationsViewer::onKeyboard(unsigned char pKey, int x, int y)
{
    // Exit on ESC key.
    if (pKey == 27)
    {
        exit(0);
    }

    if (TwEventKeyboardGLUT(pKey, x, y)) return;
}

void RotationsViewer::onMenuCb(int value)
{
    theInstance->onMenu(value);
}
void RotationsViewer::onMenu(int value)
{
    switch (value)
    {
    case -1: exit(0);
    default: onKeyboardCb(value, 0, 0); break;
    }
}

void RotationsViewer::onResizeCb(int width, int height)
{
    theInstance->onResize(width, height);
}
void RotationsViewer::onResize(int width, int height)
{
    glViewport(0, 0, (GLsizei)width, (GLsizei)height);
    mWindowWidth = width;
    mWindowHeight = height;
    TwWindowSize(width, height);
}

void TW_CALL RotationsViewer::onCopyStdStringToClient(std::string& dest, const std::string& src)
{
    dest = src;
}

void TW_CALL RotationsViewer::onSetStyleCb(const void *value, void *clientData)
{
    RotationsViewer* viewer = ((RotationsViewer*)clientData);
    ASplineQuat::InterpolationType v = *(const ASplineQuat::InterpolationType *)value;  // for instance
    viewer->mQuatSpline.setInterpolationType(v);
}

void TW_CALL RotationsViewer::onGetStyleCb(void *value, void *clientData)
{
    RotationsViewer* viewer = ((RotationsViewer*)clientData);
    *static_cast<ASplineQuat::InterpolationType *>(value) = viewer->mQuatSpline.getInterpolationType();
}