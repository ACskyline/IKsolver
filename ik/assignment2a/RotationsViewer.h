#pragma once

#include "windows.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include <AntTweakBar.h>
#include <vector>
#include "aSplineQuat.h"

#define DEFAULT_WINDOW_WIDTH 800
#define DEFAULT_WINDOW_HEIGHT 600
#define DEFAULT_WINDOW_STARTX 100
#define DEFAULT_WINDOW_STARTY 100

class ATimer;
class RotationsViewer
{
public:
    RotationsViewer();
    virtual ~RotationsViewer();

    virtual void init(int argc, char** argv,
        int winwidth = DEFAULT_WINDOW_WIDTH,
        int winheight = DEFAULT_WINDOW_HEIGHT,
        int winstartx = DEFAULT_WINDOW_STARTX,
        int winstarty = DEFAULT_WINDOW_STARTY);

    virtual void run();

protected:

	enum RotOrder { ZYX, XYZ, YZX, XZY, YXZ, ZXY } mRotOrder;
    enum { DEMO1, DEMO2 } mMode;

    virtual void initializeGui();
    virtual bool initializeOpenGL();
    virtual void drawDemo1();
    virtual void drawDemo2();
    virtual void pushEulerRotation(RotOrder roo, const vec3& euler);

    static void onMouseMotionCb(int x, int y);
    static void onMouseCb(int button, int state, int x, int y);
    static void onKeyboardCb(unsigned char key, int x, int y);
    static void onKeyboardSpecialCb(int key, int x, int y);
    static void onMenuCb(int value);
    static void onResizeCb(int width, int height);
    static void onDrawCb();
    static void onTimerCb(int value);

    virtual void onMouseMotion(int x, int y);
    virtual void onMouse(int button, int state, int x, int y);
    virtual void onKeyboard(unsigned char key, int x, int y);
    virtual void onKeyboardSpecial(int key, int x, int y);
    virtual void onMenu(int value);
    virtual void onResize(int width, int height);
    virtual void onDraw();
    virtual void onTimer(int value);

    static void TW_CALL onCopyStdStringToClient(std::string& dest, const std::string& src);
    static void TW_CALL onSetStyleCb(const void *value, void *clientData);
    static void TW_CALL onGetStyleCb(void *value, void *clientData);

protected:

    mutable int mLastX, mLastY;
    int mMenu;
    int mButtonState;
    int mModifierState;
    int mWindowWidth, mWindowHeight;

    TwBar *mDemoBar;
    TwType modeType;
    TwType rooType;
    double mXAngle, mYAngle, mZAngle;

    ASplineQuat mQuatSpline;
    TwType splineType;
    ATimer* mClock;
};
