#pragma once

#include "windows.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include <AntTweakBar.h>
#include <vector>
#include "aSplineVec3.h"

#define DEFAULT_WINDOW_WIDTH 800
#define DEFAULT_WINDOW_HEIGHT 600
#define DEFAULT_WINDOW_STARTX 100
#define DEFAULT_WINDOW_STARTY 100

class CurveViewer
{
public:
    CurveViewer();
    virtual ~CurveViewer();

    virtual void init(int argc, char** argv,
        int winwidth = DEFAULT_WINDOW_WIDTH,
        int winheight = DEFAULT_WINDOW_HEIGHT,
        int winstartx = DEFAULT_WINDOW_STARTX,
        int winstarty = DEFAULT_WINDOW_STARTY);

    virtual void run();

protected:

    virtual void initializeGui();
    virtual bool initializeOpenGL();
    virtual void drawCurveView();
    void DrawCircle(const vec3& p); 
    void DrawCurve(); 

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
    virtual void addPoint(int x, int y);
    virtual void deletePoint(int x, int y);
    virtual void pickPoint(int x, int y);
    virtual void movePicked(int x, int y);

    static void TW_CALL onClearCb(void* usr);
    static void TW_CALL onSetStyleCb(const void *value, void *clientData);
    static void TW_CALL onGetStyleCb(void *value, void *clientData);
    static void TW_CALL onCopyStdStringToClient(std::string& dest, const std::string& src);

protected:

    ASplineVec3 mSpline;
    int mSelected;
    int mPickType;

    mutable int mLastX, mLastY;
    int mMenu;
    int mButtonState;
    int mModifierState;
    int mWindowWidth, mWindowHeight;
	int mClick;

    TwBar *mCurveBar;
    TwType curveType;
    TwType modeType;
    enum Mode { ADD, EDIT, REMOVE } mMode;
    bool mShowControlPoints;
};
