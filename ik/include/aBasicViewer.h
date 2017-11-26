#ifndef ABasicViewer_H_
#define ABasicViewer_H_

#include "windows.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include "aFBX.h"
#include "aCamera.h"
#include "aTimer.h"
#include <AntTweakBar.h>
#include <vector>

#define DEFAULT_WINDOW_WIDTH 800
#define DEFAULT_WINDOW_HEIGHT 600
#define DEFAULT_WINDOW_STARTX 100
#define DEFAULT_WINDOW_STARTY 100

class ABasicViewer
{
public:
    ABasicViewer();
    virtual ~ABasicViewer();

    virtual void init(int argc, char** argv,
        int winwidth = DEFAULT_WINDOW_WIDTH,
        int winheight = DEFAULT_WINDOW_HEIGHT,
        int winstartx = DEFAULT_WINDOW_STARTX,
        int winstarty = DEFAULT_WINDOW_STARTY);

    virtual void run();
    virtual void browse();
    virtual void grabScreen();
    virtual void loadDir(const std::string& dir);
    virtual bool loadModel(const std::string& filename);

protected:

    virtual bool initializeOpenGL();
    virtual void initializeGui();
    virtual void frameCamera();
    virtual void displayGrid();

    static void onMouseMotionCb(int x, int y);
    static void onMouseCb(int button, int state, int x, int y);
    static void onKeyboardCb(unsigned char key, int x, int y);
    static void onKeyboardSpecialCb(int key, int x, int y);
    static void onTimerCb(int value);
    static void onMenuCb(int value);
    static void onResizeCb(int width, int height);
    static void onDrawCb();

    virtual void onMouseMotion(int x, int y);
    virtual void onMouse(int button, int state, int x, int y);
    virtual void onKeyboard(unsigned char key, int x, int y);
    virtual void onKeyboardSpecial(int key, int x, int y);
    virtual void onTimer(int value);
    virtual void onMenu(int value);
    virtual void onResize(int width, int height);
    virtual void onDraw();
    virtual void draw3DView();
    virtual void drawOverlay();
    virtual void load(const std::string& filename);

    static void TW_CALL onLoadCb(void *clientData);
    static void TW_CALL onPlayCb(void* usr);
    static void TW_CALL onBrowseCb(void* usr);
    static void TW_CALL onSetTimeCb(const void *value, void *clientData);
    static void TW_CALL onGetTimeCb(void *value, void *clientData);

public:
    static std::string pruneName(const std::string& name);
    static std::string pruneDir(const std::string& name);
    static void TW_CALL onCopyStdStringToClient(std::string& dest, const std::string& src);

protected:
    ATimer* mClock;
    ACamera mCamera;
    mutable int mLastX, mLastY;
    int mMenu;
    int mButtonState;
    int mModifierState;
    int mWindowWidth, mWindowHeight;

    AFBX* mFBX; 
    double mCurrentTime;
    mutable FbxString mWindowMessage;
    mutable int mCurrentLayer;

    TwBar *mPlayerBar;
    double mTimeScale;
    bool mPause;

    TwBar *mFilesBar;
    std::string mFilename, mDir, mModel;
    std::vector<std::string> mFiles;
    struct LoadData { ABasicViewer* viewer; int fileid; };
    std::vector<LoadData*> mLoadData;
};

#endif