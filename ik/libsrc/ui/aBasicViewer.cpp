#include <string>
#include "aSceneDrawer.h"
#include "aBasicViewer.h"
#include <IL/il.h>
#include <IL/ilu.h>
#include <IL/ilut.h>
#include <algorithm>

extern std::vector<std::string> GetFilenamesInDir(const std::string& dirname);
extern std::string PromptToLoadDir();

#pragma warning(disable : 4018)

static ABasicViewer* theInstance = 0;

ABasicViewer::ABasicViewer() :
    mFilename("None"),
    mModel("None"),
    mDir("None"),
    mTimeScale(1.0),
    mPause(false),
    mFBX(0),
    mCurrentLayer(0),
	mCurrentTime(0)
{
    mClock = new ATimer();
}

ABasicViewer::~ABasicViewer()
{
    delete mClock;
    delete mFBX;
    TwTerminate();
}

void ABasicViewer::init(int argc, char** argv, int winwidth, int winheight, int winstartx, int winstarty)
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

    glutDisplayFunc(ABasicViewer::onDrawCb);
    glutKeyboardFunc(ABasicViewer::onKeyboardCb);
    glutSpecialFunc(ABasicViewer::onKeyboardSpecialCb);
    glutMouseFunc(ABasicViewer::onMouseCb);
    glutMotionFunc(ABasicViewer::onMouseMotionCb);
    glutTimerFunc(100, ABasicViewer::onTimerCb, 0);
    glutReshapeFunc(ABasicViewer::onResizeCb);

    initializeGui();

    mFBX = new AFBX(lSupportVBO);
}

bool ABasicViewer::initializeOpenGL()
{
    // Initialize GLEW.
    GLenum lError = glewInit();
    if (lError != GLEW_OK)
    {
    	FBXSDK_printf("GLEW Error: %s\n", glewGetErrorString(lError));
    	return false;
    }

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glClearColor(0.8, 0.8, 0.8, 1.0);

    // OpenGL 1.5 at least.
    if (!GLEW_VERSION_1_5)
    {
    	FBXSDK_printf("The OpenGL version should be at least 1.5 to display shaded scene!\n");
    	return false;
    }

    glEnable(GL_BLEND);
    glEnable(GL_ALPHA_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glShadeModel(GL_SMOOTH);

    glEnable(GL_NORMALIZE);
    glDisable(GL_LIGHTING);
    glCullFace(GL_BACK);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // Setup lighting
    glEnable(GL_LIGHTING);
    float AmbientColor[] = { 0.25f, 0.25f, 0.25f, 0.0f };   glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientColor);
    float DiffuseColor[] = { 1.0f, 1.0f, 1.0f, 0.0f };   glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseColor);
    float SpecularColor[] = { 0.0f, 0.0f, 0.0f, 0.0f };   glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularColor);
    float Position[] = { 1.0f, 1.0f, 4.0f, 1.0f };  glLightfv(GL_LIGHT0, GL_POSITION, Position);
    glEnable(GL_LIGHT0);

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

void ABasicViewer::initializeGui()
{
    glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
    TwGLUTModifiersFunc(glutGetModifiers);
    TwInit(TW_OPENGL, NULL);
    TwWindowSize(mWindowWidth, mWindowHeight);

    TwCopyStdStringToClientFunc(onCopyStdStringToClient);

    char buff[1024];
    sprintf_s(buff, " min='%d' max='%d' step=0.1", 0, 100);

    mPlayerBar = TwNewBar("Player controls");
    TwDefine(" 'Player controls' size='200 175' position='5 5' iconified=false fontresizable=false alpha=200");
    TwAddButton(mPlayerBar, "PlayBtn", onPlayCb, this, mPause ? " label='Play'" : " label='Pause'");
    TwAddVarCB(mPlayerBar, "Time", TW_TYPE_DOUBLE, onSetTimeCb, onGetTimeCb, this, " step=0.1");
    TwAddVarRW(mPlayerBar, "Time scale", TW_TYPE_DOUBLE, &mTimeScale, "");
    TwAddVarRW(mPlayerBar, "Motion", TW_TYPE_STDSTRING, &mFilename, "");
    TwAddVarRW(mPlayerBar, "Model", TW_TYPE_STDSTRING, &mModel, "");
    TwAddVarRW(mPlayerBar, "Enable skin", TW_TYPE_BOOLCPP, &AFBX::gDrawMesh, "");

    mFilesBar = TwNewBar("File controls");
    TwDefine(" 'File controls' size='200 300' position='5 185' iconified=false fontresizable=false alpha=200");
    TwAddVarRW(mFilesBar, "Directory", TW_TYPE_STDSTRING, &mDir, " group='Dir'");
    TwAddButton(mFilesBar, "Browse", onBrowseCb, this, " label='--> Browse' group='Dir'");
}

void ABasicViewer::loadDir(const std::string& dir)
{
    for (int i = 0; i < mFiles.size(); i++)
    {
    	TwRemoveVar(mFilesBar, mFiles[i].c_str());
    	delete mLoadData[i];
    }
    mFiles.clear();
    mLoadData.clear();
    mDir = "";

    char buff[1024];
    std::vector<std::string> dirs = GetFilenamesInDir(dir);
    for (int i = 0; i < dirs.size(); i++)
    {
    	std::string ext = dirs[i].substr(std::max<int>(4, dirs[i].size()) - 4);
    	if (ext != ".bvh") continue;

    	LoadData* ld = new LoadData;
    	ld->viewer = this;
    	ld->fileid = mFiles.size();
    	mFiles.push_back(dir + "/" + dirs[i]);
    	mLoadData.push_back(ld);

    	sprintf_s(buff, " label='%s' group='Dir'", dirs[i].c_str());
    	TwAddButton(mFilesBar, mFiles.back().c_str(), onLoadCb, ld, buff);
    }
}

bool ABasicViewer::loadModel(const std::string& filename)
{
    if (mFBX->IsValid())
    {
        if (mFBX->Load(filename))
        {
            mModel = ABasicViewer::pruneName(filename);

            // move to beginning
            mCurrentTime = mFBX->GetStart().GetSecondDouble();
            frameCamera();
            return true;
        }
    }
    return false;
}

void ABasicViewer::run()
{
    glutMainLoop();
}

void ABasicViewer::browse()
{    
    std::string dir = PromptToLoadDir();
    if (dir != "")
    {
    	std::cout << dir << std::endl;
    	loadDir(dir);
    }
}

void ABasicViewer::grabScreen()
{
    static int image_count = 0;

    unsigned int image;
    ilGenImages(1, &image);
    ilBindImage(image);

    ILenum error = ilGetError();
    assert(error == IL_NO_ERROR);

    ilTexImage(mWindowWidth, mWindowHeight, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, NULL);

    error = ilGetError();
    assert(error == IL_NO_ERROR);

    unsigned char* data = ilGetData();

    error = ilGetError();
    assert(error == IL_NO_ERROR);

    for (int i = mWindowHeight - 1; i >= 0; i--)
    {
    	glReadPixels(0, i, mWindowWidth, 1, GL_RGB, GL_UNSIGNED_BYTE,
    		data + (mWindowWidth * 3 * i));
    }

    char anim_filename[2048];
    sprintf_s(anim_filename, 2048, "img_%04d.png", image_count++);

    ilEnable(IL_FILE_OVERWRITE);
    ilSave(IL_PNG, anim_filename);

    error = ilGetError();
    if (error != IL_NO_ERROR) printf("%d: %s/n", error, iluErrorString(error));
    //assert(error == IL_NO_ERROR);

    ilDeleteImages(1, &image);

    error = ilGetError();
    if (error != IL_NO_ERROR)printf("%d: %s/n", error, iluErrorString(error));
    //assert(error == IL_NO_ERROR);
}

void ABasicViewer::frameCamera()
{
    vec3 pos = vec3(0, 0, 0); 
    vec3 dim = vec3(10, 20, 10); 
    if (mFBX->GetImporter())
    {
        FbxVector4 dimfbx = mFBX->GetDimensions();
        dim[0] = dimfbx[0];
        dim[1] = dimfbx[1];
        dim[2] = dimfbx[2];

        pos[1] = dim[1]*0.5; // assumes Y is up
    }
    mCamera.frameVolume(pos, dim);
}

void ABasicViewer::onMouseMotionCb(int x, int y)
{
    theInstance->onMouseMotion(x, y);
}
void ABasicViewer::onMouseMotion(int pX, int pY)
{
    // Check GUI first
    if (TwEventMouseMotionGLUT(pX, pY)) return;

    int deltaX = mLastX - pX;
    int deltaY = mLastY - pY;
    bool moveLeftRight = abs(deltaX) > abs(deltaY);
    bool moveUpDown = !moveLeftRight;
    if (mButtonState == GLUT_LEFT_BUTTON)  // Rotate
    {
    	if (moveLeftRight && deltaX > 0) mCamera.orbitLeft(deltaX);
    	else if (moveLeftRight && deltaX < 0) mCamera.orbitRight(-deltaX);
    	else if (moveUpDown && deltaY > 0) mCamera.orbitUp(deltaY);
    	else if (moveUpDown && deltaY < 0) mCamera.orbitDown(-deltaY);
    }
    else if (mButtonState == GLUT_MIDDLE_BUTTON) // Zoom
    {
    	if (moveUpDown && deltaY > 0) mCamera.moveForward(deltaY);
    	else if (moveUpDown && deltaY < 0) mCamera.moveBack(-deltaY);
    }
    else if (mButtonState == GLUT_RIGHT_BUTTON) // Pan
    {
    	if (moveLeftRight && deltaX > 0) mCamera.moveLeft(deltaX);
    	else if (moveLeftRight && deltaX < 0) mCamera.moveRight(-deltaX);
    	else if (moveUpDown && deltaY > 0) mCamera.moveUp(deltaY);
    	else if (moveUpDown && deltaY < 0) mCamera.moveDown(-deltaY);
    }

    mLastX = pX;
    mLastY = pY;

    glutPostRedisplay();
}

void ABasicViewer::onMouseCb(int button, int state, int x, int y)
{
    theInstance->onMouse(button, state, x, y);
}
void ABasicViewer::onMouse(int pButton, int pState, int pX, int pY)
{
    // Check GUI first
    if (TwEventMouseButtonGLUT(pButton, pState, pX, pY))  return;

    mButtonState = pButton;
    mModifierState = glutGetModifiers();
    mLastX = pX;
    mLastY = pY;
    onMouseMotionCb(pX, pY);
}

void ABasicViewer::onKeyboardSpecialCb(int key, int x, int y)
{
    theInstance->onKeyboardSpecial(key, x, y);
}
void ABasicViewer::onKeyboardSpecial(int key, int x, int y)
{
    TwEventSpecialGLUT(key, x, y);
}

void ABasicViewer::onTimerCb(int value)
{
    theInstance->onTimer(value);
}

void ABasicViewer::onTimer(int value)
{
    // Loop in the animation stack if not paused.
    double dt = mClock->totalElapsedTime();
    mClock->restart();

    if (!mPause)
    {
        double currentTime = mCurrentTime + dt * mTimeScale;
        mCurrentTime = currentTime;
    }

    // Call the timer to display the next frame.    
    int nextFrame = (unsigned int) mFBX->GetFrameTime().GetMilliSeconds();
    glutTimerFunc(10, onTimerCb, 0);

    // Must call this everytime so that GUI displays
    glutPostRedisplay();
    TwRefreshBar(mPlayerBar);
}

void ABasicViewer::onKeyboardCb(unsigned char key, int x, int y)
{
    theInstance->onKeyboard(key, x, y);
}

void ABasicViewer::onKeyboard(unsigned char pKey, int x, int y)
{
    // Exit on ESC key.
    if (pKey == 27)
    {
    	exit(0);
    }

    if (TwEventKeyboardGLUT(pKey, x, y)) return;

    if (pKey == 'f')
    {
    	frameCamera();
    }
    else if (pKey == 'r')
    {
    	grabScreen();
    }
}

void ABasicViewer::onMenuCb(int value)
{
    theInstance->onMenu(value);
}
void ABasicViewer::onMenu(int value)
{
    switch (value)
    {
    case -1: exit(0);
    default: onKeyboardCb(value, 0, 0); break;
    }
}

void ABasicViewer::onResizeCb(int width, int height)
{
    theInstance->onResize(width, height);
}
void ABasicViewer::onResize(int width, int height)
{
    glViewport(0, 0, (GLsizei)width, (GLsizei)height);
    mWindowWidth = width;
    mWindowHeight = height;
    TwWindowSize(width, height);
}

void ABasicViewer::onDrawCb()
{
    theInstance->onDraw();
}
void ABasicViewer::onDraw()
{
    draw3DView();
    drawOverlay();
    TwDraw();  // gui
    glutSwapBuffers();
}

void ABasicViewer::drawOverlay()
{
}

void ABasicViewer::draw3DView()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE); // Draw the front face only, except for the texts and lights.
    glEnable(GL_LIGHTING);

    // Set the view to the current camera settings.
    mCamera.draw();

    GLfloat pos[4];
    pos[0] = mCamera.getPosition()[0];
    pos[1] = mCamera.getPosition()[1]+1.0;
    pos[2] = mCamera.getPosition()[2];
    pos[3] = 0.0;  // could be 1.0
    glLightfv(GL_LIGHT0, GL_POSITION, pos);

    mFBX->Draw(mCurrentTime, mCurrentLayer);

    glDisable(GL_LIGHTING);
    displayGrid();
}

void ABasicViewer::displayGrid()
{
    // Draw a grid 500*500
    if (ACamera::gDfltUp[2] == 1) // Z-UP =>rotate
    {
        glPushMatrix();
        glRotatef(90, 1, 0, 0);
    }
    glColor3f(0.8f, 0.8f, 0.8f);
    glLineWidth(1.0);
    const int hw = 500;
    const int step = 100;
    const int bigstep = 500;
    int i;

    // Draw Grid
    for (i = -hw; i <= hw; i += step) {

    	if (i % bigstep == 0) {
    		glLineWidth(2.0);
    	}
    	else {
    		glLineWidth(1.0);
    	}
    	glColor4f(0.6f, 0.6f, 0.6f, 1.0f);
    	if (i == 0) glColor3f(0.3f, 0.3f, 0.7f);
    	glBegin(GL_LINES);
    	glVertex3i(i, 0, -hw);
    	glVertex3i(i, 0, hw);
    	glEnd();
    	if (i == 0) glColor3f(0.7f, 0.3f, 0.3f);
    	glBegin(GL_LINES);
    	glVertex3i(-hw, 0, i);
    	glVertex3i(hw, 0, i);
    	glEnd();

    }
    if (ACamera::gDfltUp[2] == 1) // Z-UP =>rotate
    {
        glPopMatrix();
    }
}

void ABasicViewer::load(const std::string& filename)
{
    std::cout << "Load motion file " << filename.c_str() << std::endl;
}

// Callback function called by AntTweakBar to set the "TextLine" CDString variable
void TW_CALL ABasicViewer::onLoadCb(void *clientData)
{
    LoadData* data = (LoadData*)clientData;
    ABasicViewer* viewer = data->viewer;
    int fileid = data->fileid;
    std::string filename = viewer->mFiles[fileid].c_str();
    viewer->load(filename);
}

void TW_CALL ABasicViewer::onCopyStdStringToClient(std::string& dest, const std::string& src)
{
    dest = src;
}

void TW_CALL ABasicViewer::onPlayCb(void* usr)
{
    ABasicViewer* viewer = ((ABasicViewer*)usr);
    viewer->mPause = !viewer->mPause;

    if (viewer->mPause) TwSetParam(viewer->mPlayerBar, "PlayBtn", "label", TW_PARAM_CSTRING, 1, "Play");
    else TwSetParam(viewer->mPlayerBar, "PlayBtn", "label", TW_PARAM_CSTRING, 1, "Pause");
}

void TW_CALL ABasicViewer::onBrowseCb(void* usr)
{
    ((ABasicViewer*)usr)->browse();
}


void TW_CALL ABasicViewer::onSetTimeCb(const void *value, void *clientData)
{
    ABasicViewer* viewer = ((ABasicViewer*)clientData);
    double v = *(const double *) value;  // for instance
    v = std::max<double>(0, v);
    viewer->mCurrentTime = v;

    //std::cout << "SET FRAME BUTTON PRESSED " << v << " " << std::endl;
    glutPostRedisplay();
}

void TW_CALL ABasicViewer::onGetTimeCb(void *value, void *clientData)
{
    ABasicViewer* viewer = ((ABasicViewer*)clientData);
    *static_cast<double *>(value) = viewer->mCurrentTime;
}

std::string ABasicViewer::pruneName(const std::string& name)
{
    std::string temp = name;

    int i = temp.rfind(".");
    if (i != std::string::npos) temp = temp.replace(i, temp.size(), "");

    return pruneDir(temp);
}

std::string ABasicViewer::pruneDir(const std::string& name)
{
    std::string temp = name;

    int i = name.rfind("/", name.size() - 2);
    if (i == std::string::npos) i = name.rfind("\\", name.size() - 2);
    if (i != std::string::npos) temp = temp.replace(0, i + 1, "");

    return temp;
}
