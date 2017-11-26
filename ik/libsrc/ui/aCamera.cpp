//------------------------------------------------------------------------
// Copyright (C) 2009 Aline Normoyle

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <algorithm>

#include "aCamera.h"

vec3  ACamera::gDfltEye(0.0, 2.0, 2.5);
vec3  ACamera::gDfltUp(0.0, 0.0, 1.0);
vec3  ACamera::gDfltLook(0.0, 0.0, 0.0);
float ACamera::gDfltVfov = 60.0;
float ACamera::gDfltAspect = 1.0;
float ACamera::gDfltNear = 10.0; //0.1;
float ACamera::gDfltFar = 2000.0; // 500.0;
float ACamera::gDfltSpeed = 0.5;
float ACamera::gDfltTurnRate = 1.0*(M_PI / 180.0);

ACamera::ACamera() 
{   
   mDir = NONE; mTurnDir = NONE;
   reset();
}

ACamera::~ACamera() 
{
}

void ACamera::reset()
{
   mSpeed = gDfltSpeed;
   mTurnRate = gDfltTurnRate;
   mVfov = gDfltVfov;
   mAspect = gDfltAspect;
   mNear = gDfltNear;
   mFar = gDfltFar;

   // Calculate the initial heading & pitch
   // Note that  eye[0] = radius*cos(h)*cos(p); and  eye[1] = radius*sin(p);

   // Y UP
   if (gDfltUp == axisY)
   {
       mPitch = 0.0; //-asin(dfltEye[1]/dfltEye.Length());
       mHeading = -0.85; //acos(dfltEye[0]/(dfltEye.Length()*cos(mPitch)));
   }
   else
   {
       mPitch = -asin(gDfltEye[2]/gDfltEye.Length());
       mHeading = acos(gDfltEye[0]/(gDfltEye.Length()*cos(mPitch)));
   }

   //printf("PITCH: %f\n", mPitch);
   //printf("HEADING: %f\n", mHeading);
   //printf("RADIUS: %f\n", mRadius);

   //printf("INIT: %f %f\n", mPitch, mHeading);
   set(gDfltEye, gDfltLook, gDfltUp);
}

void ACamera::draw()
{
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(mVfov, mAspect, mNear, mFar);

   float m[16];
   m[0] = mV[0]; m[4] = mV[1]; m[8] = mV[2];  m[12] = -Dot(mEye, mV);
   m[1] = mU[0]; m[5] = mU[1]; m[9] = mU[2];  m[13] = -Dot(mEye, mU);
   m[2] = mN[0]; m[6] = mN[1]; m[10] = mN[2]; m[14] = -Dot(mEye, mN);
   m[3] = 0.0;  m[7] = 0.0;  m[11] = 0.0;  m[15] = 1.0;
   glMatrixMode(GL_MODELVIEW);
   glLoadMatrixf(m); 

   glGetDoublev(GL_MODELVIEW_MATRIX, mModelMatrix);
   glGetDoublev(GL_PROJECTION_MATRIX, mProjMatrix);
   glGetIntegerv(GL_VIEWPORT, mViewport);
}

const vec3& ACamera::getUp() const
{
   return mU;
}

const vec3& ACamera::getBackward() const
{
   return mN;
}

const vec3& ACamera::getRight() const
{
   return mV;
}

vec3 ACamera::getRelativePosition(float left, float up, float forward)
{
   vec3 direction = up*mU + left*mV - forward*mN;
   return mEye + direction;  // Move along forward axis 
}

void ACamera::getViewport(int& x, int& y, int& w, int& h)
{
   x = mViewport[0];
   y = mViewport[1];
   w = mViewport[2];
   h = mViewport[3];
}

void ACamera::getProjection(float& vfov, float& aspect, float& zNear, float& zFar)
{
   vfov = mVfov; aspect = mAspect; zNear = mNear; zFar = mFar;
}

void ACamera::setPosition(const vec3& pos)
{
   mEye = pos;
}

const vec3& ACamera::getPosition() const
{
   return mEye;
}

void ACamera::setProjection(float vfov, float aspect, float zNear, float zFar)
{
   mVfov = vfov;
   mAspect = aspect;
   mNear = zNear;
   mFar = zFar;
}

float ACamera::heading() const
{
   return mHeading;
}

float ACamera::pitch() const
{
   return mPitch;
}

void ACamera::set(const vec3& eyepos, const vec3& look, const vec3& up)
{
   mEye = eyepos;
   mN = eyepos - look;
   mV = up.Cross(mN);
   mU = mN.Cross(mV);
   mRadius = mN.Length(); // cache this distance

   mU.Normalize();
   mV.Normalize();
   mN.Normalize();
}

void ACamera::print()
{
   mEye.Print("EYE: ");
   mV.Print("RIGHT: ");
   mU.Print("UP: ");
   mN.Print("N: ");
   printf("-----------------------\n");
}

void ACamera::move(float dV, float dU, float dN)
{
   mEye += dU*mU + dV*mV + dN*mN;
}

void ACamera::orbit(float h, float p)
{
   //printf("PITCH: %f\n", p);
   //printf("HEADING: %f\n", h);
   //printf("RADIUS: %f\n", mRadius);

   vec3 rotatePt; // Calculate new location around sphere having mRadius
   if (gDfltUp == axisY)
   {
       // Y UP
       rotatePt[0] = mRadius*cos(h)*cos(p);
       rotatePt[1] = mRadius*sin(p);
       rotatePt[2] = mRadius*sin(h)*cos(p);
   }
   else
   {
       rotatePt[0] = mRadius*cos(h)*cos(p);
       rotatePt[1] = mRadius*sin(h)*cos(p);
       rotatePt[2] = mRadius*sin(p);
   }

   vec3 lookAt = mEye-mN*mRadius;
   set(lookAt-rotatePt, lookAt /* look */, gDfltUp /* up Approx */);
}

void ACamera::orbitLeft(float scale) 
{
   mTurnDir = TL;
   mHeading += mTurnRate*scale;
   orbit(mHeading, pitch());
}

void ACamera::moveLeft(float scale) // => move along v
{    
   mDir = L;
   move(-mSpeed*scale, 0.0, 0.0);
}

void ACamera::orbitRight(float scale)
{
   mTurnDir = TR;
   mHeading -= mTurnRate*scale;
   orbit(mHeading, pitch());
}

void ACamera::moveRight(float scale) // => move along v
{
   mDir = R;
   move(mSpeed*scale, 0.0, 0.0);   
}

void ACamera::orbitUp(float scale)
{
   mTurnDir = TU; 
   mPitch = std::min<float>(-0.1, mPitch + mTurnRate*scale);
   orbit(heading(), mPitch);
}

void ACamera::moveUp(float scale) // => move along +u
{
   mDir = U;
   move(0.0, mSpeed*scale, 0.0);   
}

void ACamera::orbitDown(float scale)
{
   mTurnDir = TD; 
   mPitch = std::max<float>(-M_PI / 2.0 + 0.01, mPitch - mTurnRate*scale);
   orbit(heading(), mPitch);
}

void ACamera::moveDown(float scale) // => move along -u
{
   mDir = D;
   move(0.0, -mSpeed*scale, 0.0);   
}

void ACamera::moveForward(float scale) // => move along -n
{
   mDir = F; 
   move(0.0, 0.0, -mSpeed*scale);      
   mRadius += -mSpeed*scale;  // Also "zoom" into radius
}

void ACamera::moveBack(float scale) // => move along n
{
   mDir = B; 
   move(0.0, 0.0, mSpeed*scale);   
   mRadius += mSpeed*scale;  // Also "zoom" out radius
}

void ACamera::turn(vec3& v1, vec3& v2, float amount)
{
   double cosTheta = cos(amount);
   double sinTheta = sin(amount);

   float vX =  cosTheta*v1[0] + sinTheta*v2[0]; 
   float vY =  cosTheta*v1[1] + sinTheta*v2[1]; 
   float vZ =  cosTheta*v1[2] + sinTheta*v2[2]; 

   float nX = -sinTheta*v1[0] + cosTheta*v2[0]; 
   float nY = -sinTheta*v1[1] + cosTheta*v2[1]; 
   float nZ = -sinTheta*v1[2] + cosTheta*v2[2]; 

   v1.set(vX, vY, vZ);
   v2.set(nX, nY, nZ);
}

void ACamera::turnLeft(float scale) // rotate around u
{
   mTurnDir = TL; 
   turn(mV, mN, -mTurnRate*scale);
}

void ACamera::turnRight(float scale) // rotate neg around u
{
   mTurnDir = TR;
   turn(mV, mN, mTurnRate*scale);
}

void ACamera::turnUp(float scale) // rotate around v
{
   mTurnDir = TU; 
   turn(mN, mU, mTurnRate*scale);
}

void ACamera::turnDown(float scale) // rotate around v
{
   mTurnDir = TD; 
   turn(mN, mU, -mTurnRate*scale);
}

bool ACamera::screenToWorld(int screenX, int screenY, vec3& worldCoords)
{
   double x, y, z;
   GLint result = gluUnProject(screenX, screenY, 0.0, 
                               mModelMatrix, mProjMatrix, mViewport, 
                               &x, &y, &z);

   worldCoords.set(x, y, z);
   return result == GL_TRUE;
}

bool ACamera::worldToScreen(const vec3& worldCoords, int& screenX, int& screenY)
{
   double x, y, z;
   GLint result = gluProject(worldCoords[0], worldCoords[1], worldCoords[2],
                             mModelMatrix, mProjMatrix, mViewport, 
                             &x, &y, &z);

   screenX = (int) x;
   screenY = (int) y;
   return result == GL_TRUE;
}

void ACamera::frameVolume(const vec3& pos, const vec3& dim)
{
    double w = dim[0];
    double h = dim[1];
    double d = dim[2];
    double angle = 0.5*gDfltVfov*M_PI / 180.0;
    double dist;
    if (d > h)
    {
        gDfltUp.set(0, 0.0, 1.0);
        if (w > d) dist = w*0.5 / tan(angle);  // aspect is 1, so i can do this
        else dist = d*0.5 / tan(angle);
        gDfltEye = gDfltEye + pos;
        gDfltLook = pos + vec3(0, 0.0, d*0.75);
    }
    else
    {
        gDfltUp.set(0, 1.0, 0.0);
        if (w > h) dist = w*0.5 / tan(angle);  // aspect is 1, so i can do this
        else dist = h*0.5 / tan(angle);
        gDfltEye.set(-(dist + d), h*0.1, (dist + d));
        gDfltEye = gDfltEye + pos;
        gDfltLook = pos + vec3(0, h*0.5, 0);
    }
    double n = std::max<double>(std::max<double>(w, h), d);
    gDfltNear = n * 0.01;
    gDfltFar = n * 5000;
    reset();
}
