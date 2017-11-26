//------------------------------------------------------------------------
// Copyright (C) 2009 Aline Normoyle


#ifndef camera_H_
#define camera_H_

#include "GL/gl.h"
#include <aVector.h>
#include <aRotation.h>

class ACamera
{
public:
   ACamera();
   virtual ~ACamera();

   // Draw projection and eyepoint
   virtual void draw();

   // Print eyepoint position and basis
   virtual void print();

   // Initialize the camera with glyLookAt parameters
   virtual void set(const vec3& eyepos, const vec3& look, const vec3& up);

   // Set camera parameters to fit the given view volume
   virtual void frameVolume(const vec3& pos, const vec3& dim);

   // Get camera state
   virtual void setPosition(const vec3& pos);
   virtual const vec3& getPosition() const;
   virtual const vec3& getUp() const;
   virtual const vec3& getBackward() const;
   virtual const vec3& getRight() const;
   virtual vec3 getRelativePosition(float left, float up, float forward);
   virtual float heading() const;
   virtual float pitch() const;

   // Camera frustrum managements
   virtual void setProjection(float vfov, float aspect, float zNear, float zFar);
   virtual void getProjection(float& vfov, float& aspect, float& zNear, float& zFar);
   virtual void getViewport(int& x, int& y, int& w, int& h);

   // Relative movement commands
   virtual void moveLeft(float scale = 1.0);
   virtual void moveRight(float scale = 1.0);
   virtual void moveUp(float scale = 1.0);
   virtual void moveDown(float scale = 1.0);
   virtual void moveForward(float scale = 1.0);
   virtual void moveBack(float scale = 1.0);

   virtual void turnLeft(float scale = 1.0);
   virtual void turnRight(float scale = 1.0);
   virtual void turnUp(float scale = 1.0);
   virtual void turnDown(float scale = 1.0);

   virtual void orbitLeft(float scale = 1.0);
   virtual void orbitRight(float scale = 1.0);
   virtual void orbitUp(float scale = 1.0);
   virtual void orbitDown(float scale = 1.0);

   // Reset to original state
   virtual void reset();

   // Conversion utilities between screen and world coordinates
   virtual bool screenToWorld(int screenX, int screenY, vec3& worldCoords);
   virtual bool worldToScreen(const vec3& worldCoords, int& screenX, int& screenY);

protected:
   enum Dir { NONE, F, B, L, R, U, D, TL, TR, TU, TD} mDir, mTurnDir;
   virtual void turn(vec3& v, vec3& n, float amount);
   virtual void move(float dU, float dV, float dN);
   virtual void orbit(float h, float p);

protected:
   float mSpeed, mTurnRate;

   vec3 mEye; // camera position
   float mHeading, mPitch, mRadius;
   float mVfov, mAspect, mNear, mFar; // projection parameters
   
   // Basis of camera local coord system
   vec3 mU; // up
   vec3 mV; // v points right
   vec3 mN; // -n points forward

   // Cache useful values
   GLdouble mProjMatrix[16];
   GLdouble mModelMatrix[16];
   GLint mViewport[4];

public:

   // Defaults
   static vec3 gDfltEye;
   static vec3 gDfltUp;
   static vec3 gDfltLook;
   static float gDfltVfov;
   static float gDfltAspect;
   static float gDfltNear;
   static float gDfltFar;
   static float gDfltSpeed;
   static float gDfltTurnRate;
};

#endif
