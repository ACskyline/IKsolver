/****************************************************************************************

Copyright (C) 2013 Autodesk, Inc.
All rights reserved.

Use of this software is subject to the terms of the Autodesk license agreement
provided at the time of installation or download, or which otherwise accompanies
this software in either electronic or hard copy form.

****************************************************************************************/
// ASN MOD - This class has been adapted to be an interface to the FBX API which is used 
// to load and save FBX files and maintain a scene graph hierarchy. 

#ifndef _FBX_H
#define _FBX_H

#include <string>
#include "windows.h"
#include <fbxsdk.h>
#include <map>
#include "aSceneDrawer.h"
#include "aTransform.h"

#include "aJoint.h"
#include "aSkeleton.h"

#include "aMotion.h"

class AFBX
{
public:
    AFBX(bool pSupportVBO);
    virtual ~AFBX();

    const FbxScene* GetScene() const { return mScene; }
    FbxScene * GetScene() { return mScene; }
    FbxImporter * GetImporter() {return mImporter; }
    FbxAnimLayer * GetAnimationLayer() { return mCurrentAnimLayer; }
    FbxVector4 GetDimensions() const;

    void SetFPS(double fps);
    void SetDuration(int numFrames);

    const FbxTime GetFrameTime() const { return mFrameTime; }
    const FbxTime GetStart() const { return mStart; }
    const FbxTime GetStop() const { return mStop; }
    int GetNumLayers() const { return mAnimStackNameArray.Size(); }

    void SetShadingMode(ShadingMode pMode);
    bool SetCurrentAnimStack(int pIndex);
    bool Load(const std::string& filename);
    bool Save(const std::string& filename);
    bool IsValid() const;
    FbxAMatrix Draw(double timeInSecs, int layer = 0);

    void SetPose(const ASkeleton& skeleton);
    void GetPose(double time, ASkeleton& skeleton);
    void SetPosition(int jointid, const vec3& pos); // set on FbxNode
    vec3 GetPosition(int jointid); // value from FbxNode

    int GetNumLimbs() const;
    int GetNumFrames() const;
    void GetLimbInfo(int i, std::string& jointname, std::string& jointparent, 
        FbxDouble3& offset, bool& isRoot, int& numChannels, std::string& rotOrder) const;
    quat GetRotation(double time, int limb); // value from animation curves
    vec3 GetRootTranslation(double time); // from curves
    ASkeleton ExportSkeleton() const;
    AMotion ExportMotion(double startTime, double endTime, double fps = 120.0);

protected:

    void UnloadCacheRecursive(FbxScene* pScene);
    void UnloadCacheRecursive(FbxNode * pNode);
    void LoadCacheRecursive(FbxScene * pScene, FbxAnimLayer * pAnimLayer, const char * pFbxFileName, bool pSupportVBO);
    void LoadCacheRecursive(FbxNode * pNode, FbxAnimLayer * pAnimLayer, bool pSupportVBO);
    bool LoadTextureFromFile(const FbxString & pFilePath, unsigned int & pTextureObject);
    void PreparePointCacheData(FbxScene* pScene, FbxTime &pCache_Start, FbxTime &pCache_Stop);
    void InitializeSdkObjects(FbxManager*& pManager, FbxScene*& pScene);
    void DestroySdkObjects(FbxManager* pManager, bool pExitStatus);
    void CreateAndFillIOSettings(FbxManager* pManager);
    void FillSkeletonArrayRecursive(FbxNode* pNode, FbxArray<FbxNode*>& pSkeletonArray);

    bool SaveScene(FbxManager* pManager, FbxDocument* pScene, const char* pFilename, int pFileFormat=-1, bool pEmbedMedia=false);
    bool LoadScene(FbxManager* pManager, FbxDocument* pScene, const char* pFilename);

protected:
    std::string mFileName;
    FbxManager * mSdkManager;
    FbxScene * mScene;
    FbxImporter * mImporter;
    FbxAnimLayer * mCurrentAnimLayer;
    FbxNode * mSelectedNode;
    FbxArray<FbxString*> mAnimStackNameArray;
    FbxArray<FbxNode*> mSkeletonArray;
    std::map<std::string, int> mJointMap; // strings to ids

    mutable FbxTime mFrameTime, mStart, mStop;
    mutable FbxTime mCache_Start, mCache_Stop;

    bool mPause;
    ShadingMode mShadingMode;
    bool mSupportVBO;
    FbxVector4 mBBoxMin;
    FbxVector4 mBBoxMax;
    FbxVector4 mDimensions;

public:
    static bool gDrawMesh;
};

#endif // _SCENE_CONTEXT_H

