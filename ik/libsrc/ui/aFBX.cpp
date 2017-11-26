/****************************************************************************************

Copyright (C) 2013 Autodesk, Inc.
All rights reserved.

Use of this software is subject to the terms of the Autodesk license agreement
provided at the time of installation or download, or which otherwise accompanies
this software in either electronic or hard copy form.

****************************************************************************************/

#include "aFBX.h"
#include "aSceneCache.h"
#include "aSceneDrawer.h"
#include "aImage.h"
#include <iostream>
#include <algorithm>
#pragma warning(disable : 4018)
//static const double M_PI = 3.1415926535897932384626433832795;
//static const double Deg2Rad = (M_PI / 180.0f);			// Degree to Rad

bool AFBX::gDrawMesh = true;

#ifdef IOS_REF
    #undef  IOS_REF
    #define IOS_REF (*(pManager->GetIOSettings()))
#endif

AFBX::AFBX(bool pSupportVBO) :
    mFileName("None"), 
    mSdkManager(NULL), 
    mScene(NULL), 
    mImporter(NULL), 
    mCurrentAnimLayer(NULL),
    mShadingMode(SHADING_MODE_SHADED),
    mSupportVBO(pSupportVBO)
{
    // initialize cache start and stop time
    mCache_Start = FBXSDK_TIME_INFINITE;
    mCache_Stop  = FBXSDK_TIME_MINUS_INFINITE;
    mDimensions.Set(100.0, 200.0, 100.0);

   // Create the AFBX SDK manager which is the object allocator for almost 
   // all the classes in the SDK and create the scene.
   InitializeSdkObjects(mSdkManager, mScene);
}

AFBX::~AFBX()
{
    if (mScene)
    {
        UnloadCacheRecursive(mScene);
    }

    // Delete the AFBX SDK manager. All the objects that have been allocated 
    // using the AFBX SDK manager and that haven't been explicitly destroyed 
    // are automatically destroyed at the same time.
    DestroySdkObjects(mSdkManager, true);
}

bool AFBX::IsValid() const
{
    return (mSdkManager != 0);
}

bool AFBX::Load(const std::string& filename)
{
    bool lResult = false;
    // Make sure that the scene is ready to load.

    mFileName = filename;
    int lFileFormat = -1;
    if (!mSdkManager->GetIOPluginRegistry()->DetectReaderFileFormat(mFileName.c_str(), lFileFormat) )
    {
       // Unrecognizable file format. Try to fall back to FbxImporter::eAFBX_BINARY
       lFileFormat = mSdkManager->GetIOPluginRegistry()->FindReaderIDByDescription( "AFBX binary (*.fbx)" );
    }

    if (!mImporter) mImporter = FbxImporter::Create(mSdkManager,"");
    lResult = mImporter->Initialize(mFileName.c_str(), lFileFormat);

    if (lResult && mImporter->Import(mScene) == true)
    {
        // Convert Axis System to what is used in this example, if needed
        FbxAxisSystem SceneAxisSystem = mScene->GetGlobalSettings().GetAxisSystem();
        FbxAxisSystem OurAxisSystem(FbxAxisSystem::eYAxis, FbxAxisSystem::eParityOdd, FbxAxisSystem::eRightHanded);
        if( SceneAxisSystem != OurAxisSystem )
        {
            OurAxisSystem.ConvertScene(mScene);
        }

        // Convert Unit System to what is used in this example, if needed
        FbxSystemUnit SceneSystemUnit = mScene->GetGlobalSettings().GetSystemUnit();
        if( SceneSystemUnit.GetScaleFactor() != 1.0 )
        {
            //The unit in this example is centimeter.
            FbxSystemUnit::cm.ConvertScene( mScene);
        }

        // Get the list of all the animation stacks.
        mScene->FillAnimStackNameArray(mAnimStackNameArray);
        SetCurrentAnimStack(0);

        // Convert mesh, NURBS and patch into triangle mesh
    	FbxGeometryConverter lGeomConverter(mSdkManager);
    	lGeomConverter.Triangulate(mScene, /*replace*/true);

    	// Split meshes per material, so that we only have one material per mesh (for VBO support)
    	lGeomConverter.SplitMeshesPerMaterial(mScene, /*replace*/true);

        // Bake the scene for one frame
        LoadCacheRecursive(mScene, mCurrentAnimLayer, mFileName.c_str(), mSupportVBO);

        // Convert any .PC2 point cache data into the .MC format for 
        // vertex cache deformer playback.
        PreparePointCacheData(mScene, mCache_Start, mCache_Stop);

        // Initialize the frame period.
        std::cout << "FRAME TIME: " << (int) mScene->GetGlobalSettings().GetTimeMode() << std::endl;
        mFrameTime.SetTime(0, 0, 0, 1, 0, mScene->GetGlobalSettings().GetTimeMode());

        lResult = true;
    }
    else
    {
        std::cout << "Unable to import file " << mFileName.c_str() << 
            "\nError reported: " << mImporter->GetStatus().GetErrorString() << std::endl;

       // Destroy the importer to release the file.
       mImporter->Destroy();
       mImporter = NULL;
    }

    mBBoxMin[0] = mBBoxMin[1] = mBBoxMin[2] =  999999999.0;
    mBBoxMax[0] = mBBoxMax[1] = mBBoxMax[2] = -999999999.0;
    FillSkeletonArrayRecursive(mScene->GetRootNode(), mSkeletonArray);

    mDimensions = mBBoxMax - mBBoxMin;
    //std::cout << "Bounding box " << mBBoxMin[0] << " " << mBBoxMin[1] << " " << mBBoxMin[2] 
    //         << " " << mBBoxMax[0] << " " << mBBoxMax[1] << " " << mBBoxMax[2] << std::endl;
    std::cout << "Size " << mDimensions[0] << " " << mDimensions[1] << " " << mDimensions[2] << std::endl;

    return lResult;
}

bool AFBX::Save(const std::string& filename)
{
    mSdkManager->GetIOSettings()->SetBoolProp(EXP_FBX_EMBEDDED, true);

    // Get the appropriate file format.
    int lFileFormat = mSdkManager->GetIOPluginRegistry()->GetNativeWriterFormat();

    FbxExporter* lExporter = FbxExporter::Create(mSdkManager, "");
    bool lExportStatus = lExporter->Initialize(filename.c_str(), -1, mSdkManager->GetIOSettings());
    if (lExportStatus) lExportStatus = lExporter->Export(mScene);

    return lExportStatus;
}

void AFBX::FillSkeletonArrayRecursive(FbxNode* pNode, FbxArray<FbxNode*>& pSkeletonArray)
{
    if (pNode)
    {
        if (pNode->GetNodeAttribute())
        {
            if (pNode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eSkeleton)
            {
                //std::cout << pNode->GetName() << " " << pNode->GetNodeAttribute()->GetTypeName() << std::endl;
                //FbxDouble3 offset = pNode->PreRotation.Get();
                //FbxDouble3 rotation = pNode->LclRotation.Get();
                //std::cout << offset[0] << " " << offset[1] << " " << offset[2] << std::endl;
                //std::cout << rotation[0] << " " << rotation[1] << " " << rotation[2] << std::endl;
                FbxAMatrix& xform = pNode->EvaluateGlobalTransform();
                FbxVector4 t = xform.GetT();
                mBBoxMin[0] = std::min<float>(mBBoxMin[0], t[0]);
                mBBoxMin[1] = std::min<float>(mBBoxMin[1], t[1]);
                mBBoxMin[2] = std::min<float>(mBBoxMin[2], t[2]);
                mBBoxMax[0] = std::max<float>(mBBoxMax[0], t[0]);
                mBBoxMax[1] = std::max<float>(mBBoxMax[1], t[1]);
                mBBoxMax[2] = std::max<float>(mBBoxMax[2], t[2]);
                pSkeletonArray.Add(pNode);
                mJointMap[pNode->GetNameOnly().Buffer()] = pSkeletonArray.Size() - 1;
            }
        }

        const int lCount = pNode->GetChildCount();
        for (int i = 0; i < lCount; i++)
        {
            FillSkeletonArrayRecursive(pNode->GetChild(i), pSkeletonArray);
        }
    }
}

int AFBX::GetNumLimbs() const
{
    return mSkeletonArray.Size();
}

void AFBX::GetLimbInfo(int i, std::string& jointname, std::string& jointparent, 
    FbxDouble3& offset, bool& isRoot, int& numChannels, std::string& rotOrder) const
{
    FbxNode* fbxnode = mSkeletonArray[i];
    FbxNode* parent = fbxnode->GetParent();
    jointname = fbxnode->GetName();
    jointparent = parent->GetName();
    FbxDouble3 t = fbxnode->LclTranslation.Get();
    offset[0] = t[0]; offset[1] = t[1]; offset[2] = t[2];
    std::cout << jointname << " " << offset[0] << " " << offset[1] << " " << offset[2] << std::endl;
    isRoot = (parent == mScene->GetRootNode());
    numChannels = isRoot? 6 : 3;
    EFbxRotationOrder roo = fbxnode->RotationOrder.Get();
    switch (roo)
    {
    case eEulerXYZ: rotOrder = "xyz";
    case eEulerXZY: rotOrder = "xzy";
    case eEulerYZX: rotOrder = "yzx";
    case eEulerYXZ: rotOrder = "yxz";
    case eEulerZXY: rotOrder = "zxy";
    case eEulerZYX: rotOrder = "zyx";
    }
}

int AFBX::GetNumFrames() const
{
    FbxTime delta = mStop - mStart;
    FbxDouble time = delta.GetFrameCountPrecise();
    return (int) time;
}

void AFBX::SetPose(const ASkeleton& skeleton)
{
    int layer = 1;
    if (layer == mAnimStackNameArray.Size())
    {
        char buff[256];
        sprintf_s(buff, "Stack%d", layer);
        mScene->CreateAnimStack(buff);
        mAnimStackNameArray.Add(new FbxString(buff));
    }
    SetCurrentAnimStack(layer);

	for (int i = 0; i < skeleton.getNumJoints(); i++)
    {
		AJoint* jointnode = skeleton.getJointByID(i);
		if (mJointMap.find(jointnode->getName()) == mJointMap.end()) continue;

		FbxNode* fbxnode = mSkeletonArray[mJointMap[jointnode->getName()]];

        if (i == 0)
        {
			vec3 pos = jointnode->getLocalTranslation();
            FbxDouble3 fbxpos(pos[0], pos[1], pos[2]);
            fbxnode->LclTranslation.Set(fbxpos);
        }

        EFbxRotationOrder roo = fbxnode->RotationOrder;
        FbxDouble3 jrot = fbxnode->PreRotation.Get();
        vec3 jeuler(jrot[0] * Deg2Rad, jrot[1] * Deg2Rad, jrot[2] * Deg2Rad);

        mat3 jmat; // ASN TODO: Cache me
        if (roo == eEulerXYZ) jmat.FromEulerAngles(mat3::ZYX, jeuler);
		if (roo == eEulerXZY) jmat.FromEulerAngles(mat3::YZX, jeuler);
		if (roo == eEulerYXZ) jmat.FromEulerAngles(mat3::ZXY, jeuler);
		if (roo == eEulerYZX) jmat.FromEulerAngles(mat3::XZY, jeuler);
		if (roo == eEulerZXY) jmat.FromEulerAngles(mat3::YXZ, jeuler);
		if (roo == eEulerZYX) jmat.FromEulerAngles(mat3::XYZ, jeuler);

        mat3 rot = jmat.Inverse() * jointnode->getLocalRotation();
        vec3 euler;
		if (roo == eEulerXYZ) rot.ToEulerAngles(mat3::ZYX, euler);
		if (roo == eEulerXZY) rot.ToEulerAngles(mat3::YZX, euler);
		if (roo == eEulerYXZ) rot.ToEulerAngles(mat3::ZXY, euler);
		if (roo == eEulerYZX) rot.ToEulerAngles(mat3::XZY, euler);
		if (roo == eEulerZXY) rot.ToEulerAngles(mat3::YXZ, euler);
		if (roo == eEulerZYX) rot.ToEulerAngles(mat3::XYZ, euler);

        euler = euler * Rad2Deg;
        FbxDouble3 fbxori(euler[0], euler[1], euler[2]);
        fbxnode->LclRotation.Set(fbxori); // simple test

        vec3 offset = jointnode->getLocalTranslation();
        fbxnode->LclTranslation.Set(FbxDouble3(offset[0], offset[1], offset[2]));
    }
}

void AFBX::GetPose(double time, ASkeleton& skeleton)
{
    SetCurrentAnimStack(0);
    vec3 pos = GetRootTranslation(time);
    skeleton.getRootNode()->setLocalTranslation(pos);
    for (int j = 0; j < mSkeletonArray.Size(); j++)
    {
        quat q = GetRotation(time, j);
        skeleton.getJointByID(j)->setLocalRotation(q.ToRotation());
    }
}

// NOTE: We need be moving time forward for the whole skeleton to update
void AFBX::SetPosition(int jointid, const vec3& pos)
{
    if (jointid >= mSkeletonArray.Size()) return;

    int layer = 1; // motion is on layer 0, this is layer 1
    if (layer == mAnimStackNameArray.Size())
    {
        char buff[256];
        sprintf_s(buff, "Stack%d", layer);
        mScene->CreateAnimStack(buff);
        mAnimStackNameArray.Add(new FbxString(buff));
    }

    SetCurrentAnimStack(layer);

    FbxNode* node = mSkeletonArray[jointid];
    FbxDouble3 fbxpos(pos[0], pos[1], pos[2]);
    node->LclTranslation.Set(fbxpos);
}

vec3 AFBX::GetPosition(int jointid)
{
    if (jointid >= mSkeletonArray.Size()) return vec3(0,0,0);
    FbxNode* fbxnode = mSkeletonArray[jointid];
    FbxDouble3 fbxpos = fbxnode->LclTranslation.Get();
    return vec3(fbxpos[0], fbxpos[1], fbxpos[2]);
}

vec3 AFBX::GetRootTranslation(double secs)
{
    FbxNode* fbxnode = mSkeletonArray[0];
    FbxAnimCurve* lCurveX = fbxnode->LclTranslation.GetCurve(mCurrentAnimLayer, FBXSDK_CURVENODE_COMPONENT_X);
    FbxAnimCurve* lCurveY = fbxnode->LclTranslation.GetCurve(mCurrentAnimLayer, FBXSDK_CURVENODE_COMPONENT_Y);
    FbxAnimCurve* lCurveZ = fbxnode->LclTranslation.GetCurve(mCurrentAnimLayer, FBXSDK_CURVENODE_COMPONENT_Z);

    FbxTime time;
    time.SetSecondDouble(secs);

    vec3 p;
    p[0] = lCurveX? lCurveX->Evaluate(time) : 0.0;
    p[1] = lCurveY ? lCurveY->Evaluate(time) : 0.0;
    p[2] = lCurveZ ? lCurveZ->Evaluate(time) : 0.0;
    return p;
}

quat AFBX::GetRotation(double secs, int limbid)
{
    FbxTime time;
    time.SetSecondDouble(secs);

    FbxNode* node = mSkeletonArray[limbid];
    FbxAnimCurve* lCurveX = 0;
    FbxAnimCurve* lCurveY = 0;
    FbxAnimCurve* lCurveZ = 0;
    lCurveX = node->LclRotation.GetCurve(mCurrentAnimLayer, FBXSDK_CURVENODE_COMPONENT_X);
    lCurveY = node->LclRotation.GetCurve(mCurrentAnimLayer, FBXSDK_CURVENODE_COMPONENT_Y);
    lCurveZ = node->LclRotation.GetCurve(mCurrentAnimLayer, FBXSDK_CURVENODE_COMPONENT_Z);
    int numKeys = lCurveX? lCurveX->KeyGetCount() : 0;

    // I might need to estimate the time using the frame and framerate and then sample each curve
    vec3 euler;
    euler[0] = lCurveX? lCurveX->Evaluate(time) * Deg2Rad : 0.0;
    euler[1] = lCurveY ? lCurveY->Evaluate(time) * Deg2Rad : 0.0;
    euler[2] = lCurveZ ? lCurveZ->Evaluate(time) * Deg2Rad : 0.0;

    EFbxRotationOrder roo = node->RotationOrder;
    mat3 mat; 
    if (roo == eEulerXYZ) mat.FromEulerAngles(mat3::ZYX, euler);
	if (roo == eEulerXZY) mat.FromEulerAngles(mat3::YZX, euler);
	if (roo == eEulerYXZ) mat.FromEulerAngles(mat3::ZXY, euler);
	if (roo == eEulerYZX) mat.FromEulerAngles(mat3::XZY, euler);
	if (roo == eEulerZXY) mat.FromEulerAngles(mat3::YXZ, euler);
	if (roo == eEulerZYX) mat.FromEulerAngles(mat3::XYZ, euler);

    FbxDouble3 jrot = node->PreRotation.Get();
    vec3 jeuler(jrot[0] * Deg2Rad, jrot[1] * Deg2Rad, jrot[2] * Deg2Rad);
    mat3 jmat; // ASN TODO: Cache me
	if (roo == eEulerXYZ) jmat.FromEulerAngles(mat3::ZYX, jeuler);
	if (roo == eEulerXZY) jmat.FromEulerAngles(mat3::YZX, jeuler);
	if (roo == eEulerYXZ) jmat.FromEulerAngles(mat3::ZXY, jeuler);
	if (roo == eEulerYZX) jmat.FromEulerAngles(mat3::XZY, jeuler);
	if (roo == eEulerZXY) jmat.FromEulerAngles(mat3::YXZ, jeuler);
	if (roo == eEulerZYX) jmat.FromEulerAngles(mat3::XYZ, jeuler);

    mat3 rot = jmat * mat;
    quat q = rot.ToQuaternion();
    return q;
}

bool AFBX::SetCurrentAnimStack(int pIndex)
{
    const int lAnimStackCount = mAnimStackNameArray.GetCount();
    if (!lAnimStackCount || pIndex >= lAnimStackCount)
    {
        return false;
    }

    // select the base layer from the animation stack
   FbxAnimStack * lCurrentAnimationStack = mScene->FindMember<FbxAnimStack>(mAnimStackNameArray[pIndex]->Buffer());
   if (lCurrentAnimationStack == NULL)
   {
       // this is a problem. The anim stack should be found in the scene!
       return false;
   }

   // we assume that the first animation layer connected to the animation stack is the base layer
   // (this is the assumption made in the AFBXSDK)
   mCurrentAnimLayer = lCurrentAnimationStack->GetMember<FbxAnimLayer>();
   mSdkManager->GetAnimationEvaluator()->SetContext(lCurrentAnimationStack);

   FbxTakeInfo* lCurrentTakeInfo = mScene->GetTakeInfo(*(mAnimStackNameArray[pIndex]));
   if (lCurrentTakeInfo)
   {
       mStart = lCurrentTakeInfo->mLocalTimeSpan.GetStart();
       mStop = lCurrentTakeInfo->mLocalTimeSpan.GetStop();
   }
   else
   {
       // Take the time line value
       FbxTimeSpan lTimeLineTimeSpan;
       mScene->GetGlobalSettings().GetTimelineDefaultTimeSpan(lTimeLineTimeSpan);

       mStart = lTimeLineTimeSpan.GetStart();
       mStop  = lTimeLineTimeSpan.GetStop();
   }

   // check for smallest start with cache start
   if(mCache_Start < mStart)
       mStart = mCache_Start;

   // check for biggest stop with cache stop
   if(mCache_Stop  > mStop)  
       mStop  = mCache_Stop;

   return true;
}

FbxAMatrix AFBX::Draw(double timeInSecs, int layer)
{
    FbxTime time;
    time.SetSecondDouble(timeInSecs);
    SetCurrentAnimStack(layer);

    FbxAMatrix lDummyGlobalPosition;
    DrawNodeRecursive(mScene->GetRootNode(), time, 0, lDummyGlobalPosition, NULL, mShadingMode);
    return lDummyGlobalPosition;
}

void AFBX::SetShadingMode(ShadingMode pMode)
{
    mShadingMode = pMode;
}

void AFBX::PreparePointCacheData(FbxScene* pScene, FbxTime &pCache_Start, FbxTime &pCache_Stop)
{
    // This function show how to cycle through scene elements in a linear way.
    const int lNodeCount = pScene->GetSrcObjectCount<FbxNode>();
    FbxStatus lStatus;

    for (int lIndex=0; lIndex<lNodeCount; lIndex++)
    {
        FbxNode* lNode = pScene->GetSrcObject<FbxNode>(lIndex);

        if (lNode->GetGeometry()) 
        {
            int i, lVertexCacheDeformerCount = lNode->GetGeometry()->GetDeformerCount(FbxDeformer::eVertexCache);

            // There should be a maximum of 1 Vertex Cache Deformer for the moment
            lVertexCacheDeformerCount = lVertexCacheDeformerCount > 0 ? 1 : 0;

            for (i=0; i<lVertexCacheDeformerCount; ++i )
            {
                // Get the Point Cache object
                FbxVertexCacheDeformer* lDeformer = static_cast<FbxVertexCacheDeformer*>(lNode->GetGeometry()->GetDeformer(i, FbxDeformer::eVertexCache));
                if( !lDeformer ) continue;
                FbxCache* lCache = lDeformer->GetCache();
                if( !lCache ) continue;

                // Process the point cache data only if the constraint is active
                if (lDeformer->IsActive())
                {
                    if (lCache->GetCacheFileFormat() == FbxCache::eMaxPointCacheV2)
                    {
                        // This code show how to convert from PC2 to MC point cache format
                        // turn it on if you need it.
#if 0 
                        if (!lCache->ConvertFromPC2ToMC(FbxCache::eMCOneFile, 
                            FbxTime::GetFrameRate(pScene->GetGlobalTimeSettings().GetTimeMode())))
                        {
                            // Conversion failed, retrieve the error here
                            FbxString lTheErrorIs = lCache->GetStaus().GetErrorString();
                        }
#endif
                    }
                    else if (lCache->GetCacheFileFormat() == FbxCache::eMayaCache)
                    {
                        // This code show how to convert from MC to PC2 point cache format
                        // turn it on if you need it.
                        //#if 0 
                        if (!lCache->ConvertFromMCToPC2(FbxTime::GetFrameRate(pScene->GetGlobalSettings().GetTimeMode()), 0, &lStatus))
                        {
                            // Conversion failed, retrieve the error here
                            FbxString lTheErrorIs = lStatus.GetErrorString();
                        }
                        //#endif
                    }


                    // Now open the cache file to read from it
                    if (!lCache->OpenFileForRead(&lStatus))
                    {
                        // Cannot open file 
                        FbxString lTheErrorIs = lStatus.GetErrorString();

                        // Set the deformer inactive so we don't play it back
                        lDeformer->SetActive(false);
                    }
    				else
    				{
    					// get the start and stop time of the cache
    					int lChannelCount = lCache->GetChannelCount();
    					
    					for (int iChannelNo=0; iChannelNo < lChannelCount; iChannelNo++)
    					{
    						FbxTime lChannel_Start;
    						FbxTime lChannel_Stop;

    						if(lCache->GetAnimationRange(iChannelNo, lChannel_Start, lChannel_Stop))
    						{
    							// get the smallest start time
    							if(lChannel_Start < pCache_Start) pCache_Start = lChannel_Start;

    							// get the biggest stop time
    							if(lChannel_Stop  > pCache_Stop)  pCache_Stop  = lChannel_Stop;
    						}
    					}
    				}
                }
            }
        }
    }
}

// Load a texture file (TGA only now) into GPU and return the texture object name
bool AFBX::LoadTextureFromFile(const FbxString & pFilePath, unsigned int & pTextureObject)
{
   AImage img;
   if (img.load(pFilePath.Buffer()))
   {
       // Transfer the texture date into GPU
       glGenTextures(1, &pTextureObject);
       glBindTexture(GL_TEXTURE_2D, pTextureObject);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
       glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); //GL_REPLACE);
       glTexImage2D(GL_TEXTURE_2D, 0, 3, img.width(), img.height(),
           0, GL_RGB, GL_UNSIGNED_BYTE, img.data());
       glBindTexture(GL_TEXTURE_2D, 0);

       GLenum error;
       if ((error = glGetError()) != GL_NO_ERROR)
       {
          printf("TEXTURE ERROR: %s\n", gluErrorString(error));
       }
       else return true;
   }
    return false;
}

// Bake node attributes and materials under this node recursively.
// Currently only mesh, light and material.
void AFBX::LoadCacheRecursive(FbxNode * pNode, FbxAnimLayer * pAnimLayer, bool pSupportVBO)
{
    // Bake material and hook as user data.
    const int lMaterialCount = pNode->GetMaterialCount();
    for (int lMaterialIndex = 0; lMaterialIndex < lMaterialCount; ++lMaterialIndex)
    {
        FbxSurfaceMaterial * lMaterial = pNode->GetMaterial(lMaterialIndex);
        if (lMaterial && !lMaterial->GetUserDataPtr())
        {
            FbxAutoPtr<MaterialCache> lMaterialCache(new MaterialCache);
            if (lMaterialCache->Initialize(lMaterial))
            {
                lMaterial->SetUserDataPtr(lMaterialCache.Release());
            }
        }
    }

    FbxNodeAttribute* lNodeAttribute = pNode->GetNodeAttribute();
    if (lNodeAttribute)
    {
        // Bake mesh as VBO(vertex buffer object) into GPU.
        if (lNodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh)
        {
            FbxMesh * lMesh = pNode->GetMesh();
            if (pSupportVBO && lMesh && !lMesh->GetUserDataPtr())
            {
                FbxAutoPtr<VBOMesh> lMeshCache(new VBOMesh);
                if (lMeshCache->Initialize(lMesh))
                {
                    lMesh->SetUserDataPtr(lMeshCache.Release());
                }
            }
        }
        // Bake light properties.
        else if (lNodeAttribute->GetAttributeType() == FbxNodeAttribute::eLight)
        {
            FbxLight * lLight = pNode->GetLight();
            if (lLight && !lLight->GetUserDataPtr())
            {
                FbxAutoPtr<LightCache> lLightCache(new LightCache);
                if (lLightCache->Initialize(lLight, pAnimLayer))
                {
                    lLight->SetUserDataPtr(lLightCache.Release());
                }
            }
        }
    }

    const int lChildCount = pNode->GetChildCount();
    for (int lChildIndex = 0; lChildIndex < lChildCount; ++lChildIndex)
    {
        LoadCacheRecursive(pNode->GetChild(lChildIndex), pAnimLayer, pSupportVBO);
    }
}

// Unload the cache and release the memory under this node recursively.
void AFBX::UnloadCacheRecursive(FbxNode * pNode)
{
    // Unload the material cache
    const int lMaterialCount = pNode->GetMaterialCount();
    for (int lMaterialIndex = 0; lMaterialIndex < lMaterialCount; ++lMaterialIndex)
    {
        FbxSurfaceMaterial * lMaterial = pNode->GetMaterial(lMaterialIndex);
        if (lMaterial && lMaterial->GetUserDataPtr())
        {
            MaterialCache * lMaterialCache = static_cast<MaterialCache *>(lMaterial->GetUserDataPtr());
            lMaterial->SetUserDataPtr(NULL);
            delete lMaterialCache;
        }
    }

    FbxNodeAttribute* lNodeAttribute = pNode->GetNodeAttribute();
    if (lNodeAttribute)
    {
        // Unload the mesh cache
        if (lNodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh)
        {
            FbxMesh * lMesh = pNode->GetMesh();
            if (lMesh && lMesh->GetUserDataPtr())
            {
                VBOMesh * lMeshCache = static_cast<VBOMesh *>(lMesh->GetUserDataPtr());
                lMesh->SetUserDataPtr(NULL);
                delete lMeshCache;
            }
        }
        // Unload the light cache
        else if (lNodeAttribute->GetAttributeType() == FbxNodeAttribute::eLight)
        {
            FbxLight * lLight = pNode->GetLight();
            if (lLight && lLight->GetUserDataPtr())
            {
                LightCache * lLightCache = static_cast<LightCache *>(lLight->GetUserDataPtr());
                lLight->SetUserDataPtr(NULL);
                delete lLightCache;
            }
        }
    }

    const int lChildCount = pNode->GetChildCount();
    for (int lChildIndex = 0; lChildIndex < lChildCount; ++lChildIndex)
    {
        UnloadCacheRecursive(pNode->GetChild(lChildIndex));
    }
}

// Bake node attributes and materials for this scene and load the textures.
void AFBX::LoadCacheRecursive(FbxScene * pScene, FbxAnimLayer * pAnimLayer, const char * pFbxFileName, bool pSupportVBO)
{
    // Load the textures into GPU, only for file texture now
    const int lTextureCount = pScene->GetTextureCount();
    for (int lTextureIndex = 0; lTextureIndex < lTextureCount; ++lTextureIndex)
    {
        FbxTexture * lTexture = pScene->GetTexture(lTextureIndex);
        FbxFileTexture * lFileTexture = FbxCast<FbxFileTexture>(lTexture);
        if (lFileTexture && !lFileTexture->GetUserDataPtr())
        {
            // Try to load the texture from absolute path
            const FbxString lFileName = lFileTexture->GetFileName();
            FBXSDK_printf("Attempting to load: %s\n", lFileName.Buffer());

            GLuint lTextureObject = 0;
            bool lStatus = LoadTextureFromFile(lFileName, lTextureObject);

            const FbxString lAbsFbxFileName = FbxPathUtils::Resolve(pFbxFileName);
            const FbxString lAbsFolderName = FbxPathUtils::GetFolderName(lAbsFbxFileName);
            if (!lStatus)
            {
                // Load texture from relative file name (relative to AFBX file)
                const FbxString lResolvedFileName = FbxPathUtils::Bind(lAbsFolderName, lFileTexture->GetRelativeFileName());
                lStatus = LoadTextureFromFile(lResolvedFileName, lTextureObject);
            }

            if (!lStatus)
            {
                // Load texture from file name only (relative to AFBX file)
                const FbxString lTextureFileName = FbxPathUtils::GetFileName(lFileName);
                const FbxString lResolvedFileName = FbxPathUtils::Bind(lAbsFolderName, lTextureFileName);
                lStatus = LoadTextureFromFile(lResolvedFileName, lTextureObject);
            }

            if (!lStatus)
            {
                FBXSDK_printf("Failed to load texture file: %s\n", lFileName.Buffer());
                continue;
            }

            if (lStatus)
            {
                GLuint * lTextureName = new GLuint(lTextureObject);
                lFileTexture->SetUserDataPtr(lTextureName);
            }
        }
    }

    LoadCacheRecursive(pScene->GetRootNode(), pAnimLayer, pSupportVBO);
}

    // Unload the cache and release the memory fro this scene and release the textures in GPU
void AFBX::UnloadCacheRecursive(FbxScene * pScene)
{
    const int lTextureCount = pScene->GetTextureCount();
    for (int lTextureIndex = 0; lTextureIndex < lTextureCount; ++lTextureIndex)
    {
        FbxTexture * lTexture = pScene->GetTexture(lTextureIndex);
        FbxFileTexture * lFileTexture = FbxCast<FbxFileTexture>(lTexture);
        if (lFileTexture && lFileTexture->GetUserDataPtr())
        {
            GLuint * lTextureName = static_cast<GLuint *>(lFileTexture->GetUserDataPtr());
            lFileTexture->SetUserDataPtr(NULL);
            glDeleteTextures(1, lTextureName);
            delete lTextureName;
        }
    }

    UnloadCacheRecursive(pScene->GetRootNode());
}

void AFBX::InitializeSdkObjects(FbxManager*& pManager, FbxScene*& pScene)
{
    //The first thing to do is to create the AFBX Manager which is the object allocator for almost all the classes in the SDK
    pManager = FbxManager::Create();
    if( !pManager )
    {
        FBXSDK_printf("Error: Unable to create FBX Manager!\n");
        exit(1);
    }
    else FBXSDK_printf("Autodesk FBX SDK version %s\n", pManager->GetVersion());

    //Create an IOSettings object. This object holds all import/export settings.
    FbxIOSettings* ios = FbxIOSettings::Create(pManager, IOSROOT);
    pManager->SetIOSettings(ios);

    //Load plugins from the executable directory (optional)
    FbxString lPath = FbxGetApplicationDirectory();
    pManager->LoadPluginsDirectory(lPath.Buffer());

    //Create an AFBX scene. This object holds most objects imported/exported from/to files.
    pScene = FbxScene::Create(pManager, "My Scene");
    if( !pScene )
    {
        FBXSDK_printf("Error: Unable to create FBX scene!\n");
        exit(1);
    }
}

void AFBX::DestroySdkObjects(FbxManager* pManager, bool pExitStatus)
{
    //Delete the AFBX Manager. All the objects that have been allocated using the FBX Manager and 
    //that haven't been explicitly destroyed are also automatically destroyed.
    if( pManager ) pManager->Destroy();
    if( pExitStatus ) FBXSDK_printf("Program Success!\n");
}

bool AFBX::SaveScene(FbxManager* pManager, FbxDocument* pScene, const char* pFilename, int pFileFormat, bool pEmbedMedia)
{
    int lMajor, lMinor, lRevision;
    bool lStatus = true;

    // Create an exporter.
    FbxExporter* lExporter = FbxExporter::Create(pManager, "");

    if( pFileFormat < 0 || pFileFormat >= pManager->GetIOPluginRegistry()->GetWriterFormatCount() )
    {
        // Write in fall back format in less no ASCII format found
        pFileFormat = pManager->GetIOPluginRegistry()->GetNativeWriterFormat();

        //Try to export in ASCII if possible
        int lFormatIndex, lFormatCount = pManager->GetIOPluginRegistry()->GetWriterFormatCount();

        for (lFormatIndex=0; lFormatIndex<lFormatCount; lFormatIndex++)
        {
            if (pManager->GetIOPluginRegistry()->WriterIsFBX(lFormatIndex))
            {
                FbxString lDesc =pManager->GetIOPluginRegistry()->GetWriterFormatDescription(lFormatIndex);
                const char *lASCII = "ascii";
                if (lDesc.Find(lASCII)>=0)
                {
                    pFileFormat = lFormatIndex;
                    break;
                }
            }
        } 
    }

    // Set the export states. By default, the export states are always set to 
    // true except for the option eEXPORT_TEXTURE_AS_EMBEDDED. The code below 
    // shows how to change these states.
    IOS_REF.SetBoolProp(EXP_FBX_MATERIAL,        true);
    IOS_REF.SetBoolProp(EXP_FBX_TEXTURE,         true);
    IOS_REF.SetBoolProp(EXP_FBX_EMBEDDED,        pEmbedMedia);
    IOS_REF.SetBoolProp(EXP_FBX_SHAPE,           true);
    IOS_REF.SetBoolProp(EXP_FBX_GOBO,            true);
    IOS_REF.SetBoolProp(EXP_FBX_ANIMATION,       true);
    IOS_REF.SetBoolProp(EXP_FBX_GLOBAL_SETTINGS, true);

    // Initialize the exporter by providing a filename.
    if(lExporter->Initialize(pFilename, pFileFormat, pManager->GetIOSettings()) == false)
    {
        FBXSDK_printf("Call to FbxExporter::Initialize() failed.\n");
        FBXSDK_printf("Error returned: %s\n\n", lExporter->GetStatus().GetErrorString());
        return false;
    }

    FbxManager::GetFileFormatVersion(lMajor, lMinor, lRevision);
    FBXSDK_printf("FBX file format version %d.%d.%d\n\n", lMajor, lMinor, lRevision);

    // Export the scene.
    lStatus = lExporter->Export(pScene); 

    // Destroy the exporter.
    lExporter->Destroy();
    return lStatus;
}

bool AFBX::LoadScene(FbxManager* pManager, FbxDocument* pScene, const char* pFilename)
{
    int lFileMajor, lFileMinor, lFileRevision;
    int lSDKMajor,  lSDKMinor,  lSDKRevision;
    //int lFileFormat = -1;
    int i, lAnimStackCount;
    bool lStatus;
    char lPassword[1024];

    // Get the file version number generate by the AFBX SDK.
    FbxManager::GetFileFormatVersion(lSDKMajor, lSDKMinor, lSDKRevision);

    // Create an importer.
    FbxImporter* lImporter = FbxImporter::Create(pManager,"");

    // Initialize the importer by providing a filename.
    const bool lImportStatus = lImporter->Initialize(pFilename, -1, pManager->GetIOSettings());
    lImporter->GetFileVersion(lFileMajor, lFileMinor, lFileRevision);

    if( !lImportStatus )
    {
        FbxString error = lImporter->GetStatus().GetErrorString();
        FBXSDK_printf("Call to FbxImporter::Initialize() failed.\n");
        FBXSDK_printf("Error returned: %s\n\n", error.Buffer());

        if (lImporter->GetStatus().GetCode() == FbxStatus::eInvalidFileVersion)
        {
            FBXSDK_printf("FBX file format version for this FBX SDK is %d.%d.%d\n", lSDKMajor, lSDKMinor, lSDKRevision);
            FBXSDK_printf("FBX file format version for file '%s' is %d.%d.%d\n\n", pFilename, lFileMajor, lFileMinor, lFileRevision);
        }

        return false;
    }

    FBXSDK_printf("FBX file format version for this FBX SDK is %d.%d.%d\n", lSDKMajor, lSDKMinor, lSDKRevision);

    if (lImporter->IsFBX())
    {
        FBXSDK_printf("FBX file format version for file '%s' is %d.%d.%d\n\n", pFilename, lFileMajor, lFileMinor, lFileRevision);

        // From this point, it is possible to access animation stack information without
        // the expense of loading the entire file.

        FBXSDK_printf("Animation Stack Information\n");

        lAnimStackCount = lImporter->GetAnimStackCount();

        FBXSDK_printf("    Number of Animation Stacks: %d\n", lAnimStackCount);
        FBXSDK_printf("    Current Animation Stack: \"%s\"\n", lImporter->GetActiveAnimStackName().Buffer());
        FBXSDK_printf("\n");

        for(i = 0; i < lAnimStackCount; i++)
        {
            FbxTakeInfo* lTakeInfo = lImporter->GetTakeInfo(i);

            FBXSDK_printf("    Animation Stack %d\n", i);
            FBXSDK_printf("         Name: \"%s\"\n", lTakeInfo->mName.Buffer());
            FBXSDK_printf("         Description: \"%s\"\n", lTakeInfo->mDescription.Buffer());

            // Change the value of the import name if the animation stack should be imported 
            // under a different name.
            FBXSDK_printf("         Import Name: \"%s\"\n", lTakeInfo->mImportName.Buffer());

            // Set the value of the import state to false if the animation stack should be not
            // be imported. 
            FBXSDK_printf("         Import State: %s\n", lTakeInfo->mSelect ? "true" : "false");
            FBXSDK_printf("\n");
        }

        // Set the import states. By default, the import states are always set to 
        // true. The code below shows how to change these states.
        IOS_REF.SetBoolProp(IMP_FBX_MATERIAL,        true);
        IOS_REF.SetBoolProp(IMP_FBX_TEXTURE,         true);
        IOS_REF.SetBoolProp(IMP_FBX_LINK,            true);
        IOS_REF.SetBoolProp(IMP_FBX_SHAPE,           true);
        IOS_REF.SetBoolProp(IMP_FBX_GOBO,            true);
        IOS_REF.SetBoolProp(IMP_FBX_ANIMATION,       true);
        IOS_REF.SetBoolProp(IMP_FBX_GLOBAL_SETTINGS, true);
    }

    // Import the scene.
    lStatus = lImporter->Import(pScene);

    if(lStatus == false && lImporter->GetStatus().GetCode() == FbxStatus::ePasswordError)
    {
        FBXSDK_printf("Please enter password: ");

        lPassword[0] = '\0';

        FBXSDK_CRT_SECURE_NO_WARNING_BEGIN
        scanf("%s", lPassword);
        FBXSDK_CRT_SECURE_NO_WARNING_END

        FbxString lString(lPassword);

        IOS_REF.SetStringProp(IMP_FBX_PASSWORD,      lString);
        IOS_REF.SetBoolProp(IMP_FBX_PASSWORD_ENABLE, true);

        lStatus = lImporter->Import(pScene);

        if(lStatus == false && lImporter->GetStatus().GetCode() == FbxStatus::ePasswordError)
        {
            FBXSDK_printf("\nPassword is wrong, import aborted.\n");
        }
    }

    // Destroy the importer.
    lImporter->Destroy();

    return lStatus;
}

void AFBX::SetFPS(double fps)
{
    FbxTime::SetGlobalTimeMode(FbxTime::eCustom, fps);

    double newfps = FbxTime::GetFrameRate(mScene->GetGlobalSettings().GetTimeMode());
    std::cout << "FRAME TIME: " << newfps << std::endl;
    mFrameTime.SetTime(0, 0, 0, 1, 0, mScene->GetGlobalSettings().GetTimeMode());
}

void AFBX::SetDuration(int numFrames)
{
    if (mAnimStackNameArray.Size() == 0) return;

    int pIndex = 0;
    FbxTakeInfo* lCurrentTakeInfo = mScene->GetTakeInfo(*(mAnimStackNameArray[pIndex]));
    if (lCurrentTakeInfo)
    {
        FbxTime start, end;
        start.SetTime(0, 0, 0, 1, 0, mScene->GetGlobalSettings().GetTimeMode());
        end.SetFrame(numFrames);
        lCurrentTakeInfo->mLocalTimeSpan.Set(start, end);

        mStart = lCurrentTakeInfo->mLocalTimeSpan.GetStart();
        mStop = lCurrentTakeInfo->mLocalTimeSpan.GetStop();
    }
}

FbxVector4 AFBX::GetDimensions() const
{
    return mDimensions;
}

ASkeleton AFBX::ExportSkeleton() const
{
    ASkeleton skeleton;
	// create all the joints of the skeleton
    for (int i = 0; i < mSkeletonArray.Size(); i++)
    {
        FbxNode* fbxnode = mSkeletonArray[i];

        std::string jointname;
        std::string jointparent;
        FbxDouble3 offset;
        bool isRoot;
        int numChannels;
        std::string rotOrder;
        GetLimbInfo(i, jointname, jointparent, offset, isRoot, numChannels, rotOrder);

		AJoint* joint = new AJoint(jointname);
        joint->setLocalTranslation(vec3(offset[0], offset[1], offset[2]));
        joint->setNumChannels(numChannels);
        joint->setRotationOrder(rotOrder);
        skeleton.addJoint(joint, isRoot);
    }

    // set joint parents
	for (int i = 0; i < mSkeletonArray.Size(); i++)
    {
        FbxNode* node = mSkeletonArray[i];
        const char* jointname = node->GetName();
        
		AJoint* joint = skeleton.getJointByName(jointname);
        if (skeleton.getRootNode() != joint)
        {
            FbxNode* parent = node->GetParent();
            const char* jointparent = parent->GetName();
			AJoint* jparent = skeleton.getJointByName(jointparent);
			AJoint::Attach(jparent, joint);
        }
    }
    return skeleton;
}

AMotion AFBX::ExportMotion(double startTime, double endTime, double fps)
{
    SetFPS(fps);
    SetCurrentAnimStack(0); // assumes animation is on layer 0
    double dt = 1.0 / fps;

    AMotion motion;
    for (double t = startTime; t <= endTime; t+=dt)
    {
        vec3 pos = GetRootTranslation(t); 
        motion.mRootMotion.appendKey(pos);

        for (int j = 0; j < mSkeletonArray.Size(); j++)
        {
            quat q = GetRotation(t, j);
            motion.mMotion[j].appendKey(q);
        }
    }
    return motion;
}