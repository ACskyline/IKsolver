#include "aIKController.h"
#include "GL/glut.h"

#include "aActor.h"

#include "aMatrix.h"//me

#pragma warning (disable : 4018)

int IKController::gIKmaxIterations = 5;
double IKController::gIKEpsilon = 0.1;

// AIKchain class functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////
AIKchain::AIKchain()
{
	mWeight0 = 0.1;
}

AIKchain::~AIKchain()
{

}

AJoint* AIKchain::getJoint(int index) 
{ 
	return mChain[index]; 
}

void AIKchain::setJoint(int index, AJoint* pJoint) 
{ 
	mChain[index] = pJoint; 
}

double AIKchain::getWeight(int index) 
{ 
	return mWeights[index]; 
}

void AIKchain::setWeight(int index, double weight) 
{ 
	mWeights[index] = weight; 
}

int AIKchain::getSize() 
{ 
	return mChain.size(); 
}

std::vector<AJoint*>& AIKchain::getChain() 
{ 
	return mChain; 
}

std::vector<double>& AIKchain::getWeights() 
{ 
	return mWeights; 
}

void AIKchain::setChain(std::vector<AJoint*> chain) 
{
	mChain = chain; 
}

void AIKchain::setWeights(std::vector<double> weights) 
{ 
	mWeights = weights; 
}

// AIKController class functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////

IKController::IKController()
{
	m_pActor = NULL;
	m_pSkeleton = NULL;
	mvalidLimbIKchains = false;
	mvalidCCDIKchains = false;

	mvalidPseudoInvIKchains = false;//me

	// Limb IK
	m_pEndJoint = NULL;
	m_pMiddleJoint = NULL;
	m_pBaseJoint = NULL;
	m_rotationAxis = vec3(0.0, 1.0, 0.0);

	ATransform desiredTarget = ATransform();
	mTarget0.setLocal2Parent(desiredTarget);  // target associated with end joint
	mTarget1.setLocal2Parent(desiredTarget);  // optional target associated with middle joint - used to specify rotation of middle joint about end/base axis
	mTarget0.setLocal2Global(desiredTarget);
	mTarget1.setLocal2Global(desiredTarget);

	//CCD IK
	mWeight0 = 0.1;  // default joint rotation weight value

}

IKController::~IKController()
{
}

ASkeleton* IKController::getSkeleton()
{
	return m_pSkeleton;
}

const ASkeleton* IKController::getSkeleton() const
{
	return m_pSkeleton;
}

ASkeleton* IKController::getIKSkeleton()
{
	return &mIKSkeleton;
}

const ASkeleton* IKController::getIKSkeleton() const
{
	return &mIKSkeleton;
}

AActor* IKController::getActor()
{
	return m_pActor;
}

void IKController::setActor(AActor* actor)

{
	m_pActor = actor;
	m_pSkeleton = m_pActor->getSkeleton();
}


AIKchain IKController::createIKchain(int endJointID, int desiredChainSize, ASkeleton* pSkeleton)
{
	// TODO: given the end joint ID and the desired size (i.e. length) of the IK chain, 
	// 1. add the corresponding skeleton joint pointers to the AIKChain "chain" vector data member starting with the end joint
	// 2. also add weight values to the associated AIKChain "weights" vector data member for use in the CCD IK implemention
	// Note: desiredChainSize = -1 should create an IK chain of maximum length (i.e. where the last chain joint is the joint before the root joint)
	bool getMaxSize = false;

	int EndJointID = endJointID;
	std::vector<AJoint*> chain;
	std::vector<double> weights;

	chain.clear();
	weights.clear();
	if (desiredChainSize == -1)
		getMaxSize = true;

	if ((EndJointID >= 0) && (EndJointID < pSkeleton->getNumJoints()))
	{
		AJoint* pJoint = pSkeleton->getJointByID(endJointID);

		// TODO: add code here to generate chain of desired size or terminate at the joint before root joint, so that root will not change during IK	
		// also add weight values to corresponding weights vector  (default value = 0.1)
		while (pJoint != pSkeleton->getRootNode())
		{
			chain.push_back(pJoint);
			weights.push_back(0.1);

			pJoint = pJoint->getParent();

			if (!getMaxSize && --desiredChainSize == 0)
				break;
		}
	}
	AIKchain result;
	result.setChain(chain);
	result.setWeights(weights);

	return result;
}



bool IKController::IKSolver_Limb(int endJointID, const ATarget& target)
{
	// Implements the analytic/geometric IK method assuming a three joint limb  

	if (!mvalidLimbIKchains)
	{
		mvalidLimbIKchains = createLimbIKchains();
		//assert(mvalidLimbIKchains);
	}

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;

	switch (endJointID)
	{
	case mLhandID:
		mLhandTarget = target;
		computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
		break;
	case mRhandID:
		mRhandTarget = target;
		computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
		break;
	case mLfootID:
		mLfootTarget = target;
		computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
		break;
	case mRfootID:
		mRfootTarget = target;
		computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
		break;
	case mRootID:
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
		computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
		computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
		computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
		break;
	default:
		mIKchain = createIKchain(endJointID, 3, &mIKSkeleton);
		computeLimbIK(target, mIKchain, axisY, &mIKSkeleton);
		break;
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}



int IKController::createLimbIKchains()
{
	bool validChains = false;
	int desiredChainSize = 3;

	// create IK chains for Lhand, Rhand, Lfoot and Rfoot 
	mLhandIKchain = createIKchain(mLhandID, desiredChainSize, &mIKSkeleton);
	mRhandIKchain = createIKchain(mRhandID, desiredChainSize, &mIKSkeleton);
	mLfootIKchain = createIKchain(mLfootID, desiredChainSize, &mIKSkeleton);
	mRfootIKchain = createIKchain(mRfootID, desiredChainSize, &mIKSkeleton);
	
	if (mLhandIKchain.getSize() == 3 && mRhandIKchain.getSize() == 3 && mLfootIKchain.getSize() == 3 && mRfootIKchain.getSize() == 3)
	{
		validChains = true;
		
		// initalize end joint target transforms for Lhand, Rhand, Lfoot and Rfoot based on current position and orientation of joints
		mIKSkeleton.copyTransforms(m_pSkeleton);
		mLhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mLhandID)->getLocal2Global());
		mRhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mRhandID)->getLocal2Global());
		mLfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mLfootID)->getLocal2Global());
		mRfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mRfootID)->getLocal2Global());
	}

	return validChains;
}




int IKController::computeLimbIK(ATarget target, AIKchain& IKchain, const vec3 midJointAxis, ASkeleton* pIKSkeleton)
{
	// TODO: Implement the analytic/geometric IK method assuming a three joint limb  
	// The actual position of the end joint should match the target position within some episilon error 
	// the variable "midJointAxis" contains the rotation axis for the middle joint
	
	bool result = false;
	int endJointID;
	mTarget0 = target;

	if (IKchain.getSize() > 0)
		 endJointID = IKchain.getJoint(0)->getID();
	else endJointID = -1;

	if ((endJointID >= 0) && (endJointID < pIKSkeleton->getNumJoints()))
	{
		m_pEndJoint = IKchain.getJoint(0);
		m_pMiddleJoint = IKchain.getJoint(1);
		m_pBaseJoint = IKchain.getJoint(2);

		//TODO:
		// 1. compute error vector between target and end joint
		//vec3 et = target.getGlobalTranslation() - m_pEndJoint->getGlobalTranslation(); // not used

		// 2. compute vector between end Joint and base joint
		//vec3 be = m_pEndJoint->getGlobalTranslation() - m_pBaseJoint->getGlobalTranslation(); // not used

		// 3. compute vector between target and base joint
		vec3 bt = target.getGlobalTranslation() - m_pBaseJoint->getGlobalTranslation();

		double l1 = m_pMiddleJoint->getLocalTranslation().Length();
		double l2 = m_pEndJoint->getLocalTranslation().Length();

		double rd = bt.Length();
		double cosPhi = (l1*l1 + l2*l2 - rd*rd) / (2 * l1*l2);
		if (cosPhi > 1) cosPhi = 1; else if (cosPhi < -1) cosPhi = -1;
		double Phi = acos(cosPhi);

		// 4. Compute desired angle for middle joint 
		//double sinTheta1z = l2*sin(Phi) / rd; // not used
		//if (sinTheta1z > 1) sinTheta1z = 1; else if (sinTheta1z < -1) sinTheta1z = -1; // not used
		//double Theta1z = asin(sinTheta1z); // not used
		double Theta2z = Phi - M_PI;

		// 5. given desired angle and midJointAxis, compute new local middle joint rotation matrix and update joint transform
		m_pMiddleJoint->setLocalRotation(mat3::Rotation3D(midJointAxis, -Theta2z));
		m_pMiddleJoint->updateTransform();

		// 6. compute vector between target and base joint
		vec3 be = m_pEndJoint->getGlobalTranslation() - m_pBaseJoint->getGlobalTranslation();

		// 7. Compute base joint rotation axis (in global coords) and desired angle
		vec3 axis = be.Cross(bt).Normalize();
		double cosAngle = Dot(be.Normalize(), bt.Normalize());
		if (cosAngle > 1) cosAngle = 1; else if (cosAngle < -1) cosAngle = -1;
		double angle = acos(cosAngle);

		// 8. transform base joint rotation axis to local coordinates
		if(m_pBaseJoint->getParent()!=nullptr) axis = m_pBaseJoint->getParent()->getLocal2Global().Inverse().Rotate(axis).Normalize();
		else axis = m_pBaseJoint->getLocal2Global().Inverse().Rotate(axis).Normalize();
		//printf("Phi:%lf Theta1z:%lf Theta2z:%lf Angle:%lf x:%f y:%f z:%f\n", Phi, Theta1z, Theta2z, angle, axis[0], axis[1], axis[2]);


		// 9. given desired angle and local rotation axis, compute new local rotation matrix and update base joint transform
		//m_pBaseJoint->setLocalRotation(mat3::Rotation3D(axisY, Beta) * mat3::Rotation3D(axisZ, Gama) * mat3::Rotation3D(axisX, Alpha) *mat3::Rotation3D(axisZ, Theta1z));
		m_pBaseJoint->setLocalRotation(mat3::Rotation3D(axis, angle)*m_pBaseJoint->getLocalRotation());
		m_pBaseJoint->updateTransform();
	}
	return result;

}

bool IKController::IKSolver_CCD(int endJointID, const ATarget& target)
{
	// Implements the CCD IK method assuming a three joint limb 

	bool validChains = false;//??? what is this about???

	if (!mvalidCCDIKchains)
	{
		mvalidCCDIKchains = createCCDIKchains();
		//assert(mvalidCCDIKchains);
	}

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;

	switch (endJointID)
	{
	case mLhandID:
		mLhandTarget = target;
		computeCCDIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		break;
	case mRhandID:
		mRhandTarget = target;
		computeCCDIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		break;
	case mLfootID:
		mLfootTarget = target;
		computeCCDIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		break;
	case mRfootID:
		mRfootTarget = target;
		computeCCDIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	case mRootID:
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computeCCDIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		computeCCDIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		computeCCDIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		computeCCDIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	default:
		mIKchain = createIKchain(endJointID, -1, &mIKSkeleton);
		computeCCDIK(target, mIKchain, &mIKSkeleton);
		break;
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}

int IKController::createCCDIKchains()
{
	bool validChains = false;

	int desiredChainSize = -1;  // default of -1 creates IK chain of maximum length from end joint to child joint of root


	// create IK chains for Lhand, Rhand, Lfoot and Rfoot 
	mLhandIKchain = createIKchain(mLhandID, desiredChainSize, &mIKSkeleton);
	mRhandIKchain = createIKchain(mRhandID, desiredChainSize, &mIKSkeleton);
	mLfootIKchain = createIKchain(mLfootID, desiredChainSize, &mIKSkeleton);
	mRfootIKchain = createIKchain(mRfootID, desiredChainSize, &mIKSkeleton);

	if (mLhandIKchain.getSize() > 1 && mRhandIKchain.getSize() > 1 && mLfootIKchain.getSize() > 1 && mRfootIKchain.getSize() > 1)
	{
		validChains = true;

		// initalize end joint target transforms for Lhand, Rhand, Lfoot and Rfoot based on current position and orientation of joints
		mIKSkeleton.copyTransforms(m_pSkeleton);
		mLhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mLhandID)->getLocal2Global());
		mRhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mRhandID)->getLocal2Global());
		mLfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mLfootID)->getLocal2Global());
		mRfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mRfootID)->getLocal2Global());
	}

	return validChains;
}


int IKController::computeCCDIK(ATarget target, AIKchain& IKchain, ASkeleton* pIKSkeleton)
{

	// TODO: Implement CCD IK  
	// The actual position of the end joint should match the desiredEndPos within some episilon error 

	bool result = false;

	mTarget0 = target;
	vec3 desiredEndPos = mTarget0.getGlobalTranslation();  // Get desired position of EndJoint

	int chainSize = IKchain.getSize();
	if (chainSize == 0) // There are no joints in the IK chain for manipulation
		return false;

	double epsilon = gIKEpsilon;
	int maxIterations = gIKmaxIterations;
	int numIterations = 0;

	m_pEndJoint = IKchain.getJoint(0);
	int endJointID = m_pEndJoint->getID();
	m_pBaseJoint = IKchain.getJoint(chainSize - 1);

	pIKSkeleton->copyTransforms(m_pSkeleton);

	if ((endJointID >= 0) && (endJointID < pIKSkeleton->getNumJoints()))
	{
		//TODO:
		double error = (m_pEndJoint->getGlobalTranslation() - desiredEndPos).Length();
		double lastError = (std::numeric_limits<double>::max)();//windows define max as macro, adding parenthesis make it a function pointer and () invokes the function. this way compiler doesn't recognize it as a macro
		while (error > epsilon && error < lastError - epsilon)
		{
			numIterations = 0;
			for (int i = 0; i < IKchain.getSize(); i++)
			{
				vec3 endPos = m_pEndJoint->getGlobalTranslation();
				AJoint* curJoint = IKchain.getJoint(i);

				// 1. compute axis and angle for each joint in the IK chain (distal to proximal) in global coordinates

				vec3 curPos = curJoint->getGlobalTranslation();
				double cosAngle = Dot((endPos - curPos).Normalize(), (desiredEndPos - curPos).Normalize());
				if (cosAngle > 1) cosAngle = 1; else if (cosAngle < -1) cosAngle = -1;
				double angle = acos(cosAngle);
				vec3 axis = (endPos - curPos).Cross(desiredEndPos - curPos).Normalize();

				// 2. once you have the desired axis and angle, convert axis to local joint coords 
				if (curJoint->getParent() != nullptr)
					axis = curJoint->getParent()->getLocal2Global().Inverse().Rotate(axis).Normalize();
				else
					axis = curJoint->getLocal2Global().Inverse().Rotate(axis).Normalize();

				// 3. multiply angle by corresponding joint weight value
				angle *= IKchain.getWeight(i);

				// 4. compute new local joint rotation matrix
				mat3 rot = mat3::Rotation3D(axis, angle) * curJoint->getLocalRotation();

				// 5. update joint transform
				curJoint->setLocalRotation(rot);
				curJoint->updateTransform();

				// 6. repeat same operations above for each joint in the IKchain from end to base joint
				if (++numIterations >= maxIterations) break;
			}
			lastError = error;
			error = (m_pEndJoint->getGlobalTranslation() - desiredEndPos).Length();
		}
	}
	return result;

}


bool IKController::IKSolver_PseudoInv(int endJointID, const ATarget& target)
{
	// TODO: Implement Pseudo Inverse-based IK  
	// The actual position of the end joint should match the target position after the skeleton is updated with the new joint angles

	if (!mvalidPseudoInvIKchains)
	{
		mvalidPseudoInvIKchains = createPseudoInvIKchains();
	}

	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;

	switch (endJointID)
	{
	case mLhandID:
		mLhandTarget = target;
		computePseudoInv(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		break;
	case mRhandID:
		mRhandTarget = target;
		computePseudoInv(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		break;
	case mLfootID:
		mLfootTarget = target;
		computePseudoInv(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		break;
	case mRfootID:
		mRfootTarget = target;
		computePseudoInv(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	case mRootID:
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computePseudoInv(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		computePseudoInv(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		computePseudoInv(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		computePseudoInv(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	default:
		mIKchain = createIKchain(endJointID, -1, &mIKSkeleton);
		computePseudoInv(target, mIKchain, &mIKSkeleton);
		break;
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}

int IKController::createPseudoInvIKchains()
{
	bool validChains = false;

	int desiredChainSize = -1;  // default of -1 creates IK chain of maximum length from end joint to child joint of root


								// create IK chains for Lhand, Rhand, Lfoot and Rfoot 
	mLhandIKchain = createIKchain(mLhandID, desiredChainSize, &mIKSkeleton);
	mRhandIKchain = createIKchain(mRhandID, desiredChainSize, &mIKSkeleton);
	mLfootIKchain = createIKchain(mLfootID, desiredChainSize, &mIKSkeleton);
	mRfootIKchain = createIKchain(mRfootID, desiredChainSize, &mIKSkeleton);

	if (mLhandIKchain.getSize() > 1 && mRhandIKchain.getSize() > 1 && mLfootIKchain.getSize() > 1 && mRfootIKchain.getSize() > 1)
	{
		validChains = true;

		// initalize end joint target transforms for Lhand, Rhand, Lfoot and Rfoot based on current position and orientation of joints
		mIKSkeleton.copyTransforms(m_pSkeleton);
		mLhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mLhandID)->getLocal2Global());
		mRhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mRhandID)->getLocal2Global());
		mLfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mLfootID)->getLocal2Global());
		mRfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mRfootID)->getLocal2Global());
	}

	return validChains;
}

int IKController::computePseudoInv(ATarget target, AIKchain& IKchain, ASkeleton* pIKSkeleton)
{	
	// TODO: Implement PseudoInv IK
	// The actual position of the end joint should match the desiredEndPos within some episilon error 

	bool result = false;

	mTarget0 = target;
	vec3 desiredEndPos = mTarget0.getGlobalTranslation();  // Get desired position of EndJoint

	int chainSize = IKchain.getSize();
	if (chainSize == 0) // There are no joints in the IK chain for manipulation
		return false;

	double epsilon = gIKEpsilon;
	int maxIterations = gIKmaxIterations;
	int numIterations = 0;

	m_pEndJoint = IKchain.getJoint(0);
	int endJointID = m_pEndJoint->getID();
	m_pBaseJoint = IKchain.getJoint(chainSize - 1);

	pIKSkeleton->copyTransforms(m_pSkeleton);

	if ((endJointID >= 0) && (endJointID < pIKSkeleton->getNumJoints()))
	{
		//TODO:
		vec3 deltaXd = target.getGlobalTranslation() - m_pEndJoint->getGlobalTranslation();
		double desiredLength = (target.getGlobalTranslation() - m_pBaseJoint->getGlobalTranslation()).Length();
		double totalLength = 0;
		for (int i = 0; i < IKchain.getSize()-1; i++)
		{
			totalLength += IKchain.getJoint(i)->getLocalTranslation().Length();
		}
		if (desiredLength > totalLength)
		{
			//printf("too far\n");
			target.setGlobalTranslation(
				m_pBaseJoint->getGlobalTranslation() +
				totalLength*((target.getGlobalTranslation() - m_pBaseJoint->getGlobalTranslation()).Normalize())
			);
			deltaXd = target.getGlobalTranslation() - m_pEndJoint->getGlobalTranslation();
		}
		std::vector<mat3> Bvec;
		std::vector<mat3> Lvec;
		std::vector<mat3> Jvec;
		std::vector<vec3> ThetaVec;
		//get B, L and J matrix
		for (int i = 0; i < chainSize; i++)
		{
			AJoint* joint = IKchain.getJoint(i);
			mat3 a = joint->getGlobalRotation();
			vec3 r = m_pEndJoint->getGlobalTranslation() - joint->getGlobalTranslation();
			mat3 B;
			B.SetCol(0, a.GetCol(0).Cross(r));
			B.SetCol(1, a.GetCol(1).Cross(r));
			B.SetCol(2, a.GetCol(2).Cross(r));
			vec3 euler;
			bool isEuler = joint->getLocalRotation().ToEulerAngles(mat3::ZYX, euler);
			//printf("%d-isEuler:%d\n", i, isEuler);
			ThetaVec.push_back(euler);
			vec3 Lrow0(1, 0, -sin(euler[1]));
			vec3 Lrow1(0, cos(euler[0]), sin(euler[0])*cos(euler[1]));
			vec3 Lrow2(0, -sin(euler[0]), cos(euler[0])*cos(euler[1]));
			mat3 L(Lrow0, Lrow1, Lrow2);
			mat3 J(B*L);
			Bvec.push_back(B);
			Lvec.push_back(L);
			Jvec.push_back(J);
		}
		//merge all J matrices in to one matrix
		matrix<double> J(3,chainSize*3);
		for (int i = 0; i < Jvec.size(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				for (int k = 0; k < 3; k++)
				{
					J(j, k + i * 3) = Jvec.at(i)[j][k];
				}
			}
		}
		//get pseudo inverse
		matrix<double> Jplus = ~J*((J*~J).Inv());
		//divide one matrix in to several matrices to compute theta("matrix" can not multiply with "vec3" but "mat3" can)
		std::vector<mat3> Jplusvec;
		for (int i = 0; i < Jvec.size(); i++)
		{
			mat3 temp;
			for (int j = 0; j < 3; j++)
			{
				for (int k = 0; k < 3; k++)
				{
					temp[j][k] = Jplus(j+i*3,k);
				}
			}
			Jplusvec.push_back(temp);
		}
		//compute theta and update
		for (int i = 0; i < Jplusvec.size(); i++)
		{
			vec3 deltaTheta = Jplusvec.at(i) * deltaXd;
			AJoint* joint = IKchain.getJoint(i);
			mat3 temp;
			temp.FromEulerAngles(mat3::ZYX, deltaTheta + ThetaVec.at(i));
			//std::cout << joint->getName() << joint->getRotationOrder() << std::endl;
			joint->setLocalRotation(temp);
			joint->updateTransform();
		}
	}

	return result;
}

bool IKController::IKSolver_Other(int endJointID, const ATarget& target)
{
	
	bool result = false;
	
	// TODO: Put Optional IK implementation or enhancements here
	 
	return result;
}