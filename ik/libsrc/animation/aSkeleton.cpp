#include "aSkeleton.h"

#pragma warning(disable : 4018)


/****************************************************************
*
*    	    Skeleton Hierarchy functions
*
****************************************************************/

ASkeleton::ASkeleton()

{
	mRoot = NULL;
	mJoints.clear();
}

ASkeleton::ASkeleton(const ASkeleton& inputSkeleton)
{
	*this = inputSkeleton;
}

ASkeleton& ASkeleton::operator = (const ASkeleton& inputSkeleton)
{
	// Performs a deep copy which includes joint hiearchy (mJoints) and transform data	
	if (&inputSkeleton == this)
	{
		return *this;
	}
	copyHierarchy(&inputSkeleton);
	copyTransforms(&inputSkeleton);

	return *this;

}
void ASkeleton::copyHierarchy(const ASkeleton* inputSkeleton)
{
	// Performs a deep copy which includes joint hiearchy (mJoints) and transform data	
	if (inputSkeleton == this)
	{
		return;
	}

	mJoints.clear();
	mRoot = 0;

	// Copy joints
	for (unsigned int i = 0; i < inputSkeleton->mJoints.size(); i++)
	{
		AJoint* jointnode = new AJoint(*(inputSkeleton->mJoints[i]));
		mJoints.push_back(jointnode);
		//std::cout << "Copy " << jointnode->GetName() << std::endl;
	}

	// Copy parent/children relationships
	for (unsigned int i = 0; i < inputSkeleton->mJoints.size(); i++)
	{
		AJoint* jointnode = inputSkeleton->mJoints[i];
		if (jointnode->getParent())
		{
			AJoint* parent = mJoints[jointnode->getParent()->getID()];
			mJoints[i]->setParent(parent);
		}
		else
		{
			mRoot = mJoints[i];
			mRoot->setParent(0);
		}

		for (unsigned int j = 0; j < jointnode->getNumChildren(); j++)
		{
			AJoint* child = mJoints[jointnode->getChildAt(j)->getID()];
			mJoints[i]->appendChild(child);
		}
	}
	mJointCount = mJoints.size();
}


void ASkeleton::copyTransforms(const ASkeleton* inputSkeleton)
{
	// assumes joint hiearchy (contained in mJoints) of input skeleton is the same.  only copies joint transform data
	if ((inputSkeleton == this) )
	{
		return;
	}

	if (this->getNumJoints() != inputSkeleton->getNumJoints())
		 assert(0);
	else mJointCount = inputSkeleton->getNumJoints();

	AJoint* pJointInput;
	AJoint* pJoint;
	 
	for (int i = 0; i < mJointCount; i++) {
		pJoint = this->mJoints[i];
		pJointInput = inputSkeleton->mJoints[i];

		if (pJoint) {
			pJoint->setLocal2Global(pJointInput->getLocal2Global());
			pJoint->setLocal2Parent(pJointInput->getLocal2Parent());
		}
		else assert(pJoint);

	}
}

ASkeleton::~ASkeleton()
{
	clear();
}

void ASkeleton::clear()
{
	mRoot = NULL;
	mJoints.clear();
}

void ASkeleton::update()
{
	if (!mRoot) return; // Nothing loaded

	// TODO: Update  Joint Transforms recursively, starting at the root
	mRoot->updateTransform();
}

AJoint* ASkeleton::getJointByName(const std::string& name) const
{
	for (int i = 0; i < mJoints.size(); i++)
	{
		if (name == mJoints[i]->getName())
			return mJoints[i];
	}

	return NULL;
}

AJoint* ASkeleton::getJointByID(unsigned int id) const
{
	assert(id >= 0 && id < mJoints.size());
	return mJoints[id];
}

AJoint* ASkeleton::getRootNode() const
{
	return mRoot;
}

void ASkeleton::addJoint(AJoint* jointnode, bool isRoot)
{
	jointnode->setID(mJoints.size());
	mJoints.push_back(jointnode);
	if (isRoot) mRoot = jointnode;
}

void ASkeleton::deleteJoint(const std::string& name)
{
	AJoint* jointnode = getJointByName(name);
	if (!jointnode) return; // no work to do

	for (int i = 0; i < jointnode->getNumChildren(); i++)
	{
		AJoint* child = jointnode->getChildAt(i);
		AJoint::Detach(jointnode, child);
		deleteJoint(child->getName());
	}

	// Re-assign ids and delete orphans
	AJoint* parent = jointnode->getParent();
	if (parent)
	{
		AJoint::Detach(parent, jointnode);
		if (parent->getNumChildren() == 0) parent->setNumChannels(0);
	}
	for (int i = jointnode->getID(); i < mJoints.size() - 1; i++)
	{
		mJoints[i] = mJoints[i + 1];
		mJoints[i]->setID(i);
	}
	mJoints.resize(mJoints.size() - 1);
	delete jointnode;
}
