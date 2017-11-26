#include "aJoint.h"

#pragma warning(disable : 4018)


/****************************************************************
*
*    	    Transform Node Functions
*
****************************************************************/
AJoint::AJoint() :
mId(-1),
mName(""),
mChannelCount(0),
mRotOrder("xyz"),
mDirty(false),
mParent(0),
mChildren(),
mLocal2Parent(),
mLocal2Global()
{

}

AJoint::AJoint(const std::string& name) :
mId(-1),
mName(name),
mChannelCount(0),
mRotOrder("xyz"),
mDirty(false),
mParent(0),
mChildren(),
mLocal2Parent(),
mLocal2Global()
{

}

AJoint::AJoint(const AJoint& jointnode)
{
	*this = jointnode;
}

AJoint& AJoint::operator=(const AJoint& orig)
{
	if (&orig == this)
	{
		return *this;
	}

	// copy everything except parents/children
	mParent = 0;
	mChildren.clear();
	mDirty = true;

	mId = orig.mId;
	mName = orig.mName;
	mChannelCount = orig.mChannelCount;
	mRotOrder = orig.mRotOrder;
	mLocal2Parent = orig.mLocal2Parent;
	mLocal2Global = orig.mLocal2Global;

	return *this;
}

AJoint::~AJoint()
{

}

AJoint* AJoint::getParent()
{
	return mParent;
}
void AJoint::setParent(AJoint* parent)
{
	mParent = parent;
}

unsigned int AJoint::getNumChildren() const
{
	return mChildren.size();
}
AJoint* AJoint::getChildAt(unsigned int index)
{
	assert(index >= 0 && index < mChildren.size());
	return mChildren[index];
}
void AJoint::appendChild(AJoint* child)
{
	mChildren.push_back(child);
}

void AJoint::setName(const std::string& name)
{
	mName = name;
}
void AJoint::setID(int id)
{
	mId = id;
	if (strncmp("Site", mName.c_str(), 4) == 0)
	{
		char dummy[32];
		sprintf_s(dummy, "Site%d", mId);
		mName = dummy;
	}
}
void AJoint::setNumChannels(unsigned int count)
{
	mChannelCount = count;
}
void AJoint::setRotationOrder(const std::string& _order)
{
	std::string order = _order;

	if (order.find("Zrotation Xrotation Yrotation") != std::string::npos) mRotOrder = "zxy";
	else if (order.find("Zrotation Yrotation Xrotation") != std::string::npos) mRotOrder = "zyx";
	else if (order.find("Xrotation Yrotation Zrotation") != std::string::npos) mRotOrder = "xyz";
	else if (order.find("Xrotation Zrotation Yrotation") != std::string::npos) mRotOrder = "xzy";
	else if (order.find("Yrotation Xrotation Zrotation") != std::string::npos) mRotOrder = "yxz";
	else if (order.find("Yrotation Zrotation Xrotation") != std::string::npos) mRotOrder = "yzx";
	else mRotOrder = order;

}

void AJoint::setLocal2Parent(const ATransform& transform)
{
	mLocal2Parent = transform;
}
void AJoint::setLocalTranslation(const vec3& translation)
{
	mLocal2Parent.m_translation = translation;
}
void AJoint::setLocalRotation(const mat3& rotation)
{
	mLocal2Parent.m_rotation = rotation;
}


void AJoint::setLocal2Global(const ATransform& transform)
{
	mLocal2Global = transform;
}

void AJoint::setGlobalTranslation(const vec3& translation)  // new function
{
	mLocal2Global.m_translation = translation;
}

void AJoint::setGlobalRotation(const mat3& rotation) // new function
{
	mLocal2Global.m_rotation = rotation;
}



int AJoint::getID() const
{
	return mId;
}
const std::string& AJoint::getName() const
{
	return mName;
}
unsigned int AJoint::getNumChannels() const
{
	return mChannelCount;
}

const std::string& AJoint::getRotationOrder() const
{
	return mRotOrder;
}


const ATransform& AJoint::getLocal2Parent() const
{
	return mLocal2Parent;
}
const vec3& AJoint::getLocalTranslation() const
{
	return mLocal2Parent.m_translation;
}
const mat3& AJoint::getLocalRotation() const
{
	return mLocal2Parent.m_rotation;
}


const ATransform& AJoint::getLocal2Global() const
{
	return mLocal2Global;
}
const vec3& AJoint::getGlobalTranslation() const
{
	return mLocal2Global.m_translation;
}
const mat3& AJoint::getGlobalRotation() const
{
	return mLocal2Global.m_rotation;
}

void AJoint::updateTransform()
{
	mLocal2Global = ATransform();

	// TODO: Compute mLocal2Global for the joint, which allows vectors to be transformed from local coordinates to world coordinates

	if (mParent != nullptr)
	{
		mLocal2Global = mParent->mLocal2Global * mLocal2Parent;
	}
	else
	{
		mLocal2Global = mLocal2Parent;
	}

	// TODO: Also update mLocal2Global for each of the children

	for (int i = 0; i < mChildren.size(); i++)
	{
		mChildren[i]->updateTransform();
	}

}

void AJoint::Attach(AJoint* pParent, AJoint* pChild)
{
	if (pChild)
	{
		AJoint* pOldParent = pChild->mParent;
		if (pOldParent)
		{
			// erase the child from old parent's children list
			std::vector<AJoint*>::iterator index0 = pOldParent->mChildren.begin();
			for (int i = 0; i < pOldParent->mChildren.size(); i++)
			{
				AJoint* pOldParentChild = pOldParent->mChildren[i];
				if (pOldParentChild == pChild)
				{
					pOldParent->mChildren.erase(index0 + i);
				}

			}

		}
		// Set the new parent
		pChild->mParent = pParent;
		// Add child to new parent's children list
		if (pParent)
		{
			pParent->mChildren.push_back(pChild);
		}
	}
}

void AJoint::Detach(AJoint* pParent, AJoint* pChild)
{
	if (pChild && pChild->mParent == pParent)
	{
		if (pParent)
		{
			// erase the child from parent's children list
			for (int i = 0; i < pParent->mChildren.size(); i++)
			{
				if (pParent->mChildren[i] == pChild)
				{
					for (int j = i; j < pParent->mChildren.size() - 1; j++)
					{
						pParent->mChildren[j] = pParent->mChildren[j + 1];
					}
					pParent->mChildren.resize(pParent->mChildren.size() - 1);
					break;
				}
			}
		}
		pChild->mParent = NULL;
	}
}

