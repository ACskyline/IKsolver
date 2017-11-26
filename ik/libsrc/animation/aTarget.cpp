#include "aTarget.h"

#pragma warning (disable : 4018)



/////////////////////////////////////////////////////////////////////////////////////////////////////////

ATarget::ATarget()
{
	mValid = true;
}

ATarget::~ATarget()
{

}

void ATarget::update()
{
	if (getParent() == NULL)
		mLocal2Global = mLocal2Parent;
	else
	{
		AJoint* pJoint = getParent();
		ATransform targetLocal2Global = pJoint->getLocal2Global()*getLocal2Parent();
		setLocal2Global(targetLocal2Global);
	}
}

bool ATarget::isValid() const
{
	return mValid;
}

void ATarget::setValid(bool valid)
{
	mValid = valid;
}


void ATarget::setLocal2Parent(const ATransform& targetTransform)
{
	AJoint::setLocal2Parent(targetTransform);
	if (getParent() == NULL)
		mLocal2Global = mLocal2Parent;
	else
	{
		AJoint* pJoint = getParent();
		ATransform local2Global = pJoint->getLocal2Global()*getLocal2Parent();
		AJoint::setLocal2Global(local2Global);
	}
}

void ATarget::setLocalTranslation(const vec3& targetTranslation)
{
	mLocal2Parent.m_translation = targetTranslation;
	if (getParent() == NULL)
		mLocal2Global = mLocal2Parent;
	else
	{
		AJoint* pJoint = getParent();
		ATransform local2Global = pJoint->getLocal2Global()*getLocal2Parent();
		AJoint::setLocal2Global(local2Global);
	}
}

void ATarget::setLocalRotation(const mat3& targetRotation)
{
	mLocal2Parent.m_rotation = targetRotation;
	if (getParent() == NULL)
		mLocal2Global = mLocal2Parent;
	else
	{
		AJoint* pJoint = getParent();
		ATransform local2Global = pJoint->getLocal2Global()*getLocal2Parent();
		AJoint::setLocal2Global(local2Global);
	}
}