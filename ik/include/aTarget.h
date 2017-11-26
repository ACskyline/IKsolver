#ifndef ATARGET_H_
#define ATARGET_H_

#pragma once


#include "aJoint.h"

class ATarget : public AJoint
{
public:
	ATarget();
	virtual ~ATarget();

	bool isValid() const;
	void setValid(bool valid);
	void setLocal2Parent(const ATransform& transform);
	void setLocalTranslation(const vec3& translation);
	void setLocalRotation(const mat3& rotation);

	void update();

protected:
	bool mValid;

};

#endif