#ifndef AJOINT_H_
#define AJOINT_H_

#include "aTransform.h"
#include <vector>


class AJoint
{
public:
	AJoint();
	AJoint(const std::string& name);
	AJoint(const AJoint& node);
	virtual AJoint& operator=(const AJoint& node);

	virtual ~AJoint();

	AJoint* getParent();
	void setParent(AJoint* parent);

	unsigned int getNumChildren() const;
	AJoint* getChildAt(unsigned int index);
	void appendChild(AJoint* child);

	void updateTransform();

	void setName(const std::string& name);
	void setID(int id);
	void setNumChannels(unsigned int count);
	void setRotationOrder(const std::string& order);
 
	void setLocal2Global(const ATransform& transform);  // new function
	void setGlobalTranslation(const vec3& translation);  // new function
	void setGlobalRotation(const mat3& rotation);        // new function
	void setLocal2Parent(const ATransform& transform);
	void setLocalTranslation(const vec3& translation);
	void setLocalRotation(const mat3& rotation);

	int getID() const;
	const std::string& getName() const;
	unsigned int getNumChannels() const;
	const std::string& getRotationOrder() const;

	const ATransform& getLocal2Parent() const;
	const vec3& getLocalTranslation() const;
	const mat3& getLocalRotation() const;

	const ATransform& getLocal2Global() const;
	const vec3& getGlobalTranslation() const;
	const mat3& getGlobalRotation() const;

	static void Attach(AJoint* pParent, AJoint* pChild);
	static void Detach(AJoint* pParent, AJoint* pChild);

protected:
	int mId;
	std::string mName;
	unsigned int mChannelCount;
	std::string mRotOrder;
	bool mDirty;

	AJoint* mParent;
	std::vector<AJoint*> mChildren;

	ATransform mLocal2Parent;
	ATransform mLocal2Global;
};


#endif