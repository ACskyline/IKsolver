#ifndef ASKELETON_H_
#define ASKELETON_H_

#include "aTransform.h"
#include "aJoint.h"
#include <vector>

// Class for createing hierarchies of joints


class ASkeleton
{
public:
	ASkeleton();
	ASkeleton(const ASkeleton& inputSkeleton); // Deep copy

	virtual ~ASkeleton();
	virtual void update();
	virtual void clear();

	// new/revised functions
	virtual ASkeleton& operator=(const ASkeleton& inputSkeleton); // Copies both joint hiearchy and transforms of the input skeleton
	virtual void copyHierarchy(const ASkeleton* inputSkeleton);  // copies the input skeleton joint hierarchy
	virtual void copyTransforms(const ASkeleton* inputSkeleton); // assumes the same joint hierarchy as input skeleton and copies joint transforms
	// end new/ revised functions

	AJoint* getJointByName(const std::string& name) const;
	AJoint* getJointByID(unsigned int id) const;
	AJoint* getRootNode() const;

	void addJoint(AJoint* jointnode, bool isRoot = false);
	void deleteJoint(const std::string& name);

	size_t getNumJoints() const { return mJoints.size(); }

protected:
	std::vector<AJoint*> mJoints;
	int mJointCount = 0;
	AJoint* mRoot;
};


#endif