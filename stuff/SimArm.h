#pragma once

#include "sim/SimCharSoftFall.h"


class cSimArm : public cSimCharacter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eJoint
	{
		eJointRoot,
		eJointLink0,
		eJointLink1,
		eJointLink2,
		eJointLinkEnd,
		eJointMax,
		eJointInvalid
	};
	
	cSimArm();
	virtual ~cSimArm();

	virtual bool IsEndEffector(int idx) const;

protected:
	virtual short GetPartColGroup(int part_id) const;
	virtual short GetPartColMask(int part_id) const;
};