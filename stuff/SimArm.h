#pragma once

#include "sim/SimCharSoftFall.h"


class cSimArm : public cSimCharacter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cSimArm();
	virtual ~cSimArm();

	virtual bool IsEndEffector(int idx) const;

protected:
	virtual short GetPartColGroup(int part_id) const;
	virtual short GetPartColMask(int part_id) const;
};