#pragma once

#include "sim/SimCharSoftFall.h"


class cSimArm : public cSimCharacter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cSimArm();
	virtual ~cSimArm();

	virtual bool IsEndEffector(int idx) const;
	virtual void SetPose(const Eigen::VectorXd& pose);
	virtual void SetVel(const Eigen::VectorXd& vel);

protected:
	virtual short GetPartColGroup(int part_id) const;
	virtual short GetPartColMask(int part_id) const;
};