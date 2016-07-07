#include "SimArm.h"
#include <iostream>

const short gColFlag = cContactManager::gFlagNone;

cSimArm::cSimArm()
{
}

cSimArm::~cSimArm()
{
}

short cSimArm::GetPartColGroup(int part_id) const
{
	return gColFlag;
}

short cSimArm::GetPartColMask(int part_id) const
{
	return gColFlag;
}

bool cSimArm::IsEndEffector(int idx) const
{
	return (idx == GetNumJoints() - 1);
}

void cSimArm::SetPose(const Eigen::VectorXd& pose)
{
	cSimCharacter::SetPose(pose);
	if (mController != nullptr)
	{
		mController->Reset(); // hack
	}
}

void cSimArm::SetVel(const Eigen::VectorXd& vel)
{
	cSimCharacter::SetVel(vel);
	if (mController != nullptr)
	{
		mController->Reset(); // hack
	}
}