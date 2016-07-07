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